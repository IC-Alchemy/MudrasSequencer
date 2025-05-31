// ==========================
//      Pico2DSP2CoreWorks
//   Refactored & Organized
// ==========================

/* 
 * Main firmware for Pico2DSP2CoreWorks
 * - Audio synthesis and output (I2S)
 * - Sequencer and OLED visualization
 * - Touch matrix input (MPR121)
 * - MIDI and clock sync
 * 
 * Refactored for readability, maintainability, and organization.
 */

// -----------------------------------------------------------------------------
// 1. INCLUDES & DEFINES
// -----------------------------------------------------------------------------

// --- Audio & DSP ---
#include "src/audio/audio.h"
#include "src/audio/audio_i2s.h"
#include "src/audio/audio_pins.h"
#include "src/dsp/adsr.h"
#include "src/dsp/oscillator.h"
#include "src/dsp/phasor.h"
#include <cmath>
#include <cstdint>

// --- Sequencer ---
#include "src/sequencer/Sequencer.h"

// --- Display (OLED) ---
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- MIDI & USB ---
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <uClock.h>

// --- Touch Matrix ---
#include "src/matrix/Matrix.h"
#include <Adafruit_MPR121.h> // https://github.com/adafruit/Adafruit_MPR121_Library>

// -----------------------------------------------------------------------------
// 2. CONSTANTS & GLOBALS
// -----------------------------------------------------------------------------

// --- I2S Pin Configuration ---
#define PICO_AUDIO_I2S_DATA_PIN 15
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16

// --- OLED Display ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
#define NOTE_LENGTH        12 // min: 1 max: 23 DO NOT EDIT BEYOND!!! 12 = 50% on 96ppqn, same as original tb303. 62.5% for triplets time signature
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// --- Sequencer ---
Sequencer seq;
volatile uint8_t sequencer_display_page = 0; // 0 = Note Page, 1 = Gate Page

// --- MIDI & Clock ---
Adafruit_USBD_MIDI raw_usb_midi;
midi::SerialMIDI<Adafruit_USBD_MIDI> serial_usb_midi(raw_usb_midi);
midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>> usb_midi(serial_usb_midi);
uint8_t bpm_blink_timer = 1;

// --- Touch Matrix ---
const int PIN_TOUCH_IRQ = 6;
Adafruit_MPR121 touchSensor = Adafruit_MPR121();

// --- Multicore Communication ---
volatile int note1 = 48, note2 = 48, note3 = 48, note4 = 48, note5 = 48;
volatile bool trig1, trig2, trigenv1, trigenv2, dualEnvFlag;
volatile bool buttonEventFlag = false;
volatile uint8_t buttonEventIndex = 0;
volatile uint8_t buttonEventType = 0;

// --- Scale Table ---
int scale[] = {
  0,2,4,5,7,9,10,12,14,16,17,19,21,22,24,26,28,29,31,33,34,36,38,40,41,43,46,46,48,50,52,53,55,57,58,60,60,60,60,60,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48
};

// --- Audio & Synth ---
constexpr float SAMPLE_RATE = 44100.0f;
constexpr float OSC_SUM_SCALING = 0.1f;
constexpr float INT16_MAX_AS_FLOAT = 32767.0f;
constexpr float INT16_MIN_AS_FLOAT = -32768.0f;
constexpr int NUM_AUDIO_BUFFERS = 3;
constexpr int SAMPLES_PER_BUFFER = 256;
float baseFreq = 110.0f; // Hz
constexpr float OSC_DETUNE_FACTOR = 1.01f;

// --- Oscillators & Envelopes ---
daisysp::Oscillator osc1, osc2, osc3, osc4, osc5, osc6, osc7, osc8;
daisysp::Adsr env1, env2;

// --- Audio Buffer Pool ---
audio_buffer_pool_t *producer_pool = nullptr;

// --- Timing ---
unsigned long previousMillis = 0;
const long interval = 333; // ms

// -----------------------------------------------------------------------------
// 3. UTILITY FUNCTIONS
// -----------------------------------------------------------------------------

/**
 * @brief Converts a floating-point sample to int16_t with scaling, rounding, and clamping.
 */
static inline int16_t convertSampleToInt16(float sample) {
    float scaled = sample * INT16_MAX_AS_FLOAT;
    scaled = roundf(scaled);
    scaled = daisysp::fclamp(scaled, INT16_MIN_AS_FLOAT, INT16_MAX_AS_FLOAT);
    return static_cast<int16_t>(scaled);
}

// -----------------------------------------------------------------------------
// 4. DISPLAY: OLED SEQUENCER VISUALIZATION
// -----------------------------------------------------------------------------

void drawSequencerOLED(const SequencerState& seqState) {
  display.clearDisplay();
  display.setTextSize(1);

  // Layout: 8 steps per row
  const uint8_t steps_per_row = 8;
  const uint8_t row_height = 16; // pixels between rows
  const uint8_t note_y_offset = 8; // vertical offset for first row (note page)
  const uint8_t gate_y_offset = 14; // vertical offset for first row (gate page)
  const uint8_t underline_y_offset = 20; // y position for playhead underline

  // Calculate playhead position in grid
  uint8_t playhead_row = seqState.playhead / steps_per_row;
  uint8_t playhead_col = seqState.playhead % steps_per_row;
  int playhead_x = playhead_col * 16; // wider cell for numbers/circles
  int playhead_y = (sequencer_display_page == 0)
                     ? (note_y_offset + playhead_row * row_height + 10)
                     : (gate_y_offset + playhead_row * row_height + 8);

  // Draw playhead underline (under the correct step)
  display.drawLine(playhead_x, playhead_y, playhead_x + 15, playhead_y, SSD1306_WHITE);

  if (sequencer_display_page == 0) {
    // Note Page: show only MIDI note numbers, 8 per row
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
      uint8_t row = i / steps_per_row;
      uint8_t col = i % steps_per_row;
      int x = col * 16;
      int y = note_y_offset + row * row_height;
      display.setCursor(x, y);
      display.print(seqState.steps[i].note);
    }
    display.setCursor(0, 56);
    display.print("Page: Note");
  } else {
    // Gate Page: show only gate state (● = ON, ○ = OFF), 8 per row
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
      uint8_t row = i / steps_per_row;
      uint8_t col = i % steps_per_row;
      int x = col * 16 + 7;
      int y = gate_y_offset + row * row_height;
      if (seqState.steps[i].gate)
        display.fillCircle(x, y, 4, SSD1306_WHITE);
      else
        display.drawCircle(x, y, 4, SSD1306_WHITE);
    }
    display.setCursor(0, 56);
    display.print("Page: Gate");
  }

  display.display();
}

// -----------------------------------------------------------------------------
// 5. AUDIO: SYNTHESIS & BUFFER FILLING
// -----------------------------------------------------------------------------

/**
 * @brief Fills an audio buffer with synthesized stereo samples.
 */
void fill_audio_buffer(audio_buffer_t *buffer) {
  int N = buffer->max_sample_count;
  int16_t *out = reinterpret_cast<int16_t *>(buffer->buffer->bytes);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // (LED blink or other timed events could go here)
  }

  for (int i = 0; i < N; ++i) {
    osc1.SetFreq(daisysp::mtof(note1 + 36));
    osc2.SetFreq(daisysp::mtof(note2 + 36));
    osc3.SetFreq(daisysp::mtof(note3 + 36));
    osc4.SetFreq(daisysp::mtof(note2 - 12));

    float current_out1 = env1.Process(trigenv1);
    float current_out2 = env2.Process(trigenv2);

    float osc111 = osc1.Process();
    float osc222 = osc2.Process();
    float osc333 = osc3.Process();
    float osc444 = osc4.Process();

    float out1 = (osc111 + osc222) * current_out1;
    float out2 = (osc333 + osc444) * current_out2;

    float sumL = out1 * 0.5f;
    float sumR = out2 * 0.5f;

    int16_t intSampleL = convertSampleToInt16(sumL);
    int16_t intSampleR = convertSampleToInt16(sumR);

    out[2 * i + 0] = intSampleL;
    out[2 * i + 1] = intSampleR;
  }
  buffer->sample_count = N;
  // Set up page toggle button (e.g., pin 2)
  pinMode(2, INPUT_PULLUP); // (Redundant, but kept for compatibility)
}

// -----------------------------------------------------------------------------
// 6. AUDIO: OSCILLATOR & ENVELOPE INITIALIZATION
// -----------------------------------------------------------------------------

void initOscillators() {
  osc1.Init(SAMPLE_RATE); osc2.Init(SAMPLE_RATE); osc3.Init(SAMPLE_RATE); osc4.Init(SAMPLE_RATE);
  osc5.Init(SAMPLE_RATE); osc6.Init(SAMPLE_RATE); osc7.Init(SAMPLE_RATE); osc8.Init(SAMPLE_RATE);
  env1.Init(SAMPLE_RATE); env2.Init(SAMPLE_RATE);

  env1.SetReleaseTime(.061f);
  env1.SetAttackTime(0.006f);
  env2.SetAttackTime(0.001f);
  env1.SetDecayTime(0.1f);
  env2.SetDecayTime(0.1f);
  env1.SetSustainLevel(0.4f);
  env2.SetSustainLevel(0.4f);
  env2.SetReleaseTime(0.07f);

  // Set initial waveform for all oscillators
  osc1.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  osc2.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  osc3.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  osc4.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  osc5.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
}

// -----------------------------------------------------------------------------
// 7. MIDI & CLOCK HANDLERS
// -----------------------------------------------------------------------------

void ledOn()   { /* Implement as needed */ }
void ledOff()  { /* Implement as needed */ }

void handle_bpm_led(uint32_t tick) {
  // BPM led indicator
  if (!(tick % 96) || (tick == 1)) {
    bpm_blink_timer = 8;
    ledOn();
  } else if (!(tick % 24)) {
    bpm_blink_timer = 1;
    ledOn();
  } else if (!(tick % bpm_blink_timer)) {
    ledOff();
  }
}

void onSync24Callback(uint32_t tick) {
  usb_midi.sendRealTime(midi::Clock);
  handle_bpm_led(tick);
}

void onClockStart() {
  usb_midi.sendRealTime(midi::Start);
}

/**
 * Callback invoked on each sequencer step event.
 * Advances the sequencer and updates the OLED display if the state changes.
 */
/**
 * Monophonic step callback: handles rest, note length, and MIDI for a single note.
 * Preserves rests, glide (if implemented), note length, and MIDI handling.
 */
void onStepCallback(uint32_t tick) {
    uint16_t step = tick % SEQUENCER_NUM_STEPS;
    uint16_t length = NOTE_LENGTH;
    const auto& currentStep = seq.getStep(step);

    // Handle rest: do nothing if this step is a rest
    if (currentStep.state == StepState::OFF) {
        // Optionally send note-off if a note was playing
        if (seq.getLastNote() >= 0) {
            usb_midi.sendNoteOff(seq.getLastNote(), 0, 1);
            seq.setLastNote(-1);
        }
        return;
    }

    // Glide and note length calculation (look ahead for glide)
    for (uint16_t i = 1; i < SEQUENCER_NUM_STEPS; ++i) {
        uint16_t nextStep = (step + i) % SEQUENCER_NUM_STEPS;
        const auto& s = seq.getStep(nextStep);
        // If you add a 'glide' property to Step, check it here:
        // if (s.glide && s.state == StepState::ON) {
        if (s.state == StepState::ON) {
            // If you want to support glide, add logic here
            length = NOTE_LENGTH + (i * 24);
            break;
        }
    }

    // Monophonic: always send note-off for the last note before note-on
    if (seq.getLastNote() >= 0) {
        usb_midi.sendNoteOff(seq.getLastNote(), 0, 1);
    }

    // Send note-on for the current note
    usb_midi.sendNoteOn(currentStep.note, 100, 1); // Use accent velocity if needed
    seq.setLastNote(currentStep.note);

    // Set oscillator frequency and trigger envelope as in your Sequencer::advanceStep()
   // setOscillatorFrequency(currentStep.note);
    //triggerEnvelope();

    // TODO: Start a timer for note length to send note-off after 'length'
    // This may require integration with your main loop or a timer interrupt.

    // OLED update (optional, as before)
    auto state = seq.getState();
   // static decltype(state) lastState;
    //if (state != lastState) {
        drawSequencerOLED(state);
       // lastState = state;
  //  }
}
void onClockStop() {
  seq.stop();
  usb_midi.sendRealTime(midi::Stop);
}

// -----------------------------------------------------------------------------
// 8. ARDUINO SETUP FUNCTIONS
// -----------------------------------------------------------------------------

void setup() {
  // --- Synth & Envelope ---
  initOscillators();

  // --- OLED Display ---
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    for (;;); // Display initialization failed, halt
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Sequencer OLED Ready");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();

  // --- I2S Output Initialization ---
  static audio_format_t my_audio_format = {
    .sample_freq = (uint32_t)SAMPLE_RATE,
    .format = AUDIO_BUFFER_FORMAT_PCM_S16,
    .channel_count = 2
  };
  static audio_buffer_format_t my_buffer_format = {
    .format = &my_audio_format,
    .sample_stride = 4 // 2 channels * 2 bytes
  };
  producer_pool = audio_new_producer_pool(&my_buffer_format, NUM_AUDIO_BUFFERS, SAMPLES_PER_BUFFER);

  audio_i2s_config_t i2s_config = {
    .data_pin = PICO_AUDIO_I2S_DATA_PIN,
    .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
    .dma_channel = 0,
    .pio_sm = 0
  };
  audio_i2s_setup(&my_audio_format, &i2s_config);
  audio_i2s_connect(producer_pool);
  audio_i2s_set_enabled(true);

  // --- Initial Envelope Triggers ---
 

  // --- Initial OLED Sequencer State ---
    drawSequencerOLED(seq.getState());

}
void matrixEventHandler(const MatrixButtonEvent &evt) {
    if (evt.type == MATRIX_BUTTON_PRESSED && evt.buttonIndex < 16) {
        seq.toggleStep(evt.buttonIndex);
        // Optionally, update the OLED display immediately:
        drawSequencerOLED(seq.getState());
    }
}
void setup1() {
  Serial.begin(9600);
  delay(55);
  Serial.println("FastRead.ino: Setup starting...");

#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  TinyUSB_Device_Init(0);
#endif

  usb_midi.begin(MIDI_CHANNEL_OMNI);

  // Initialize builtin led for clock timer blinking
  // (Implement initBlinkLed() as needed)
  // initBlinkLed();

  // Setup clock system
  uClock.init();
  uClock.setOnSync24(onSync24Callback);
  uClock.setOnClockStart(onClockStart);
  uClock.setOnClockStop(onClockStop);
  uClock.setOnStep(onStepCallback);
  uClock.setTempo(126);
  uClock.start();

  // Touch sensor
  if (!touchSensor.begin()) {
    Serial.println("ERROR: MPR121 not found. Check wiring and I2C address!");
    while (1) { delay(55); }
  } else {
    Serial.println("MPR121 initialized successfully.");
  }

  Matrix_init(&touchSensor);
  pinMode(PIN_TOUCH_IRQ, INPUT);

  Serial.println("Setup complete.");
// --- Matrix to Sequencer Step Toggling Integration ---
// This handler toggles sequencer steps 0-15 when the corresponding matrix button is pressed.


// Register the handler after Matrix_init in setup1:
struct MatrixEventHandlerRegistrar {
    MatrixEventHandlerRegistrar() {
        Matrix_setEventHandler(matrixEventHandler);
    }
} _matrixEventHandlerRegistrar;
}

// -----------------------------------------------------------------------------
// 9. MAIN LOOPS
// -----------------------------------------------------------------------------

void loop() {
  // --- Inter-core event receive and handler dispatch ---
  if (buttonEventFlag) {
    uint8_t idx = buttonEventIndex;
    uint8_t type = buttonEventType;
    buttonEventFlag = false;
/*
    if (type == 1) {
      usbMIDI.sendControlChange(idx, 1, CHANNEL);
      trigenv1 = 1;
      trigenv2 = 1;
      note1 = idx;
      note2 = idx + 12;
      note3 = idx + 7;
      note4 = idx + 12;
    } else if (type == 0) {
      trigenv1 = false;
      trigenv2 = 0;
    }
  }
*/
  }
  // --- Audio Buffer Output ---
  audio_buffer_t *buf = take_audio_buffer(producer_pool, true);
  if (buf) {
    fill_audio_buffer(buf);
    give_audio_buffer(producer_pool, buf);
  }
  // (Optional: delay(1); for low-priority tasks)
}

void loop1() {
  static uint32_t lastScanMs = 0;
  static uint32_t lastDebugMs = 0;
  static uint32_t lastMatrixPrintMs = 0;
  static bool prevButtonState[MATRIX_BUTTON_COUNT] = {0};
  uint32_t nowMs = millis();

  usb_midi.read();

  // Debug message every 1000ms
  if (nowMs - lastDebugMs >= 1000) {
    Serial.println("Scanning...");
    lastDebugMs = nowMs;
  }

  // Scan at regular intervals (every 1ms)
  if (nowMs - lastScanMs >= 1) {
    lastScanMs = nowMs;
    Matrix_scan();

    // Event detection and inter-core communication
    for (uint8_t idx = 0; idx < MATRIX_BUTTON_COUNT; ++idx) {
      bool curr = Matrix_getButtonState(idx);
      if (curr != prevButtonState[idx]) {
        if (!buttonEventFlag) {
       
      }
    }
  }

  // Print button matrix every 100ms
  if (nowMs - lastMatrixPrintMs >= 100) {
    Matrix_printState();
    lastMatrixPrintMs = nowMs;
  }
}}

// -----------------------------------------------------------------------------
// 10. END OF FILE
// -----------------------------------------------------------------------------
