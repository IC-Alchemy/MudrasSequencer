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
#include "Adafruit_VL53L0X.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>


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
//  Distance Sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int raw_mm = 0;
int mm = 0;

// --- I2S Pin Configuration ---
#define PICO_AUDIO_I2S_DATA_PIN 15
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16

// --- OLED Display ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI 10
#define OLED_CLK 11
#define OLED_DC 12
#define OLED_CS 13
#define OLED_RESET 9
#define NOTE_LENGTH                                                            \
  4 // min: 1 max: 23 DO NOT EDIT BEYOND!!! 12 = 50% on 96ppqn, same as original
    // tb303. 62.5% for triplets time signature
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK,
                         OLED_DC, OLED_RESET, OLED_CS);

// --- Sequencer ---
Sequencer seq;
volatile uint8_t sequencer_display_page = 0; // 0 = Note Page, 1 = Gate Page

// --- MIDI & Clock ---
Adafruit_USBD_MIDI raw_usb_midi;
midi::SerialMIDI<Adafruit_USBD_MIDI> serial_usb_midi(raw_usb_midi);
midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>>
    usb_midi(serial_usb_midi);
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

int scale[7][24] = {
    {0,  2,  4,  5,  7,  9,  11, 12, 14, 16, 17, 19,
     21, 23, 24, 26, 28, 29, 31, 33, 35, 36, 36, 36}, //  Major
    {0,  2,  3,  5,  7,  8,  11, 12, 14, 15, 17, 19,
     20, 23, 24, 26, 27, 29, 31, 32, 35, 36, 36, 36}, //  Harmonic Minor
    {0,  2,  4,  5,  7,  9,  10, 12, 14, 16, 17, 19,
     21, 22, 24, 26, 28, 29, 31, 33, 34, 36, 36, 36}, //  Mixolydian
    {0,  0,  3,  3,  5,  5,  7,  7,  10, 10, 12, 12,
     15, 15, 17, 17, 19, 19, 22, 22, 24, 24, 27, 29}, //  minor penta  doubled
    {0,  2,  3,  5,  7,  8,  10, 12, 14, 15, 17, 19,
     20, 22, 24, 26, 27, 29, 31, 32, 34, 36, 36, 36},
    {0,  2,  4,  6,  8,  10, 12, 14, 16, 18, 20, 22,
     24, 26, 28, 30, 32, 34, 36, 38, 38, 38, 38, 38}, //  whole tone
    {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
     12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23} //  Chromatic

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
const long interval = 1; // ms

// -----------------------------------------------------------------------------
// 3. UTILITY FUNCTIONS
// -----------------------------------------------------------------------------

/**
 * @brief Converts a floating-point sample to int16_t with scaling, rounding,
 * and clamping.
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

void initOLED() {

  // --- OLED Display ---
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    for (;;)
      ; // Display initialization failed, halt
  }
  display.clearDisplay();
  display.setTextSize(1); // Normal 1:1 pixel scale

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Sequencer OLED Ready"));
  display.display();
  // delay(10000);
  display.clearDisplay();
  display.display();
}
void drawSequencerOLED(const SequencerState &seqState) {
  display.clearDisplay();
  display.setTextSize(1);

  // Layout: 8 steps per row
  const uint8_t steps_per_row = 8;
  const uint8_t row_height = 16;    // pixels between rows
  const uint8_t note_y_offset = 8;  // vertical offset for first row (note page)
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
  display.drawLine(playhead_x, playhead_y, playhead_x + 15, playhead_y,
                   SSD1306_WHITE);

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

  for (int i = 0; i < N; ++i) {
    osc1.SetFreq(daisysp::mtof(note1));
    osc2.SetFreq(daisysp::mtof(note1) * 1.0032f);
    osc3.SetFreq(daisysp::mtof(note1) * .9975f);
    // osc4.SetFreq(daisysp::mtof(note1 ));

    float current_out1 = env1.Process(trigenv1);
    float current_out2 = env2.Process(trigenv2);

    float osc111 = osc1.Process();
    float osc222 = osc2.Process();
    float osc333 = osc3.Process();
    // float osc444 = osc4.Process();

    float out1 = (osc111 + osc222 + osc333) * current_out1;
    // float out2 = (osc333 + osc444) * current_out2;
    float sumL = out1 * 0.5f;
    float sumR = out1 * 0.5f;

    int16_t intSampleL = convertSampleToInt16(sumL);
    int16_t intSampleR = convertSampleToInt16(sumR);

    out[2 * i + 0] = intSampleL;
    out[2 * i + 1] = intSampleR;
  }
  buffer->sample_count = N;
}

// -----------------------------------------------------------------------------
// 6. AUDIO: OSCILLATOR & ENVELOPE INITIALIZATION
// -----------------------------------------------------------------------------

void initOscillators() {
  osc1.Init(SAMPLE_RATE);
  osc2.Init(SAMPLE_RATE);
  osc3.Init(SAMPLE_RATE);
  osc4.Init(SAMPLE_RATE);
  // osc5.Init(SAMPLE_RATE); //osc6.Init(SAMPLE_RATE); //osc7.Init(SAMPLE_RATE);
  // //osc8.Init(SAMPLE_RATE);
  env1.Init(SAMPLE_RATE);
  env2.Init(SAMPLE_RATE);

  env1.SetReleaseTime(.061f);
  env1.SetAttackTime(0.0016f);
  env2.SetAttackTime(0.001f);
  env1.SetDecayTime(0.121f);
  env2.SetDecayTime(0.121f);
  env1.SetSustainLevel(0.f);
  env2.SetSustainLevel(0.f);
  env2.SetReleaseTime(0.03f);

  // Set initial waveform for all oscillators
  osc1.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  osc2.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  osc3.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  osc4.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
  //  osc5.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
}

// -----------------------------------------------------------------------------
// MATRIX EVENT HANDLER
// -----------------------------------------------------------------------------

void matrixEventHandler(const MatrixButtonEvent &evt) {
  Serial.print("[MATRIX] Event! Index: ");
  Serial.print(evt.buttonIndex);
  Serial.print(", Type: ");
  Serial.println(evt.type == MATRIX_BUTTON_PRESSED ? "PRESSED" : "RELEASED");

  if (evt.buttonIndex < 16) { // Sequencer steps 0-15
    if (evt.type == MATRIX_BUTTON_PRESSED) {
      seq.toggleStep(evt.buttonIndex);
    }
  } else if (evt.type == MATRIX_BUTTON_PRESSED) {
    switch (evt.buttonIndex) {
    case 16: // Button 16
    {
      // Read distance from sensor
      int16_t distance = distanceRead_getDistance();
      if (distance < 0) {
        distance = 0; // Clamp negative to zero
      }
      if (distance > 800) {
        distance = 800; // Clamp max to 800 mm
      }
      // Map 0-800 mm to 0-48 note range
      int mappedNote = map(distance, 0, 800, 0, 48);

      // Set the note to the current step (use evt.buttonIndex or another step
      // index as needed) Here assuming current step is sequencer playhead
      uint8_t currentStep = seq.getPlayhead();
      seq.setStepNote(currentStep, mappedNote);
      Serial.print(F("Mapped distance "));
      Serial.print(distance);
      Serial.print(F(" mm to note "));
      Serial.println(mappedNote);
    } break;
    case 17: // Button 17
      int16_t distance = distanceRead_getDistance();
      if (distance < 0) {
        distance = 0; // Clamp negative to zero
      }
      if (distance > 800) {
        distance = 800; // Clamp max to 800 mm
      }
      // Map 0-800 mm to 0-48 note range
      int mappedVelocity = map(distance, 0, 800, 0, 127);
      uint8_t currentStep = seq.getPlayhead();
      seq.setStepVelocity(currentStep, mappedVelocity);

      // Serial.println("[MATRIX] Button 17 Pressed");
      break;
    // ... add cases for buttons 18 through 31 as needed ...
    case 18: // Button 18
      // record DetuneAmt to current Step based on variable mm from distance
      // sensor
      break;
    case 32: // Button 32
      // Handle button 32 press
      Serial.println("[MATRIX] Button 32 Pressed");
      break;
    default:
      // Handle other button presses (if any beyond 32)
      Serial.print("[MATRIX] Unhandled Button Pressed: ");
      Serial.println(evt.buttonIndex);
      break;
    }
  }
 
  drawSequencerOLED(
      seq.getState()); // Update display on any relevant matrix event
}
// -----------------------------------------------------------------------------
// 7. MIDI & CLOCK HANDLERS
// -----------------------------------------------------------------------------

void ledOn() { /* Implement as needed */ }
void ledOff() { /* Implement as needed */ }

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
  Serial.println("[uCLOCK] onClockStart() called.");
  usb_midi.sendRealTime(midi::Start); // MIDI Start message
  seq.start();
}

void onClockStop() {
  Serial.println("[uCLOCK] onClockStop() called.");
  usb_midi.sendRealTime(midi::Stop); // MIDI Stop message
  seq.stop();
}

/**
 * Monophonic step callback: handles rest, note length, and MIDI for a single
 * note. Preserves rests, glide (if implemented), note length, and MIDI
 * handling.
 */
void onStepCallback(uint32_t step) { // uClock provides the current step number
  // Ensure the step value wraps to the sequencer's number of steps (0-15)
  uint8_t wrapped_step = static_cast<uint8_t>(step % SEQUENCER_NUM_STEPS);

  //  Serial.print("[uCLOCK] onStepCallback, uClock raw step: ");
  //  Serial.print(step); Serial.print(", wrapped step for sequencer: ");
  //  Serial.println(wrapped_step);

  seq.advanceStep(wrapped_step); // Pass the wrapped step to the sequencer

  // DEBUG: Check the state of trigenv1 and note1 AFTER advanceStep
  //  Serial.print("  [SEQ_OUT] Step: "); Serial.print(wrapped_step);
  // Serial.print(", trigenv1: "); Serial.print(trigenv1 ? "ON" : "OFF");
  // Serial.print(", note1: "); Serial.println(note1);

  drawSequencerOLED(seq.getState()); // OLED is off
  // Serial.println("------------------------------------"); // Optional
  // separator for logs
}

// -----------------------------------------------------------------------------
// 8. ARDUINO SETUP FUNCTIONS
// -----------------------------------------------------------------------------

void setup() {
  // --- Synth & Envelope ---
  initOscillators();

  // Initialize distance sensor
  if (!distanceRead_init()) {
    Serial.println(F("Failed to initialize VL53L1X sensor"));
  } else {
    Serial.println(F("VL53L1X sensor initialized successfully"));
  }

  // --- I2S Output Initialization ---
  static audio_format_t my_audio_format = {.sample_freq = (uint32_t)SAMPLE_RATE,
                                           .format =
                                               AUDIO_BUFFER_FORMAT_PCM_S16,
                                           .channel_count = 2};
  static audio_buffer_format_t my_buffer_format = {
      .format = &my_audio_format,
      .sample_stride = 4 // 2 channels * 2 bytes
  };
  producer_pool = audio_new_producer_pool(&my_buffer_format, NUM_AUDIO_BUFFERS,
                                          SAMPLES_PER_BUFFER);

  audio_i2s_config_t i2s_config = {.data_pin = PICO_AUDIO_I2S_DATA_PIN,
                                   .clock_pin_base =
                                       PICO_AUDIO_I2S_CLOCK_PIN_BASE,
                                   .dma_channel = 0,
                                   .pio_sm = 0};
  audio_i2s_setup(&my_audio_format, &i2s_config);
  audio_i2s_connect(producer_pool);
  audio_i2s_set_enabled(true);

  // --- Initial Envelope Triggers ---
  trigenv1 = false; // Explicitly initialize envelope trigger flags
  trigenv2 = false;

  // --- Initial OLED Sequencer State ---
}
void setup1() {
  Serial.begin(115200);

#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Initialize TinyUSB stack. This should be done once, early, on the core
  // handling USB.
  TinyUSB_Device_Init(0);
#endif

  // Initialize Serial for debugging AFTER TinyUSB is initialized.
  // A small delay can sometimes help ensure the serial monitor is ready.
  // delay(random(333));
  //  Serial.println("Core 1: Setup1 starting. USB and Serial initialized.");
  delay(random(333));
  // Seed the random number generator
  randomSeed(
      analogRead(A0) +
      millis()); // Use an unconnected analog pin and millis for better seed
                 // Serial.println("Core 1: Random number generator seeded.");
  initOLED();
  // Initialize sequencer state BEFORE starting the clock that might use it
  seq.init();

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
  uClock.setTempo(90);
  uClock.start();
  delay(45);

  // Touch sensor
  if (!touchSensor.begin()) {
    Serial.println(
        "Core 1: ERROR - MPR121 not found. Check wiring and I2C address!");
    while (1) {
      delay(55);
    }
  } else {
    Serial.println("Core 1: MPR121 initialized successfully.");
  }

  Matrix_init(&touchSensor);
  Matrix_setEventHandler(matrixEventHandler); // Register the event handler

  pinMode(PIN_TOUCH_IRQ, INPUT);
  drawSequencerOLED(seq.getState());

  Serial.println("Core 1: Setup1 complete.");
}

// ------------------------------------------------------------------------
// 9. MAIN LOOPS
// --------------------------------------------------------------------------

void loop() {
  // --- Audio Buffer Output --
  audio_buffer_t *buf = take_audio_buffer(producer_pool, true);
  if (buf) {
    fill_audio_buffer(buf);
    give_audio_buffer(producer_pool, buf);
  }
}

void loop1() {
  usb_midi.read();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1) {
    previousMillis = currentMillis;
    Matrix_scan(); // Add this line to process touch matrix events

    // Read distance sensor and print value
    int16_t distance = distanceRead_getDistance();
    if (distance >= 0) {
      Serial.print(F("Distance: "));
      Serial.print(distance);
      Serial.println(F(" mm"));
    }
  }

} // Closes loop1()
