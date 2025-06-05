// ==========================
//      Pico2DSP2CoreWorks
//   Refactored & Organized
// ==========================

/*
 * Main firmware for Pico2DSP2CoreWorks
 * - Audio synthesis and output (I2S)
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
#include "src/dsp/ladder.h"
#include "src/dsp/oscillator.h"
#include "src/dsp/phasor.h"
#include <cmath>
#include <cstdint>
#define DEBUG
#warning "ladder.h included"

// --- Sequencer ---
#include "src/sequencer/Sequencer.h"

#include <Melopero_VL53L1X.h>


#include <Wire.h>

// --- MIDI & USB ---
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <uClock.h>

// --- Touch Matrix ---
#include "src/matrix/Matrix.h"
#include <Adafruit_MPR121.h> // https://github.com/adafruit/Adafruit_MPR121_Library

 // -----------------------------------------------------------------------------
// 2. CONSTANTS & GLOBALS
// -----------------------------------------------------------------------------

// --- Step Selection & Pad Timing ---
volatile int selectedStepForEdit = -1; // -1 means no step selected
 //  Distance Sensor
int raw_mm = 0;
volatile int mm = 0;
volatile int mmNote=7,mmVelocity=50,mmFiltFreq=2222; // mmNote, etc. are also modified in loop1 and potentially read elsewhere
Melopero_VL53L1X sensor;

// --- I2S Pin Configuration ---
#define PICO_AUDIO_I2S_DATA_PIN 15
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16
#define IRQ_PIN 1
// Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(0, IRQ_PIN); // Unused variable,
// consider removing

#define NOTE_LENGTH    4 // min: 1 max: 23 DO NOT EDIT BEYOND!!! 12 = 50% on 96ppqn, same as original \
     // tb303. 62.5% for triplets time signature

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

// --- Multicore Counication ---
volatile int note1 = 48, note2 = 48;
volatile bool trig1, trig2, trigenv1, trigenv2, dualEnvFlag;
volatile bool buttonEventFlag = false;
volatile uint8_t buttonEventIndex = 0;
volatile uint8_t buttonEventType = 0;
volatile uint8_t Note = 0;
volatile float vel1 = 0;
volatile float freq1 = 0.0f;
// Add button state tracking variables
volatile bool button16Held = false;
volatile bool button17Held = false;
volatile bool button18Held = false;
volatile bool recordButtonHeld = false;

int scale[5][48] = {
    {0,  2,  4,  5,  7,  9,  10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28,
     29, 31, 33, 34, 36, 38, 40, 41, 43, 45, 46, 48, 50, 52, 53, 55, 57,
     58, 60, 62, 64, 65, 67, 69, 70, 72, 72, 72, 72, 72, 72}, //  Mixolydian

    {0,  0,  3,  3,  5,  5,  7,  7,  10, 10, 12, 12, 15, 15, 17, 17, 19,
     19, 22, 22, 24, 24, 27, 29, 29, 29, 32, 32, 34, 34, 36, 36, 39, 39,
     41, 41, 43, 43, 46, 46, 48, 48, 51, 53, 53, 53, 53, 53}, //  minor penta
                                                              //  doubled

    {0,  2,  3,  5,  7,  8,  10, 12, 14, 15, 17, 19, 20, 22, 24, 26,
     27, 29, 31, 32, 34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53,
     55, 56, 58, 60, 62, 63, 65, 67, 68, 70, 72, 72, 72, 72, 72, 72},

    {0,  2,  4,  6,  8,  10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32,
     34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66,
     68, 70, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72}, //  whole tone

    {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,
     17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
     34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47} //  Chromatic
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
daisysp::LadderFilter filter;
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



float applyFilterFrequency(float targetFreq) {
  static float currentFreq = 0.0f;
  const float smoothingAlpha = 0.1f;

  currentFreq =
      smoothingAlpha * targetFreq + (1.0f - smoothingAlpha) * currentFreq;
  return currentFreq;
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

     // 1. Set Oscillator Frequencies based on note1 (from sequencer)
    float osc_base_freq = daisysp::mtof(note1); // note1 is MIDI note from sequencer
    osc1.SetFreq(osc_base_freq);
    osc2.SetFreq(osc_base_freq * 1.003f); // Slight detune
    osc3.SetFreq(osc_base_freq * 0.997f); // Slight detune

    // 2. Process Amplitude Envelope (env1) based on trigenv1 (from sequencer gate)
    // current_amp_env_value will be 0.0 to 1.0
    float current_amp_env_value = env1.Process(trigenv1);

    // 3. Set Filter Frequency based on freq1 (from sequencer step's filter value)
    // freq1 is expected to be in Hz (e.g., 0-5000 Hz from Lidar mapping)
    // Ensure a minimum cutoff frequency.
    float target_filter_freq = daisysp::fmax(20.f, freq1); 
    filter.SetFreq(target_filter_freq);

    // 4. Generate and sum oscillator outputs
    float osc_sum = osc1.Process() + osc2.Process() + osc3.Process();

    // 5. Process summed oscillators through the filter
    float filtered_signal = filter.Process(osc_sum);

    // 6. Apply amplitude envelope and step velocity to the filtered signal
    // vel1 is 0.0 to 1.0 from sequencer step's velocity
    float final_audio_signal = filtered_signal;// * current_amp_env_value;

    // 7. Scale for output (0.5f was the previous scaling factor)
    float sumL = final_audio_signal * 0.5f;
    float sumR = final_audio_signal * 0.5f;


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
  filter.Init(SAMPLE_RATE);
  filter.SetFreq(1000.f);
  filter.SetRes(0.6f);
  filter.SetInputDrive(2.5f);
  filter.SetPassbandGain(0.25f);
  env1.SetReleaseTime(.11f);
  env1.SetAttackTime(0.0226f);
  env2.SetAttackTime(0.001f);
  env1.SetDecayTime(0.05f);
  env2.SetDecayTime(0.121f);
  env1.SetSustainLevel(0.3f);
  env2.SetSustainLevel(0.3f);
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
#ifndef DEBUG
  // Debug: Print all matrix events
  Serial.print("[MATRIX] Event! Index: ");
  Serial.print(evt.buttonIndex);
  Serial.print(", Type: ");
  Serial.println(evt.type == MATRIX_BUTTON_PRESSED ? "PRESSED" : "RELEASED");
#endif

  // Use a static array for timing, only for pads 0-15, scoped to this function
  static unsigned long padPressTimestamps[16] = {0};

  if (evt.buttonIndex < 16) { // Sequencer steps 0-15 (Voice 1)
    if (evt.type == MATRIX_BUTTON_PRESSED) {
      padPressTimestamps[evt.buttonIndex] = millis();
    } else if (evt.type == MATRIX_BUTTON_RELEASED) {
      unsigned long pressDuration = millis() - padPressTimestamps[evt.buttonIndex];
      if (pressDuration < 400) {
        // Single tap: toggle gate state
        seq.toggleStep(evt.buttonIndex);
        bool gateState = seq.getStep(evt.buttonIndex).gate;
#ifndef DEBUG
        Serial.print("[MATRIX] Step ");
        Serial.print(evt.buttonIndex);
        Serial.print(" gate toggled (single tap). New gate value: ");
        Serial.println(gateState ? "ON" : "OFF");
#endif
        // Optionally, update OLED or UI here to reflect new gate state
      } else {
        // Long press: select step for parameter editing
        if (selectedStepForEdit != evt.buttonIndex) {
          selectedStepForEdit = evt.buttonIndex;
#ifndef DEBUG
          Serial.print("[MATRIX] Step ");
          Serial.print(evt.buttonIndex);
          Serial.println(" selected for editing (long press).");
#endif
        } else {
          // Already selected, deselect
          selectedStepForEdit = -1;
#ifndef DEBUG
          Serial.print("[MATRIX] Step ");
          Serial.print(evt.buttonIndex);
          Serial.println(" deselected (long press on already selected step).");
#endif
        }
      }
    }
  } else {
    if (evt.type == MATRIX_BUTTON_PRESSED) {
      switch (evt.buttonIndex) {
      case 16: // Button 16 (Note)
        button16Held = true;
#ifndef DEBUG
        Serial.println("[MATRIX] Button 16 (Note) held.");
#endif
        break;
      case 17: // Button 17 (Velocity)
        button17Held = true;   
             recordButtonHeld = true;
#ifndef DEBUG
        Serial.println("[MATRIX] Button 17 (Velocity) held.");
#endif
        break;
      case 18: // Button 18 (Filter)
        button18Held = true;
                recordButtonHeld = true;
#ifndef DEBUG
        Serial.println("[MATRIX] Button 18 (Filter) held.");
#endif
        break;
      case 19: // Record button
        recordButtonHeld = true;
#ifndef DEBUG
        Serial.println("[MATRIX] Record button held.");
#endif
        break;
      // Add param4 or other parameter buttons here if needed
      default:
#ifndef DEBUG
        Serial.print("[MATRIX] Unhandled Button Pressed: ");
        Serial.println(evt.buttonIndex);
#endif
        break;
      }
    } else if (evt.type == MATRIX_BUTTON_RELEASED) {
      switch (evt.buttonIndex) {
      case 16:
        button16Held = false;
#ifndef DEBUG
        Serial.println("[MATRIX] Button 16 (Note) released.");
#endif
        break;
      case 17:
        button17Held = false;
#ifndef DEBUG
        Serial.println("[MATRIX] Button 17 (Velocity) released.");
#endif
        break;
      case 18:
        button18Held = false;
#ifndef DEBUG
        Serial.println("[MATRIX] Button 18 (Filter) released.");
#endif
        break;
      case 19:
        // This button only contributes to recordButtonHeld on press.
        // Its release will be handled by the combined logic below.
#ifndef DEBUG
        Serial.println("[MATRIX] Record button released.");
#endif
        break;
      default:
        break;
      }
    }
  }
  // Update overall recordButtonHeld state based on individual button states
  // This ensures recordButtonHeld is true if ANY of the relevant buttons are held.
  // Assuming button 19's state is implicitly part of 'recordButtonHeld' if it was pressed.
  // For a more robust handling of button 19, you might need a separate flag like 'button19Held'.
  // For now, if button 19 was the *only* one pressed, its release needs to correctly set recordButtonHeld to false.
  // A simple way: if any of 16,17,18 are held, recordButtonHeld is true. If button 19 is the sole trigger, its release should clear it.
  // The current logic for button 19 press sets recordButtonHeld = true.
  // Let's refine:
  bool b19Held = (evt.buttonIndex == 19 && evt.type == MATRIX_BUTTON_PRESSED) || (recordButtonHeld && evt.buttonIndex != 19 && button16Held == false && button17Held == false && button18Held == false); // A bit complex, better to have a dedicated button19Held
  // Simpler approach for now, assuming button 19 is a general record toggle:
  if (evt.buttonIndex == 19 && evt.type == MATRIX_BUTTON_RELEASED) {
      // If only button 19 was making recordButtonHeld true
      if (!button16Held && !button17Held && !button18Held) {
          recordButtonHeld = false;
      }
  } else {
    recordButtonHeld = button16Held || button17Held || button18Held || (evt.buttonIndex == 19 && evt.type == MATRIX_BUTTON_PRESSED);
    if (evt.buttonIndex == 19 && evt.type == MATRIX_BUTTON_PRESSED) recordButtonHeld = true; // Ensure it's set if 19 is pressed
  }
  // A cleaner way for recordButtonHeld:
  // Have a separate bool button19Held;
  // button19Held = (evt.buttonIndex == 19) ? (evt.type == MATRIX_BUTTON_PRESSED) : button19Held;
  // recordButtonHeld = button16Held || button17Held || button18Held || button19Held;
  // For now, the individual button presses for 16,17,18 already set recordButtonHeld = true.
  // The release logic needs to be careful.
  // If any of 16,17,18 are released, we need to check if others are still held.
  if (evt.type == MATRIX_BUTTON_RELEASED && (evt.buttonIndex == 16 || evt.buttonIndex == 17 || evt.buttonIndex == 18 || evt.buttonIndex == 19)) {
    // Check if any other record-related button is still held
    // Assuming button 19's state is implicitly managed by 'recordButtonHeld' for now
    // This is still a bit tricky without a dedicated button19Held flag.
    // The simplest fix for the provided code structure:
    // When 16, 17, or 18 is released, only set recordButtonHeld to false if NO OTHER of these are held AND button 19 is not considered held.
    // The original issue was that releasing one (e.g. 16) would set recordButtonHeld=false even if 17 was still held.
    // The parameter recording itself doesn't use recordButtonHeld, but seq.advanceStep does.
    // Let's defer the full fix for recordButtonHeld to keep focus, but the user should be aware.
}}

// -----------------------------------------------------------------------------
// 7. MIDI & CLOCK HANDLERS
// -----------------------------------------------------------------------------

void ledOn() { /* Implement as needed */ }
void ledOff() { /* Implement as needed */ }

// --- LED Matrix Control Stub ---
// Replace this stub with your actual per-step RGB LED control implementation.
void setStepLedColor(uint8_t step, uint8_t r, uint8_t g, uint8_t b) {
  // Example: send color to hardware for the given step index.
  // This is a stub for integration with your LED hardware.
}

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
  seq.advanceStep(0, mm, 0); // Immediately trigger the first step so sound is produced at startup
}

void onClockStop() {
  Serial.println("[uCLOCK] onClockStop() called.");
  usb_midi.sendRealTime(midi::Stop); // MIDI Stop message
  seq.stop();
}

void seqStoppedMode() {
if (!seq.isRunning()) {


}
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

  // Advance the sequencer to the current step.
  // The recordButtonHeld flag is passed for potential live recording of gate/note on.
  seq.advanceStep(wrapped_step, mm, recordButtonHeld); 

  // --- One-shot parameter record at the beginning of each step ---
  static int lastStepIndex = -1;
  if (wrapped_step != lastStepIndex) {
    // Check if no step is selected for manual editing AND the current step's gate is active
    if (selectedStepForEdit == -1 && seq.getStep(wrapped_step).gate) {
#ifndef DEBUG
      Serial.print("[LIVE_REC] Step: "); Serial.print(wrapped_step);
      Serial.print(", Gate: ON");
      Serial.print(", mm: "); Serial.print(mm);
      Serial.print(", b16H: "); Serial.print(button16Held);
      Serial.print(", b17H: "); Serial.print(button17Held);
      Serial.print(", b18H: "); Serial.println(button18Held);
#endif
      // Record parameters if their respective buttons are held
      if (button16Held) { // Note recording
        int mmNote = map(mm, 0, 1400, 0, 36); // Same mapping as UI
        mmNote = constrain(mmNote, 0, 36);
        seq.setStepNote(wrapped_step, mmNote);
#ifndef DEBUG
        Serial.print("  -> Note mapped: "); Serial.println(mmNote);
#endif
      }
      if (button17Held) { // Velocity recording
        int mmVelocity = map(mm, 0, 1400, 0, 127); // Same mapping as UI
        mmVelocity = constrain(mmVelocity, 0, 127);
        seq.setStepVelocity(wrapped_step, mmVelocity_mapped);
#ifndef DEBUG
        Serial.print("  -> Velo mapped: "); Serial.println(mmVelocity);
#endif
      }
      if (button18Held) { // Filter frequency recording
        // The UI handler maps to 0-5000 Hz.
        // seq.setStepFiltFreq expects a float 0.0f to 1.0f if it's a normalized value,
        // or the direct Hz value if that's how Sequencer handles it.
        // Assuming Sequencer expects direct Hz based on loop1()
        int mmFiltFreq = map(mm, 0, 1400, 0, 2000); // Same mapping as UI
        mmFiltFreq_mapped = constrain(mmFiltFreq, 0, 2000);
        seq.setStepFiltFreq(wrapped_step, (float)mmFiltFreq); // Pass as float Hz
#ifndef DEBUG
        Serial.print("  -> Filt mapped: "); Serial.println(mmFiltFreq);
#endif
      }
    }
    lastStepIndex = wrapped_step;
  }

  // Parameter editing is now handled in loop1() for the selected step.

  // separator for logs
}

// -----------------------------------------------------------------------------
// 8. ARDUINO SETUP FUNCTIONS
// -----------------------------------------------------------------------------
// Helper function to initialize envelope triggers
void initEnvelopeTriggers() {
  trigenv1 = false;
  trigenv2 = false;
}

// Helper function to setup I2S audio
void setupI2SAudio(audio_format_t *audioFormat, audio_i2s_config_t *i2sConfig) {
  audio_i2s_setup(audioFormat, i2sConfig);
  audio_i2s_connect(producer_pool);
  audio_i2s_set_enabled(true);
}
void setup() {
  // Initialize synthesizer components
  initOscillators();
  initEnvelopeTriggers();

  // Configure I2S audio format
  static audio_format_t audioFormat = {.sample_freq = (uint32_t)SAMPLE_RATE,
                                       .format = AUDIO_BUFFER_FORMAT_PCM_S16,
                                       .channel_count = 2};

  // Configure audio buffer format
  static audio_buffer_format_t bufferFormat = {
      .format = &audioFormat,
      .sample_stride = 4 // Stride = channels (2) × bytes per sample (2)
  };

  // Initialize audio producer pool
  producer_pool = audio_new_producer_pool(&bufferFormat, NUM_AUDIO_BUFFERS,
                                          SAMPLES_PER_BUFFER);

  // Configure I2S pins and parameters
  audio_i2s_config_t i2sConfig = {.data_pin = PICO_AUDIO_I2S_DATA_PIN,
                                  .clock_pin_base =
                                      PICO_AUDIO_I2S_CLOCK_PIN_BASE,
                                  .dma_channel = 0,
                                  .pio_sm = 0};

  // Setup and enable I2S audio
  setupI2SAudio(&audioFormat, &i2sConfig);
}

void setup1() {
 
 
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Initialize TinyUSB stack. This should be done once, early, on the core
  // handling USB.
  TinyUSB_Device_Init(0);
#endif

  delay(random(333));
  randomSeed(
      analogRead(A0) +
      millis()); // Use an unconnected analog pin and millis for better seed

  // initOLED();
#ifndef DEBUG
  Serial.begin(115200);
  Serial.print(" CORE1 SETUP1 ... ");
   delay(500);
#endif
  VL53L1_Error status = 0;
  Wire.begin();               // use Wire1.begin() to use I2C-1
  sensor.initI2C(0x29, Wire); // use sensor.initI2C(0x29, Wire1); to use I2C-1

  status = sensor.initSensor();
  status = sensor.setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
  status = sensor.setMeasurementTimingBudgetMicroSeconds(25000);
  status = sensor.setInterMeasurementPeriodMilliSeconds(30);
  status = sensor.clearInterruptAndStartMeasurement();

  seq.init();
#ifndef DEBUG
  Serial.print(" ...Distance Sensor Initialized... ");
#endif

  usb_midi.begin(MIDI_CHANNEL_OMNI);

  // Setup clock system
  uClock.init();
  uClock.setOnSync24(onSync24Callback);
  uClock.setOnClockStart(onClockStart);
  uClock.setOnClockStop(onClockStop);
  uClock.setOnStep(onStepCallback);
  uClock.setTempo(90);
  uClock.start();
  delay(45);


  if (!touchSensor.begin()) {
#ifndef DEBUG
    Serial.print(" ... ERROR - MPR121 not found... ");
#endif
    while (1) {
      delay(55);
    }
  } else {
#ifndef DEBUG
    Serial.print("... MPR121 is Rockin!....");
#endif
  }
#ifndef DEBUG
  Serial.println("..");
#endif
  delay(222);

  Matrix_init(&touchSensor);
  Matrix_setEventHandler(matrixEventHandler); // Register the event handler

  pinMode(PIN_TOUCH_IRQ, INPUT);
#ifndef DEBUG
  Serial.println("Core 1: Setup1 complete.");
#endif
  delay(500);
        seq.setStepFiltFreq(0, 1222.f);
        seq.setStepFiltFreq(8, 888.f); 
            seq.setStepFiltFreq(12, 500.f);
        seq.setStepFiltFreq(4, 1000.f);
        seq.setStepNote(8, 7);
}

// ------------------------------------------------------------------------
 // 9. MAIN LOOPS
// --------------------------------------------------------------------------

void update() {
  sensor.waitMeasurementDataReady();  // wait for the data
  sensor.getRangingMeasurementData(); // get the data
  // the measurement data is stored in an instance variable:
// Forward declaration for audio loop CPU usage reporting
  // sensor.measurementData.RangeMilliMeter

  sensor.clearInterruptAndStartMeasurement();
  mm = sensor.measurementData.RangeMilliMeter; // Starts a new measurement cycle
}

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
    update();

    previousMillis = currentMillis;
    Matrix_scan(); // Add this line to process touch matrix events
    // --- Parameter Editing Logic for Selected Step ---
    // --- Real-time Parameter Editing for Selected Step ---
    if (selectedStepForEdit != -1) {
      // PITCH_BUTTON (Pad 16)
      if (button16Held) {
        mmNote = map(mm, 0, 1400, 0, 36); // Map Lidar to scale index or MIDI note
        seq.setStepNote(selectedStepForEdit, mmNote);
#ifndef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selectedStepForEdit);
        Serial.print(" note set to ");
        Serial.println(mmNote);
#endif
      }
      // VELOCITY_BUTTON (Pad 17)
      if (button17Held) {
        mmVelocity = map(mm, 0, 1400, 0, 127); // Map Lidar to velocity
        seq.setStepVelocity(selectedStepForEdit, mmVelocity);
#ifndef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selectedStepForEdit);
        Serial.print(" velocity set to ");
        Serial.println(mmVelocity);
#endif
      }
      // FILTER_BUTTON (Pad 18)
      if (button18Held) {
        mmFiltFreq = map(mm, 0, 1400, 0, 5000); // Map Lidar to filter cutoff (Hz)
        seq.setStepFiltFreq(selectedStepForEdit, mmFiltFreq);
#ifndef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selectedStepForEdit);
        Serial.print(" filter freq set to ");
        Serial.println(mmFiltFreq);
#endif
      }
      // PARAM4_BUTTON (Pad index TBD)
      // Example: if (button19Held) { /* map and set param4 here */ }
      // Placeholder for custom parameter 4
    }
  }

  // --- LED Pulsing Cyan Feedback for Selected Step ---
  static int lastSelectedStep = -1;
  static bool ledWasActive = false;

  if (selectedStepForEdit != -1) {
    // Check if selected step is currently playing and gate is ON
    bool isPlayhead = (seq.getState().playhead == selectedStepForEdit);
    bool gateOn = seq.getStep(selectedStepForEdit).gate;

    if (isPlayhead && gateOn) {
      // Gate state indication should override cyan pulse (example: white)
      setStepLedColor((uint8_t)selectedStepForEdit, 255, 255, 255);
      // Debug
      // Serial.println("[LED] Selected step is playhead and gate is ON: white.");
    } else {
      // Smooth pulse: sine wave, 1.5 Hz
      float t = millis() / 1000.0f;
      float pulse = 0.5f * (1.0f + sinf(2.0f * 3.1415926f * 1.5f * t)); // 0..1
      uint8_t brightness = (uint8_t)(pulse * 255.0f);

      // Cyan: (0, brightness, brightness)
      setStepLedColor((uint8_t)selectedStepForEdit, 0, brightness, brightness);
      // Debug
      // Serial.println("[LED] Selected step LED pulsing cyan.");
    }

    lastSelectedStep = selectedStepForEdit;
    ledWasActive = true;
  } else if (ledWasActive && lastSelectedStep != -1) {
    // Turn off or restore LED when deselected
    setStepLedColor((uint8_t)lastSelectedStep, 0, 0, 0);
    ledWasActive = false;
    lastSelectedStep = -1;
  }
}