
// -----------------------------------------------------------------------------
// 1. INCLUDES & DEFINES
// -----------------------------------------------------------------------------
// --- Audio & DSP ---
#include <Adafruit_NeoPixel.h>

#include "src/audio/audio.h"
#include "src/audio/audio_i2s.h"
#include "src/audio/audio_pins.h"
#include "src/dsp/adsr.h"
#include "src/dsp/ladder.h"
#include "src/dsp/oscillator.h"
#include "src/dsp/phasor.h"
#include <cmath>
#include <cstdint>
//  #define DEBUG

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
volatile int raw_mm = 0;
volatile int mm = 0;
Melopero_VL53L1X sensor;

// --- I2S Pin Configuration ---
#define PICO_AUDIO_I2S_DATA_PIN 15
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16
#define IRQ_PIN 1

#define NOTE_LENGTH    12 // min: 1 max: 23 DO NOT EDIT BEYOND!!! 12 = 50% on 96ppqn, same as original \
     // tb303. 62.5% for triplets time signature

// --- Sequencer ---
Sequencer seq;

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
volatile bool  trigenv1, trigenv2, dualEnvFlag;
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
daisysp::Oscillator osc1, osc2, osc3, osc4;
daisysp::LadderFilter filter;
daisysp::Adsr env1;
daisysp::Adsr env2;

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
    osc2.SetFreq(osc_base_freq * 1.009f); // Slight detune
    osc3.SetFreq(osc_base_freq * 0.998f); // Slight detune

    // 2. Process Amplitude Envelope (env1) based on trigenv1 (from sequencer gate)
    // current_amp_env_value will be 0.0 to 1.0
    float current_amp_env_value = env1.Process(trigenv1);

    // 3. Set Filter Frequency based on freq1 (from sequencer step's filter value)
    // freq1 is expected to be in Hz (e.g., 0-5000 Hz from Lidar mapping)
    // Ensure a minimum cutoff frequency.
    float target_filter_freq =applyFilterFrequency() daisysp::fmax(20.f, freq1); 
    filter.SetFreq(90.f+target_filter_freq*current_amp_env_value);  //*current_amp_env_value);

    // 4. Generate and sum oscillator outputs
    float osc_sum = osc1.Process() + osc2.Process() + osc3.Process();

    // 5. Process summed oscillators through the filter
    float filtered_signal = filter.Process(osc_sum);

    // 6. Apply amplitude envelope and step velocity to the filtered signal
    // vel1 is 0.0 to 1.0 from sequencer step's velocity
    float final_audio_signal = filtered_signal*vel1;//* (current_amp_env_value*1.1f);// * current_amp_env_value;

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
  filter.SetFreq(3333.f);
  filter.SetRes(0.99f);
  filter.SetInputDrive(2.5f);
  filter.SetPassbandGain(0.28f);
  env1.SetReleaseTime(.1f);
  env1.SetAttackTime(0.005f);
  env2.SetAttackTime(0.001f);
  env1.SetDecayTime(0.07f);
  env2.SetDecayTime(0.121f);
  env1.SetSustainLevel(0.5f);
  env2.SetSustainLevel(0.5f);
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
        Serial.print("[MATRIX] Step ");
        Serial.print(evt.buttonIndex);
        Serial.print(" gate toggled (single tap). New gate value: ");
        Serial.println(gateState ? "ON" : "OFF");
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
#ifndef DEBUG
        Serial.println("[MATRIX] Button 17 (Velocity) held.");
#endif
        break;
      case 18: // Button 18 (Filter)
        button18Held = true;
#ifndef DEBUG
        Serial.println("[MATRIX] Button 18 (Filter) held.");
#endif
        break;
      case 19: // Record button
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
      default:
        break;
      }
    }
  
}}

// -----------------------------------------------------------------------------
// 7. MIDI & CLOCK HANDLERS
// -----------------------------------------------------------------------------

void onSync24Callback(uint32_t tick) {
  usb_midi.sendRealTime(midi::Clock);
  seq.tickNoteDuration();
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
void onOutputPPQNCallback(uint32_t tick) 
{
    seq.tickNoteDuration();
}


/**
 * Monophonic step callback: handles rest, note length, and MIDI for a single
 * note. Preserves rests, glide (if implemented), note length, and MIDI
 * handling.
 */
void onStepCallback(uint32_t step) { // uClock provides the current step number
  uint8_t wrapped_step = static_cast<uint8_t>(step % SEQUENCER_NUM_STEPS);


  seq.advanceStep(wrapped_step, mm,
                   button16Held, button17Held, button18Held,
                  selectedStepForEdit);
                  
  static int lastStepIndex = -1;
  

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

  TinyUSB_Device_Init(0);
#endif
  usb_midi.begin(MIDI_CHANNEL_OMNI);

  delay(random(333));
  randomSeed(
      analogRead(A0) +
      millis()); 

#ifndef DEBUG
Serial.print(" CORE1 SETUP1 ... ");
   delay(500);
#endif
  VL53L1_Error status = 0;
  Wire.begin();               
  sensor.initI2C(0x29, Wire); 
  status = sensor.initSensor();
  status = sensor.setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
  status = sensor.setMeasurementTimingBudgetMicroSeconds(25000);
  status = sensor.setInterMeasurementPeriodMilliSeconds(30);
  status = sensor.clearInterruptAndStartMeasurement();

  seq.init();
#ifndef DEBUG
  Serial.print(" ...Distance Sensor Initialized... ");
#endif


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

  Matrix_init(&touchSensor);
  Matrix_setEventHandler(matrixEventHandler); 

  pinMode(PIN_TOUCH_IRQ, INPUT);
#ifndef DEBUG
  Serial.println("Core 1: Setup1 complete.");
#endif
  delay(100);
      

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
void doLEDStuff() {

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
void loop1() {
  usb_midi.read();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1) {

        previousMillis = currentMillis;

    update();
doLEDStuff();
    Matrix_scan(); // Add this line to process touch matrix events
  
}
}
// Handles note duration and noteOff logic for the modular sequencer
