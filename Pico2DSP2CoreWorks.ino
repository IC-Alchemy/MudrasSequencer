

// To enable debug prints, uncomment the line below:
// #define DEBUG
// (This ensures DEBUG is off by default unless explicitly uncommented).


// --- Audio & DSP ---
#include "src/LEDMatrix/LEDMatrixFeedback.h"
#include "src/LEDMatrix/ledMatrix.h"
#include "src/audio/audio.h"
#include "src/audio/audio_i2s.h"
#include "src/audio/audio_pins.h"
#include "src/dsp/adsr.h"
#include "src/dsp/ladder.h"
#include "src/dsp/oscillator.h"
#include "src/dsp/phasor.h"
#include <cmath>
#include <cstdint>

// --- Sequencer ---
#include "src/sequencer/Sequencer.h"
#include "src/SynthState.h"

// --- Sensors & LEDs ---
#include <Melopero_VL53L1X.h>
#include <FastLED.h>

// --- Communication ---
#include <SPI.h>
#include <Wire.h>

// --- MIDI & USB ---
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <uClock.h>

// --- Touch Matrix ---
#include "src/matrix/Matrix.h"
#include <Adafruit_MPR121.h> // https://github.com/adafruit/Adafruit_MPR121_Library

// Forward declarations for static helper functions
static void handleParameterEditing(bool btn16H, bool btn17H, bool btn18H, int mm_val, int selStep, Sequencer& s);
static void initCore1SerialAndPlatform();
static void initDistanceSensor(Melopero_VL53L1X& sensor_obj, TwoWire& wire_obj);
static void initTouchMatrix(Adafruit_MPR121& touch_sensor_obj);
static void initMIDI(decltype(usb_midi)& midi_obj);
static void initClockAndSequencer(decltype(uClock)& clock_obj, Sequencer& sequencer_obj);
static void initLEDMatrix(decltype(ledMatrix)& matrix_obj);


// -----------------------------------------------------------------------------
// 2. CONSTANTS & GLOBAL VARIABLES
// -----------------------------------------------------------------------------

// --- LED Matrix ---
LEDMatrix ledMatrix; ///< Instance of the LED matrix controller.

// --- Step Selection & Pad Timing ---
int selectedStepForEdit = -1; ///< Index of the sequencer step currently selected for editing (-1 if none).

// --- Distance Sensor ---
int raw_mm = 0;             ///< Raw distance reading from VL53L1X sensor.
int mm = 0;                 ///< Processed distance reading from VL53L1X sensor.
int mmNote = 7;             ///< Note value derived from distance sensor, used for live input/recording.
int mmVelocity = 50;        ///< Velocity value derived from distance sensor (0-127 for MIDI mapping, then normalized for Step).
int mmFiltFreq = 2222;      ///< Filter frequency value derived from distance sensor (Hz for direct use or mapping to 0-1 for Step).
Melopero_VL53L1X sensor;    ///< Instance of the VL53L1X distance sensor driver.

// --- I2S Pin Configuration ---
#define PICO_AUDIO_I2S_DATA_PIN 15      ///< Data pin for I2S audio output.
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16 ///< Base clock pin for I2S audio output.
#define IRQ_PIN 1                       ///< General IRQ pin (purpose may vary, e.g. for sensor).

// --- MIDI & Clock ---
Adafruit_USBD_MIDI raw_usb_midi;        ///< Raw USB MIDI device instance.
midi::SerialMIDI<Adafruit_USBD_MIDI> serial_usb_midi(raw_usb_midi); ///< MIDI library wrapper for USB MIDI.
midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>> usb_midi(serial_usb_midi); ///< MIDI interface object.
uint8_t bpm_blink_timer = 1;            ///< Timer/counter for BPM LED blinking logic.

// --- Sequencer ---
Sequencer seq(usb_midi, scale); ///< Sequencer instance, initialized with MIDI interface and scale data.

// --- Touch Matrix ---
const int PIN_TOUCH_IRQ = 6;            ///< IRQ pin for the MPR121 touch sensor.
Adafruit_MPR121 touchSensor = Adafruit_MPR121(); ///< Instance of the MPR121 touch sensor driver.

// --- Synth State ---
volatile SynthVoiceState g_synthVoiceState = {48, 0.0f, 200.0f, false}; ///< Global state for the primary synth voice.

// --- Multicore Communication & Other Flags ---
volatile int note2 = 48;                ///< MIDI note for a potential second voice or other purpose.
volatile bool trig1, trig2;             ///< General purpose trigger flags (usage TBD or specific to unrefactored parts).
volatile bool dualEnvFlag;              ///< Flag for dual envelope mode (usage TBD or specific to unrefactored parts).
volatile bool buttonEventFlag = false;  ///< Flag indicating a button event has occurred.
volatile uint8_t buttonEventIndex = 0;  ///< Index of the button that triggered an event.
volatile uint8_t buttonEventType = 0;
volatile uint8_t Note = 0;
// vel1 and freq1 removed.

// --- Button State Tracking ---
bool button16Held = false;
bool button17Held = false;
bool button18Held = false;
bool recordButtonHeld = false;

// --- Musical Scales ---
int scale[5][48] = {
    // Mixolydian
    {0, 2, 4, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28,
     29, 31, 33, 34, 36, 38, 40, 41, 43, 45, 46, 48, 50, 52, 53, 55, 57,
     58, 60, 62, 64, 65, 67, 69, 70, 72, 72, 72, 72, 72, 72},
    // Minor Pentatonic (doubled)
    {0, 0, 3, 3, 5, 5, 7, 7, 10, 10, 12, 12, 15, 15, 17, 17, 19,
     19, 22, 22, 24, 24, 27, 29, 29, 29, 32, 32, 34, 34, 36, 36, 39, 39,
     41, 41, 43, 43, 46, 46, 48, 48, 51, 53, 53, 53, 53, 53},
    // Custom Scale
    {0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26,
     27, 29, 31, 32, 34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53,
     55, 56, 58, 60, 62, 63, 65, 67, 68, 70, 72, 72, 72, 72, 72, 72},
    // Whole Tone
    {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32,
     34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66,
     68, 70, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72},
    // Chromatic
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
     17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
     34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47}
};

// --- Audio & Synth ---
constexpr float SAMPLE_RATE = 44100.0f;         ///< Audio sample rate in Hz.
constexpr float OSC_SUM_SCALING = 0.1f;         ///< Scaling factor for summed oscillator outputs (consider if still actively used or if mixing is handled differently).
constexpr float INT16_MAX_AS_FLOAT = 32767.0f;  ///< Maximum value of a 16-bit signed integer as a float.
constexpr float INT16_MIN_AS_FLOAT = -32768.0f; ///< Minimum value of a 16-bit signed integer as a float.
constexpr int NUM_AUDIO_BUFFERS = 3;            ///< Number of audio buffers in the pool.
constexpr int SAMPLES_PER_BUFFER = 256;         ///< Number of samples per audio buffer.
float baseFreq = 110.0f;                        ///< Base frequency, possibly for tuning or reference (Hz).
constexpr float OSC_DETUNE_FACTOR = 1.01f;      ///< Detune factor for oscillators.

// --- Oscillators & Envelopes & Filter ---
daisysp::Oscillator osc1, osc2, osc3, osc4, osc5, osc6, osc7, osc8; ///< Oscillator objects.
daisysp::Adsr env1, env2;                       ///< ADSR envelope objects (env1 for primary voice, env2 for potential second).
daisysp::LadderFilter filter;                   ///< Ladder filter object for the primary voice.

// --- Audio Buffer Pool ---
audio_buffer_pool_t *producer_pool = nullptr;   ///< Pointer to the audio buffer pool used by the producer (core 0).

// --- Timing ---
unsigned long previousMillis1 = 0; ///< Timestamp for general timing purposes in loop1.
unsigned long previousMillis2 = 0; ///< Timestamp for LED update timing in loop1.
const long interval = 1;           ///< General interval in milliseconds (currently 1ms, for loop1 main tasks).

// -----------------------------------------------------------------------------
// 3. UTILITY FUNCTIONS
// -----------------------------------------------------------------------------

/**
 * @brief Converts a floating-point audio sample to int16_t.
 * Includes scaling to full 16-bit range, rounding, and clamping.
 * @param sample The input floating-point sample.
 * @return The converted int16_t sample.
 */
static inline int16_t convertSampleToInt16(float sample) {
    float scaled = sample * INT16_MAX_AS_FLOAT; // Scale to 16-bit range
    scaled = roundf(scaled);                    // Round to nearest integer
    scaled = daisysp::fclamp(scaled, INT16_MIN_AS_FLOAT, INT16_MAX_AS_FLOAT); // Clamp to ensure valid range
    return static_cast<int16_t>(scaled);
}

/**
 * @brief Smooths filter frequency changes using a simple low-pass filter.
 * This helps prevent abrupt, clicky changes in filter cutoff.
 * @param targetFreq The desired target filter frequency.
 * @return The smoothed filter frequency to be applied.
 */
float applyFilterFrequency(float targetFreq) {
    static float currentFreq = 0.0f; // Static variable to hold the current smoothed frequency
    const float smoothingAlpha = 0.1f; // Smoothing factor (adjust for faster/slower smoothing)
    currentFreq = smoothingAlpha * targetFreq + (1.0f - smoothingAlpha) * currentFreq;
    return currentFreq;
}

// -----------------------------------------------------------------------------
// 4. AUDIO: SYNTHESIS & BUFFER FILLING
// -----------------------------------------------------------------------------

/**
 * @brief Fills an audio buffer with synthesized stereo samples.
 */
void fill_audio_buffer(audio_buffer_t *buffer) {
    int N = buffer->max_sample_count;
    int16_t *out = reinterpret_cast<int16_t *>(buffer->buffer->bytes);

    for (int i = 0; i < N; ++i) {
        // 1. Set Oscillator Frequencies based on g_synthVoiceState.note
        float osc_base_freq = daisysp::mtof(g_synthVoiceState.note);
        osc1.SetFreq(osc_base_freq);
        osc2.SetFreq(osc_base_freq * 1.003f); // Slight detune
        osc3.SetFreq(osc_base_freq * 0.997f); // Slight detune

        // 2. Process Amplitude Envelope (env1) based on g_synthVoiceState.gateOn
        float current_amp_env_value = env1.Process(g_synthVoiceState.gateOn);

        // 3. Set Filter Frequency based on g_synthVoiceState.filterCutoff
        float target_filter_freq = daisysp::fmax(20.f, g_synthVoiceState.filterCutoff);
        filter.SetFreq(target_filter_freq);

        // 4. Generate and sum oscillator outputs
        float osc_sum = osc1.Process() + osc2.Process() + osc3.Process();

        // 5. Process summed oscillators through the filter
        float filtered_signal = filter.Process(osc_sum);

        // 6. Apply amplitude envelope and step velocity to the filtered signal
        float final_audio_signal = filtered_signal * current_amp_env_value * g_synthVoiceState.velocity;

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
// 5. AUDIO: OSCILLATOR & ENVELOPE INITIALIZATION
// -----------------------------------------------------------------------------

/**
 * @brief Initializes oscillators, envelopes, and filter with default parameters.
 */
void initOscillators() {
    osc1.Init(SAMPLE_RATE);
    osc2.Init(SAMPLE_RATE);
    osc3.Init(SAMPLE_RATE);
    osc4.Init(SAMPLE_RATE);
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
}

// -----------------------------------------------------------------------------
// 6. MATRIX EVENT HANDLER
// -----------------------------------------------------------------------------

/**
 * @brief Handles button events from the touch matrix.
 */
void matrixEventHandler(const MatrixButtonEvent &evt) {
#ifdef DEBUG
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
#ifdef DEBUG
                Serial.print("[MATRIX] Step ");
                Serial.print(evt.buttonIndex);
                Serial.print(" gate toggled (single tap). New gate value: ");
                Serial.println(gateState ? "ON" : "OFF");
#endif
            } else {
                // Long press: select step for parameter editing
                if (selectedStepForEdit != evt.buttonIndex) {
                    selectedStepForEdit = evt.buttonIndex;
#ifdef DEBUG
                    Serial.print("[MATRIX] Step ");
                    Serial.print(evt.buttonIndex);
                    Serial.println(" selected for editing (long press).");
#endif
                } else {
                    // Already selected, deselect
                    selectedStepForEdit = -1;
#ifdef DEBUG
                    Serial.print("[MATRIX] Step ");
                    Serial.print(evt.buttonIndex);
                    Serial.println(" deselected ");
#endif
                }
            }
        }
    } else {
        // Handle parameter and record buttons (indices 16+)
        bool *buttonHeld = nullptr;
        const char *buttonName = nullptr;
        switch (evt.buttonIndex) {
            case 16: buttonHeld = &button16Held; buttonName = "Button 16 (Note)"; break;
            case 17: buttonHeld = &button17Held; buttonName = "Button 17 (Velocity)"; break;
            case 18: buttonHeld = &button18Held; buttonName = "Button 18 (Filter)"; break;
            case 19: buttonHeld = &recordButtonHeld; buttonName = "Record button"; break;
            default: break;
        }
        if (buttonHeld) {
            if (evt.type == MATRIX_BUTTON_PRESSED) {
                *buttonHeld = true;
#ifdef DEBUG
                Serial.print("[MATRIX] ");
                Serial.print(buttonName);
                Serial.println(" held.");
#endif
            } else if (evt.type == MATRIX_BUTTON_RELEASED) {
                *buttonHeld = false;
#ifdef DEBUG
                Serial.print("[MATRIX] ");
                Serial.print(buttonName);
                Serial.println(" released.");
#endif
            }
        } else if (evt.type == MATRIX_BUTTON_PRESSED) {
#ifdef DEBUG
            Serial.print("[MATRIX] Unhandled Button Pressed: ");
            Serial.println(evt.buttonIndex);
#endif
        }
    }
}

// -----------------------------------------------------------------------------
// 7. MIDI & CLOCK HANDLERS
// -----------------------------------------------------------------------------

/**
 * @brief Sends MIDI clock tick and updates BPM LED.
 */
void onSync24Callback(uint32_t tick) {
    usb_midi.sendRealTime(midi::Clock);
}

/**
 * @brief Handles clock start event.
 */
void onClockStart() {
    usb_midi.sendRealTime(midi::Start); // MIDI Start message
    seq.start();
    seq.advanceStep(0); // Immediately trigger the first step so sound is produced at startup
}

/**
 * @brief Handles clock stop event.
 */
void onClockStop() {
#ifdef DEBUG
    Serial.println("[uCLOCK] onClockStop() called.");
#endif
    usb_midi.sendRealTime(midi::Stop); // MIDI Stop message
    seq.stop();
}

/**
 * @brief Called when sequencer is stopped.
 */
void seqStoppedMode() {
    if (!seq.isRunning()) {
        // Placeholder for stopped mode logic
    }
}

/**
 * @brief Step callback for monophonic sequencer.
 * Handles rest, note length, and MIDI for a single note.
 */
void onStepCallback(uint32_t step) {
    // Ensure the step value wraps to the sequencer's number of steps (0-15)
    uint8_t wrapped_step = static_cast<uint8_t>(step % SEQUENCER_NUM_STEPS);

    seq.advanceStep(wrapped_step);

    // g_synthVoiceState is now updated directly by Sequencer::advanceStep.
    // The bridging code below is no longer needed.
    // g_synthVoiceState.note = note1;
    // g_synthVoiceState.velocity = vel1;
    // g_synthVoiceState.filterCutoff = freq1;
    // g_synthVoiceState.gateOn = trigenv1;

    // Live record: if record button is held, overwrite/update the current step
    if (seq.isRunning() && recordButtonHeld) {
        seq.setStepNote(wrapped_step, mmNote);
        seq.setStepVelocity(wrapped_step, mmVelocity);
        seq.setStepFiltFreq(wrapped_step, mmFiltFreq);
    }

    // Parameter editing is now handled in loop1() for the selected step.
    // No parameter editing here to avoid conflicts.

}

// -----------------------------------------------------------------------------
// 8. ARDUINO SETUP FUNCTIONS
// -----------------------------------------------------------------------------

/**
 * @brief Initializes global synth state and related legacy flags.
 */
void initSynthStateGlobal() {
    g_synthVoiceState.gateOn = false;
    // trigenv1 and trigenv2 are no longer used/declared globally for this voice.
}

/**
 * @brief Configures and enables I2S audio.
 */
void setupI2SAudio(audio_format_t *audioFormat, audio_i2s_config_t *i2sConfig) {
    audio_i2s_setup(audioFormat, i2sConfig);
    audio_i2s_connect(producer_pool);
    audio_i2s_set_enabled(true);
}

/**
 * @brief Arduino setup for core 0.
 */
void setup() {
    initOscillators();
    initSynthStateGlobal();

    // Configure I2S audio format
    static audio_format_t audioFormat = {
        .sample_freq = (uint32_t)SAMPLE_RATE,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2
    };

    // Configure audio buffer format
    static audio_buffer_format_t bufferFormat = {
        .format = &audioFormat,
        .sample_stride = 4 // Stride = channels (2) × bytes per sample (2)
    };

    // Initialize audio producer pool
    producer_pool = audio_new_producer_pool(&bufferFormat, NUM_AUDIO_BUFFERS, SAMPLES_PER_BUFFER);

    // Configure I2S pins and parameters
    audio_i2s_config_t i2sConfig = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0
    };

    // Setup and enable I2S audio
    setupI2SAudio(&audioFormat, &i2sConfig);
}

/**
 * @brief Arduino setup for core 1.
 */
void setup1() {
    initCore1SerialAndPlatform();
    initDistanceSensor(sensor, Wire);
    initTouchMatrix(touchSensor); // MPR121 check is inside this function now
    initMIDI(usb_midi);
    pinMode(PIN_TOUCH_IRQ, INPUT); // Set touch IRQ pin mode after touch sensor init
    initClockAndSequencer(uClock, seq); // seq.init() is called inside this
    initLEDMatrix(ledMatrix);

#ifdef DEBUG
    // Combined messages from individual init functions provide detailed status.
    Serial.println("Core 1: All initializations complete.");
    delay(500);
#endif
}

/**
 * @brief Updates distance sensor measurement.
 */

void update() {
    sensor.waitMeasurementDataReady();
    sensor.getRangingMeasurementData();
    sensor.clearInterruptAndStartMeasurement();
    mm = sensor.measurementData.RangeMilliMeter;
}

// -----------------------------------------------------------------------------
// STATIC HELPER FUNCTION IMPLEMENTATIONS
// -----------------------------------------------------------------------------

/**
 * @brief Handles real-time parameter editing for a selected sequencer step using Lidar input.
 * Modifies note, velocity, or filter of the selected step based on which parameter button is held.
 * @param btn16H True if the 'note edit' button is held.
 * @param btn17H True if the 'velocity edit' button is held.
 * @param btn18H True if the 'filter edit' button is held.
 * @param mm_val Current Lidar distance reading.
 * @param selStep Index of the currently selected step for editing.
 * @param s Reference to the Sequencer object.
 */
static void handleParameterEditing(bool btn16H, bool btn17H, bool btn18H, int mm_val, int selStep, Sequencer& s) {
    // This function is called when selectedStepForEdit != -1
    // Real-time Parameter Editing for Selected Step
    if (btn16H) {
        mmNote = map(mm_val, 0, 1400, 0, 36); // Map Lidar to scale index or MIDI note
        s.setStepNote(selStep, mmNote);
#ifndef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selStep);
        Serial.print(" note set to ");
        Serial.println(mmNote);
#endif
    }
    if (btn17H) {
        float mapped_velocity = map(mm_val, 0, 1400, 0, 100) / 100.0f; // Map Lidar to 0.0 - 1.0
        mapped_velocity = daisysp::fclamp(mapped_velocity, 0.0f, 1.0f); // Clamp to 0.0 - 1.0
        s.setStepVelocity(selStep, mapped_velocity);
#ifndef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selStep);
        Serial.print(" velocity set to ");
        Serial.println(mapped_velocity);
#endif
    }
    if (btn18H) {
        float mapped_filter = map(mm_val, 0, 1400, 0, 100) / 100.0f; // Map Lidar to 0.0 - 1.0
        mapped_filter = daisysp::fclamp(mapped_filter, 0.0f, 1.0f); // Clamp to 0.0 - 1.0
        s.setStepFiltFreq(selStep, mapped_filter);
#ifndef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selStep);
        Serial.print(" filter set to ");
        Serial.println(mapped_filter);
#endif
    }
}

/** @brief Initializes Serial communication and basic platform settings for Core 1. */
static void initCore1SerialAndPlatform() {
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
    TinyUSB_Device_Init(0); // Initialize TinyUSB stack
#endif
    delay(random(333)); // Random delay for potentially better ADC noise floor for randomSeed
    randomSeed(analogRead(A0) + millis()); // Seed random number generator

#ifndef DEBUG
    Serial.begin(115200); // Start serial communication
    Serial.print(" CORE1 PLATFORM INIT ... ");
    delay(500); // Allow time for serial port to connect
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the VL53L1X distance sensor. */
static void initDistanceSensor(Melopero_VL53L1X& sensor_obj, TwoWire& wire_obj) {
#ifndef DEBUG
    Serial.print("Initializing Distance Sensor... ");
#endif
    VL53L1_Error status = 0;
    wire_obj.begin();
    sensor_obj.initI2C(0x29, wire_obj); // Initialize sensor I2C communication

    status = sensor_obj.initSensor(); // Initialize the sensor hardware
    if (status) { Serial.print("Sensor init failed with status: "); Serial.println(status); }
    status = sensor_obj.setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM); // Set distance mode
    if (status) { Serial.print("Set dist mode failed: "); Serial.println(status); }
    status = sensor_obj.setMeasurementTimingBudgetMicroSeconds(27000); // Set timing budget
    if (status) { Serial.print("Set timing budget failed: "); Serial.println(status); }
    status = sensor_obj.setInterMeasurementPeriodMilliSeconds(32); // Set inter-measurement period
    if (status) { Serial.print("Set inter-measurement failed: "); Serial.println(status); }
    status = sensor_obj.clearInterruptAndStartMeasurement(); // Start measurement
    if (status) { Serial.print("Clear int & start meas failed: "); Serial.println(status); }
#ifndef DEBUG
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the MPR121 touch sensor matrix. */
static void initTouchMatrix(Adafruit_MPR121& touch_sensor_obj) {
#ifndef DEBUG
    Serial.print("Initializing Touch Matrix... ");
#endif
    Matrix_init(&touch_sensor_obj); // Initialize matrix logic with the sensor
    Matrix_setEventHandler(matrixEventHandler); // Set the event handler for matrix events

    if (!touch_sensor_obj.begin()) {
#ifndef DEBUG
        Serial.println("ERROR - MPR121 not found... HALTING.");
#endif
        while (1) { delay(55); } // Halt if sensor not found
    } else {
#ifndef DEBUG
        Serial.println("DONE. MPR121 is Rockin!");
#endif
    }
}

/** @brief Initializes the USB MIDI interface. */
static void initMIDI(decltype(usb_midi)& midi_obj) {
#ifndef DEBUG
    Serial.print("Initializing MIDI... ");
#endif
    midi_obj.begin(MIDI_CHANNEL_OMNI); // Initialize MIDI communication on all channels
#ifndef DEBUG
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the uClock system and the Sequencer. */
static void initClockAndSequencer(decltype(uClock)& clock_obj, Sequencer& sequencer_obj) {
#ifndef DEBUG
    Serial.print("Initializing Clock & Sequencer... ");
#endif
    clock_obj.init(); // Initialize uClock library
    clock_obj.setOnSync24(onSync24Callback); // Set MIDI clock sync callback
    clock_obj.setOnClockStart(onClockStart); // Set clock start callback
    clock_obj.setOnClockStop(onClockStop);   // Set clock stop callback
    clock_obj.setOnStep(onStepCallback);     // Set per-step callback
    clock_obj.setTempo(90);                 // Set initial tempo
    clock_obj.start();                      // Start the clock
    delay(45); // Allow clock to stabilize

    sequencer_obj.init(); // Initialize the sequencer logic
#ifndef DEBUG
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the LED Matrix. */
static void initLEDMatrix(decltype(ledMatrix)& matrix_obj) {
#ifndef DEBUG
    Serial.print("Initializing LED Matrix... ");
#endif
    matrix_obj.begin(); // Initialize LED matrix hardware and clear display
#ifndef DEBUG
    Serial.println("DONE.");
#endif
}


// -----------------------------------------------------------------------------
// STATIC HELPER FUNCTION IMPLEMENTATIONS
// -----------------------------------------------------------------------------

/**
 * @brief Handles real-time parameter editing for a selected sequencer step using Lidar input.
 * Modifies note, velocity, or filter of the selected step based on which parameter button is held.
 * @param btn16H True if the 'note edit' button is held.
 * @param btn17H True if the 'velocity edit' button is held.
 * @param btn18H True if the 'filter edit' button is held.
 * @param mm_val Current Lidar distance reading.
 * @param selStep Index of the currently selected step for editing.
 * @param s Reference to the Sequencer object.
 */
static void handleParameterEditing(bool btn16H, bool btn17H, bool btn18H, int mm_val, int selStep, Sequencer& s) {
    // This function is called when selectedStepForEdit != -1
    // Real-time Parameter Editing for Selected Step
    if (btn16H) {
        mmNote = map(mm_val, 0, 1400, 0, 36); // Map Lidar to scale index or MIDI note
        s.setStepNote(selStep, mmNote);
#ifdef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selStep);
        Serial.print(" note set to ");
        Serial.println(mmNote);
#endif
    }
    if (btn17H) {
        float mapped_velocity = map(mm_val, 0, 1400, 0, 100) / 100.0f; // Map Lidar to 0.0 - 1.0
        mapped_velocity = daisysp::fclamp(mapped_velocity, 0.0f, 1.0f); // Clamp to 0.0 - 1.0
        s.setStepVelocity(selStep, mapped_velocity);
#ifdef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selStep);
        Serial.print(" velocity set to ");
        Serial.println(mapped_velocity);
#endif
    }
    if (btn18H) {
        float mapped_filter = map(mm_val, 0, 1400, 0, 100) / 100.0f; // Map Lidar to 0.0 - 1.0
        mapped_filter = daisysp::fclamp(mapped_filter, 0.0f, 1.0f); // Clamp to 0.0 - 1.0
        s.setStepFiltFreq(selStep, mapped_filter);
#ifdef DEBUG
        Serial.print("[PARAM] Step ");
        Serial.print(selStep);
        Serial.print(" filter set to ");
        Serial.println(mapped_filter);
#endif
    }
}

/** @brief Initializes Serial communication and basic platform settings for Core 1. */
static void initCore1SerialAndPlatform() {
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
    TinyUSB_Device_Init(0); // Initialize TinyUSB stack
#endif
    delay(random(333)); // Random delay for potentially better ADC noise floor for randomSeed
    randomSeed(analogRead(A0) + millis()); // Seed random number generator

#ifdef DEBUG
    Serial.begin(115200); // Start serial communication
    Serial.print(" CORE1 PLATFORM INIT ... ");
    delay(500); // Allow time for serial port to connect
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the VL53L1X distance sensor. */
static void initDistanceSensor(Melopero_VL53L1X& sensor_obj, TwoWire& wire_obj) {
#ifdef DEBUG
    Serial.print("Initializing Distance Sensor... ");
#endif
    VL53L1_Error status = 0;
    wire_obj.begin();
    sensor_obj.initI2C(0x29, wire_obj); // Initialize sensor I2C communication

    status = sensor_obj.initSensor(); // Initialize the sensor hardware
    if (status) {
#ifdef DEBUG
        Serial.print("Sensor init failed with status: "); Serial.println(status);
#endif
    }
    status = sensor_obj.setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM); // Set distance mode
    if (status) {
#ifdef DEBUG
        Serial.print("Set dist mode failed: "); Serial.println(status);
#endif
    }
    status = sensor_obj.setMeasurementTimingBudgetMicroSeconds(27000); // Set timing budget
    if (status) {
#ifdef DEBUG
        Serial.print("Set timing budget failed: "); Serial.println(status);
#endif
    }
    status = sensor_obj.setInterMeasurementPeriodMilliSeconds(32); // Set inter-measurement period
    if (status) {
#ifdef DEBUG
        Serial.print("Set inter-measurement failed: "); Serial.println(status);
#endif
    }
    status = sensor_obj.clearInterruptAndStartMeasurement(); // Start measurement
    if (status) {
#ifdef DEBUG
        Serial.print("Clear int & start meas failed: "); Serial.println(status);
#endif
    }
#ifdef DEBUG
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the MPR121 touch sensor matrix. */
static void initTouchMatrix(Adafruit_MPR121& touch_sensor_obj) {
#ifdef DEBUG
    Serial.print("Initializing Touch Matrix... ");
#endif
    Matrix_init(&touch_sensor_obj); // Initialize matrix logic with the sensor
    Matrix_setEventHandler(matrixEventHandler); // Set the event handler for matrix events

    if (!touch_sensor_obj.begin()) {
#ifdef DEBUG
        Serial.println("ERROR - MPR121 not found... HALTING.");
#endif
        while (1) { delay(55); } // Halt if sensor not found
    } else {
#ifdef DEBUG
        Serial.println("DONE. MPR121 is Rockin!");
#endif
    }
}

/** @brief Initializes the USB MIDI interface. */
static void initMIDI(decltype(usb_midi)& midi_obj) {
#ifdef DEBUG
    Serial.print("Initializing MIDI... ");
#endif
    midi_obj.begin(MIDI_CHANNEL_OMNI); // Initialize MIDI communication on all channels
#ifdef DEBUG
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the uClock system and the Sequencer. */
static void initClockAndSequencer(decltype(uClock)& clock_obj, Sequencer& sequencer_obj) {
#ifdef DEBUG
    Serial.print("Initializing Clock & Sequencer... ");
#endif
    clock_obj.init(); // Initialize uClock library
    clock_obj.setOnSync24(onSync24Callback); // Set MIDI clock sync callback
    clock_obj.setOnClockStart(onClockStart); // Set clock start callback
    clock_obj.setOnClockStop(onClockStop);   // Set clock stop callback
    clock_obj.setOnStep(onStepCallback);     // Set per-step callback
    clock_obj.setTempo(90);                 // Set initial tempo
    clock_obj.start();                      // Start the clock
    delay(45); // Allow clock to stabilize

    sequencer_obj.init(); // Initialize the sequencer logic
#ifdef DEBUG
    Serial.println("DONE.");
#endif
}

/** @brief Initializes the LED Matrix. */
static void initLEDMatrix(decltype(ledMatrix)& matrix_obj) {
#ifdef DEBUG
    Serial.print("Initializing LED Matrix... ");
#endif
    matrix_obj.begin(); // Initialize LED matrix hardware and clear display
#ifdef DEBUG
    Serial.println("DONE.");
#endif
}


// -----------------------------------------------------------------------------
// 9. MAIN LOOPS
// -----------------------------------------------------------------------------



/**
 * @brief Main audio loop (core 0).
 */
void loop() {
    audio_buffer_t *buf = take_audio_buffer(producer_pool, true);
    if (buf) {
        fill_audio_buffer(buf);
        give_audio_buffer(producer_pool, buf);
    }
}

/**
 * @brief Main control loop (core 1).
 */
void loop1() {
    usb_midi.read();

    // Use separate timers for different update rates
    unsigned long currentMillis1 = millis();
    unsigned long currentMillis2 = millis();

    // Update step LEDs at 20ms intervals
    if (currentMillis2 - previousMillis2 >= 20) {
        updateStepLEDs(ledMatrix, seq, selectedStepForEdit, button16Held,
                       button17Held, button18Held, recordButtonHeld, mm);
    }

    // Update sensor and matrix, and handle parameter editing at 1ms intervals
    if (currentMillis1 - previousMillis1 >= 1) {
        update();
        previousMillis1 = currentMillis1;
        Matrix_scan(); // Process touch matrix events

    if (selectedStepForEdit != -1) {
        // The LED feedback logic for a selected (but not actively being parameter-edited) step
        // (e.g., pulsing cyan) is handled by updateStepLEDs().
        // The specific logic for parameter button-held (e.g., yellow modulated by Lidar)
        // is also handled by updateStepLEDs() based on button16Held etc.
        // Thus, no direct LED manipulation is needed here before calling handleParameterEditing.

        // Handle actual parameter changes
        handleParameterEditing(button16Held, button17Held, button18Held, mm, selectedStepForEdit, seq);
    }
}}