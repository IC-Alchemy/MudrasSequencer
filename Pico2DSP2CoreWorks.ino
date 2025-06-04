
  #define DEBUG     Turns on a ton of serial prints, don't use unless you need it


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

// -----------------------------------------------------------------------------
// 2. CONSTANTS & GLOBAL VARIABLES
// -----------------------------------------------------------------------------

// --- LED Matrix ---
LEDMatrix ledMatrix;

// --- Step Selection & Pad Timing ---
int selectedStepForEdit = -1; // -1 means no step selected

// --- Distance Sensor ---
int raw_mm = 0;
int mm = 0;
int mmNote = 7, mmVelocity = 50, mmFiltFreq = 2222;
Melopero_VL53L1X sensor;

// --- I2S Pin Configuration ---
#define PICO_AUDIO_I2S_DATA_PIN 15
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16
#define IRQ_PIN 1

// --- Sequencer ---
Sequencer seq;

// --- MIDI & Clock ---
Adafruit_USBD_MIDI raw_usb_midi;
midi::SerialMIDI<Adafruit_USBD_MIDI> serial_usb_midi(raw_usb_midi);
midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>> usb_midi(serial_usb_midi);
uint8_t bpm_blink_timer = 1;

// --- Touch Matrix ---
const int PIN_TOUCH_IRQ = 6;
Adafruit_MPR121 touchSensor = Adafruit_MPR121();

// --- Multicore Communication ---
volatile int note1 = 48, note2 = 48;
volatile bool trig1, trig2, trigenv1, trigenv2, dualEnvFlag;
volatile bool buttonEventFlag = false;
volatile uint8_t buttonEventIndex = 0;
volatile uint8_t buttonEventType = 0;
volatile uint8_t Note = 0;
volatile float vel1 = 0;
volatile float freq1 = 0.0f;

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
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
const long interval = 1; // ms

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

/**
 * @brief Smooths filter frequency changes to avoid abrupt jumps.
 */
float applyFilterFrequency(float targetFreq) {
    static float currentFreq = 0.0f;
    const float smoothingAlpha = 0.1f;
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
        // 1. Set Oscillator Frequencies based on note1 (from sequencer)
        float osc_base_freq = daisysp::mtof(note1); // note1 is MIDI note from sequencer
        osc1.SetFreq(osc_base_freq);
        osc2.SetFreq(osc_base_freq * 1.003f); // Slight detune
        osc3.SetFreq(osc_base_freq * 0.997f); // Slight detune

        // 2. Process Amplitude Envelope (env1) based on trigenv1 (from sequencer gate)
        float current_amp_env_value = env1.Process(trigenv1);

        // 3. Set Filter Frequency based on freq1 (from sequencer step's filter value)
        float target_filter_freq = daisysp::fmax(20.f, freq1);
        filter.SetFreq(target_filter_freq);

        // 4. Generate and sum oscillator outputs
        float osc_sum = osc1.Process() + osc2.Process() + osc3.Process();

        // 5. Process summed oscillators through the filter
        float filtered_signal = filter.Process(osc_sum);

        // 6. Apply amplitude envelope and step velocity to the filtered signal
        float final_audio_signal = filtered_signal * current_amp_env_value * vel1;

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
    // osc5.Init(SAMPLE_RATE); //osc6.Init(SAMPLE_RATE); //osc7.Init(SAMPLE_RATE); //osc8.Init(SAMPLE_RATE);
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
    // osc5.SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_SAW);
}

// -----------------------------------------------------------------------------
// 6. MATRIX EVENT HANDLER
// -----------------------------------------------------------------------------

/**
 * @brief Handles button events from the touch matrix.
 */
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
#ifndef DEBUG
                Serial.print("[MATRIX] ");
                Serial.print(buttonName);
                Serial.println(" held.");
#endif
            } else if (evt.type == MATRIX_BUTTON_RELEASED) {
                *buttonHeld = false;
#ifndef DEBUG
                Serial.print("[MATRIX] ");
                Serial.print(buttonName);
                Serial.println(" released.");
#endif
            }
        } else if (evt.type == MATRIX_BUTTON_PRESSED) {
#ifndef DEBUG
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
  //  Serial.println("[uCLOCK] onClockStart() called.");
    usb_midi.sendRealTime(midi::Start); // MIDI Start message
    seq.start();
    seq.advanceStep(0); // Immediately trigger the first step so sound is produced at startup
}

/**
 * @brief Handles clock stop event.
 */
void onClockStop() {
    Serial.println("[uCLOCK] onClockStop() called.");
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
 * @brief Initializes envelope trigger flags.
 */
void initEnvelopeTriggers() {
    trigenv1 = false;
    trigenv2 = false;
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
    initEnvelopeTriggers();

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
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
    TinyUSB_Device_Init(0);
#endif

    delay(random(333));
    randomSeed(analogRead(A0) + millis()); // Use an unconnected analog pin and millis for better seed

#ifndef DEBUG
    Serial.begin(115200);
    Serial.print(" CORE1 SETUP1 ... ");
    delay(500);
#endif

    VL53L1_Error status = 0;
    Wire.begin(); 
    sensor.initI2C(0x29, Wire);

    status = sensor.initSensor();
    status = sensor.setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
    status = sensor.setMeasurementTimingBudgetMicroSeconds(27000);
    status = sensor.setInterMeasurementPeriodMilliSeconds(32);
    status = sensor.clearInterruptAndStartMeasurement();

    Matrix_init(&touchSensor);
    Matrix_setEventHandler(matrixEventHandler); 

    usb_midi.begin(MIDI_CHANNEL_OMNI);

    pinMode(PIN_TOUCH_IRQ, INPUT);

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
        Serial.print(" ...uClock is GOOD... ");
        delay(345);
        Serial.println("Core 1: Setup1 complete.");
        delay(500);
#endif
    }
    seq.init();
    ledMatrix.begin();
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
        // Check if selected step is currently playing and gate is ON
        bool isPlayhead = (seq.getState().playhead == selectedStepForEdit);
        bool gateOn = seq.getStep(selectedStepForEdit).gate;

        if (isPlayhead && gateOn) {
            // Gate state indication should override cyan pulse (example: white)
            // Removed call to setStepLedColor to fix compilation error
            // setStepLedColor((uint8_t)selectedStepForEdit, 255, 255, 255);
        } else {
            // Smooth pulse: sine wave, 1.5 Hz
            // --- LED Pulsing Cyan Feedback for Selected Step ---
            float t = millis() / 1000.0f;
            float pulse = 0.5f * (1.0f + sinf(2.0f * 3.1415926f * 1.5f * t)); // 0..1
            uint8_t brightness = (uint8_t)(pulse * 255.0f);
            // Cyan: (0, brightness, brightness)
            // Removed call to setStepLedColor to fix compilation error
            // setStepLedColor((uint8_t)selectedStepForEdit, 0, brightness, brightness);
        }

        // --- Parameter Editing Logic for Selected Step ---
        // Real-time Parameter Editing for Selected Step
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
    }
}}