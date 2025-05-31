# Pico2DSP2CoreWorks Developer Manual

## Table of Contents

1. [Introduction](#introduction)
2. [System Architecture Overview](#system-architecture-overview)
3. [Audio Module](#audio-module)  
   - [Overview & Data Structures](#audio-overview--data-structures)  
   - [Initialization & Configuration](#audio-initialization--configuration)  
   - [Core Functionality](#audio-core-functionality)  
   - [API Reference](#audio-api-reference)  
   - [Example: Audio Output Setup](#example-audio-output-setup)  
4. [Sequencer Module](#sequencer-module)  
   - [Architecture & Data Structures](#architecture--data-structures)  
   - [Initialization & Configuration](#initialization--configuration)  
   - [Core Functionality](#core-functionality)  
   - [API Reference](#api-reference)  
   - [Usage Patterns & Best Practices](#usage-patterns--best-practices)  
   - [Example: Basic Sequencer Usage](#example-basic-sequencer-usage)  
5. [Matrix Module](#matrix-module)  
   - [Architecture & Data Structures](#matrix-architecture--data-structures)  
   - [Initialization & Configuration](#matrix-initialization--configuration)  
   - [Scanning, Debouncing, and Event Handling](#scanning-debouncing-and-event-handling)  
   - [API Reference](#matrix-api-reference)  
   - [Example: Matrix Usage](#example-matrix-usage)  
6. [uClock Integration](#uclock-integration)  
7. [Module Interactions](#module-interactions)  
8. [Performance Considerations & Optimization Tips](#performance-considerations--optimization-tips)  
9. [Troubleshooting Common Issues](#troubleshooting-common-issues)  
10. [Extending and Integrating](#extending-and-integrating)  

---

## Introduction

This manual provides a comprehensive guide to the **Audio**, **Sequencer**, and **Matrix** modules of the Pico2DSP2CoreWorks firmware. It covers architecture, configuration, core APIs, and integration patterns for developers.

**Prerequisites:**

- RP2040 development environment with Pico SDK and C++17 support  
- Adafruit MPR121 and SSD1306 libraries installed  
- I2C, SPI, and PIO interfaces configured correctly  

---

## System Architecture Overview

```mermaid
flowchart TD
    subgraph Hardware
        MPR121[Capacitive Touch Sensor (MPR121)]
        OLED[OLED Display]
        AudioOut[Audio Output (I2S)]
    end

    subgraph Firmware
        Audio[Audio Module]
        Sequencer[Sequencer Module]
        Matrix[Matrix Module]
        uClock[uClock (Clock Sync)]
        Display[OLED Visualization]
    end

    MPR121 -- I2C --> Matrix
    Matrix -- Button Events --> Sequencer
    uClock -- Step Callback --> Sequencer
    Sequencer -- State --> Audio
    Sequencer -- State --> Display
    Display -- SPI/I2C --> OLED
    Audio -- I2S --> AudioOut
```

---

## Audio Module

### Overview & Data Structures

The Audio module handles buffer management, sample conversion, and PIO-based I2S output. Core types:

- `audio_buffer_format_t`: describes sample format and stride ([`audio.h`](src/audio/audio.h:57))  
- `audio_buffer_t`: holds a `mem_buffer_t` and format metadata ([`audio.h`](src/audio/audio.h:64))  
- `audio_buffer_pool_t`: manages producer/consumer buffer pools ([`audio.h`](src/audio/audio.h:76))  
- `audio_connection_t`: defines callbacks for buffer give/take ([`audio.h`](src/audio/audio.h:93))  
- `mem_buffer_t`: raw memory wrapper ([`buffer.h`](src/audio/buffer.h:42))  
- Sample conversion templates (`FmtS16`, `Mono`, `Stereo`, `converting_copy`) ([`sample_conversion.h`](src/audio/sample_conversion.h:16))  

### Initialization & Configuration

1. Allocate producer pool:  
   ```cpp
   producer_pool = audio_new_producer_pool(
       &my_buffer_format, NUM_AUDIO_BUFFERS, SAMPLES_PER_BUFFER
   ); // see [`audio.h`](src/audio/audio.h:114)
   ```

2. Configure PIO-based I2S:  
   ```cpp
   audio_i2s_config_t cfg = {
     .data_pin = PICO_AUDIO_I2S_DATA_PIN,
     .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
     .dma_channel = 0,
     .pio_sm = 0
   };
   audio_i2s_setup(&my_audio_format, &cfg); // see [`audio_i2s.h`](src/audio/audio_i2s.h:133)
   audio_i2s_connect(producer_pool);        // see [`audio_i2s.h`](src/audio/audio_i2s.h:153)
   audio_i2s_set_enabled(true);             // see [`audio_i2s.h`](src/audio/audio_i2s.h:182)
   ```

### Core Functionality

- `take_audio_buffer(pool, block)`: get free buffer ([`audio.h`](src/audio/audio.h:169))  
- `give_audio_buffer(pool, buffer)`: queue buffer ([`audio.h`](src/audio/audio.h:162))  
- Internal queue operations: `queue_full_audio_buffer`, `queue_free_audio_buffer`  
- Sample conversion on consumer take: template `consumer_pool_take<ToFmt,FromFmt>`  

### API Reference

- `audio_new_producer_pool(format, count, samples)`  
- `audio_new_consumer_pool(format, count, samples)`  
- `audio_new_buffer(format, samples)`  
- `audio_new_wrapping_buffer(format, buffer)`  
- `audio_init_buffer(buffer, format, samples)`  
- `get_free_audio_buffer(context, block)`  
- `queue_free_audio_buffer(context, buffer)`  
- `get_full_audio_buffer(context, block)`  
- `queue_full_audio_buffer(context, buffer)`  
- `audio_i2s_setup(format, config)`  
- `audio_i2s_connect(producer)`  
- `audio_i2s_connect_s8(producer)`  
- `audio_i2s_connect_extra(...)`  
- `audio_i2s_set_enabled(enabled)`  

### Example: Audio Output Setup

```cpp
// Setup audio buffers
static audio_buffer_format_t my_buffer_format = {
  .format = &my_audio_format,
  .sample_stride = 4
};
producer_pool = audio_new_producer_pool(&my_buffer_format, 3, 256);
// Initialize I2S
audio_i2s_setup(&my_audio_format, &i2s_config);
audio_i2s_connect(producer_pool);
audio_i2s_set_enabled(true);
```

---

## Sequencer Module

### Architecture & Data Structures

- **16-step monophonic sequencer**: Each step has ON/OFF state, scale index, and gate.  
- Core types in [`SequencerDefs.h`](src/sequencer/SequencerDefs.h:20):  
  ```cpp
  enum class StepState : uint8_t { OFF = 0, ON = 1 };
  struct Step {
    StepState state;
    uint8_t note;    // scale index
    bool gate;
  };
  constexpr uint8_t SEQUENCER_NUM_STEPS = 16;
  constexpr uint8_t SCALE_ARRAY_SIZE = 40;
  ```
- `SequencerState`: holds `steps[]`, `playhead`, and `running` ([`SequencerDefs.h`](src/sequencer/SequencerDefs.h:41))  
- `Sequencer` class in [`Sequencer.h`](src/sequencer/Sequencer.h:30)  

### Initialization & Configuration

```cpp
Sequencer seq;
bool ok = seq.init();  // reset state and validate
seq.start();           // begin playback
seq.stop();            // halt playback
seq.reset();           // reset to default state
```

### Core Functionality

- `advanceStep(current_uclock_step)`: update playhead, send MIDI and trigger envelope ([`Sequencer.cpp`](src/sequencer/Sequencer.cpp:161))  
- `toggleStep(idx)`: flip step ON/OFF ([`Sequencer.cpp`](src/sequencer/Sequencer.cpp:279))  
- `setStepNote(idx, noteIndex)`: assign scale index ([`Sequencer.cpp`](src/sequencer/Sequencer.cpp:299))  

### API Reference

- `bool init()`  
- `bool hasError() const`  
- `void start()`  
- `void stop()`  
- `void reset()`  
- `void advanceStep(uint8_t step)`  
- `void toggleStep(uint8_t idx)`  
- `void setStepNote(uint8_t idx, uint8_t note)`  
- `void setOscillatorFrequency(uint8_t midiNote)`  
- `void triggerEnvelope()`  
- `void releaseEnvelope()`  
- `const Step& getStep(uint8_t idx) const`  
- `uint8_t getPlayhead() const`  
- `bool isRunning() const`  
- `int8_t getLastNote() const`  
- `void setLastNote(int8_t note)`  
- `const SequencerState& getState() const`  

### Usage Patterns & Best Practices

- Always call `init()` before interaction.  
- Advance steps only on clock events.  
- Use `getState()` or `getStep()` for UI updates.  
- Use `toggleStep()` and `setStepNote()` for live editing.  

### Example: Basic Sequencer Usage

```cpp
Sequencer seq;
seq.init();
seq.start();
// On clock tick:
seq.advanceStep(tick % SEQUENCER_NUM_STEPS);
```

---

## Matrix Module

### Matrix Architecture & Data Structures

- **4x8 button matrix** via MPR121 capacitive touch sensor  
- `MatrixButton`: maps row/column inputs ([`Matrix.h`](src/matrix/Matrix.h:40))  
- `MATRIX_BUTTON_COUNT = 32` ([`Matrix.h`](src/matrix/Matrix.h:33))  
- Raw vs. debounced states: internal arrays in [`Matrix.cpp`](src/matrix/Matrix.cpp:13)  

### Matrix Initialization & Configuration

```cpp
Adafruit_MPR121 touchSensor;
Matrix_init(&touchSensor);                     // see [`Matrix.h`](src/matrix/Matrix.h:58)
Matrix_setEventHandler(myEventHandler);        // set callback
```

### Scanning, Debouncing, and Event Handling

- `Matrix_scan()`: read raw bits and update debounced state ([`Matrix.cpp`](src/matrix/Matrix.cpp:105))  
- Debounce delay: `DEBOUNCE_MS = 10` ms ([`Matrix.cpp`](src/matrix/Matrix.cpp:21))  
- Events dispatched via `eventHandler(const MatrixButtonEvent&)`  

### Matrix API Reference

- `void Matrix_init(Adafruit_MPR121 *sensor)`  
- `void Matrix_scan()`  
- `bool Matrix_getButtonState(uint8_t idx)`  
- `void Matrix_setEventHandler(void (*handler)(const MatrixButtonEvent &))`  
- `void Matrix_printState()`  

### Example: Matrix Usage

```cpp
Matrix_init(&touchSensor);
Matrix_setEventHandler([](auto &evt){
  if(evt.type==MATRIX_BUTTON_PRESSED){
    seq.toggleStep(evt.buttonIndex);
  }
});
// In main loop:
Matrix_scan();
```

---

## uClock Integration

- `uClock` provides MIDI clock sync and step callbacks  
- Key setup in [`Pico2DSP2CoreWorks.ino`](Pico2DSP2CoreWorks.ino:394):  
```cpp
uClock.init();
uClock.setOnSync24(onSync24Callback);
uClock.setOnClockStart(onClockStart);
uClock.setOnClockStop(onClockStop);
uClock.setOnStep(onStepCallback);
uClock.setTempo(126);
uClock.start();
```

### Sequencer Step Advancement

- `onStepCallback(uint32_t step)`: wraps `step % SEQUENCER_NUM_STEPS` and calls `seq.advanceStep(...)`  

---

## Module Interactions

- **Sequencer–Matrix**: map touch events to `seq.toggleStep()` and display updates.  
- **Sequencer–Audio**: `advanceStep()` triggers MIDI via `usb_midi.sendNoteOn/Off()` and updates `note1` for synthesis.  
- **Audio–Display**: render audio/synth state (e.g., waveform or levels) via custom code.  

---

## Performance Considerations & Optimization Tips

- Call `Matrix_scan()` at ≥1 kHz for responsive input.  
- Keep audio buffer fill loops efficient to prevent underruns.  
- Offload timing-critical work to PIO and DMA.  
- Use hardware spinlocks and `__wfe/__sev` for low-overhead synchronization.  

---

## Troubleshooting Common Issues

- **No I2S audio**: verify `audio_i2s_set_enabled(true)` and pin defines in [`audio_pins.h`](src/audio/audio_pins.h:3).  
- **Stuck buttons**: check `DEBOUNCE_MS` and wiring for MPR121.  
- **Sequencer errors**: use `seq.hasError()` and inspect state via `getState()`.  
- **Clock sync**: ensure `uClock` callbacks are registered before `uClock.start()`.  

---

## Extending and Integrating

- Add new audio effects by extending DSP modules under `src/dsp/`.  
- Support custom matrix layouts by modifying `setupMatrixMapping()` ([`Matrix.cpp`](src/matrix/Matrix.cpp:24)).  
- Extend sequencer with polyphony by increasing `SEQUENCER_NUM_STEPS` and adapting `advanceStep()`.  
- Integrate additional outputs (CV/Gate) using similar buffer/connection patterns from Audio module.  

---