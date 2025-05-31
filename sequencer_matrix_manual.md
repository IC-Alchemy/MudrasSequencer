# Pico2DSP2CoreWorks Sequencer & Matrix Developer Manual

## Table of Contents

1. [Introduction](#introduction)
2. [System Architecture Overview](#system-architecture-overview)
3. [Sequencer Module](#sequencer-module)
    - [Architecture & Data Structures](#architecture--data-structures)
    - [Initialization & Configuration](#initialization--configuration)
    - [Core Functionality](#core-functionality)
    - [API Reference](#api-reference)
    - [Usage Patterns & Best Practices](#usage-patterns--best-practices)
    - [Example: Basic Sequencer Usage](#example-basic-sequencer-usage)
4. [Matrix Module](#matrix-module)
    - [Architecture & Data Structures](#matrix-architecture--data-structures)
    - [Initialization & Configuration](#matrix-initialization--configuration)
    - [Scanning, Debouncing, and Event Handling](#scanning-debouncing-and-event-handling)
    - [API Reference](#matrix-api-reference)
    - [Example: Matrix Usage](#example-matrix-usage)
5. [uClock Integration](#uclock-integration)
    - [uClock’s Role](#uclocks-role)
    - [uClock Function Calls](#uclock-function-calls)
    - [Sequencer Step Advancement](#sequencer-step-advancement)
    - [Example: Clock-Synced Sequencer](#example-clock-synced-sequencer)
6. [Sequencer–Matrix Interaction](#sequencer–matrix-interaction)
    - [Mapping Matrix Buttons to Sequencer Steps](#mapping-matrix-buttons-to-sequencer-steps)
    - [Example: Live Step Editing](#example-live-step-editing)
7. [Performance Considerations & Optimization Tips](#performance-considerations--optimization-tips)
8. [Troubleshooting Common Integration Issues](#troubleshooting-common-integration-issues)
9. [Extending and Integrating](#extending-and-integrating)

---

## Introduction

This manual provides a comprehensive guide to the `sequencer` and `matrix` modules in Pico2DSP2CoreWorks, focusing on their architecture, functionality, and integration—especially with the uClock timing system. It is intended for developers seeking to understand, extend, or integrate these modules into their own projects.

---

### Prerequisites

- RP2040 development environment with Pico SDK and C++17 support
- Adafruit MPR121 and SSD1306 libraries installed
- I2C and SPI interfaces initialized correctly

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
        Matrix[Matrix Module]
        Sequencer[Sequencer Module]
        uClock[uClock (Clock Sync)]
        Audio[Audio Engine]
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

## Sequencer Module

### Architecture & Data Structures

- **16-step monophonic sequencer**: Each step has ON/OFF state, MIDI note, and gate.
- **State**: Managed via `SequencerState`, which holds all steps, playhead, and running/error flags.
- **Step**: Defined in [`SequencerDefs.h`](src/sequencer/SequencerDefs.h:26):
  ```cpp
  struct Step {
    StepState state; // ON/OFF
    uint8_t note;    // MIDI note number (0-127)
    bool gate;       // Gate output state
    Step() : state(StepState::OFF), note(60), gate(false) {}
  };
  ```

### Initialization & Configuration

- **Create and initialize**:
  ```cpp
  Sequencer seq;
  seq.init();
  ```
- **Reset**: `seq.reset();`
- **Start/Stop**: `seq.start();` / `seq.stop();`

### Core Functionality

- **Advance step**: `seq.advanceStep();` (typically called from a clock callback)
- **Toggle step**: `seq.toggleStep(stepIdx);`
- **Set step note**: `seq.setStepNote(stepIdx, note);`
- **Query state**: `seq.getStep(idx);`, `seq.getPlayhead();`, `seq.isRunning();`

### API Reference

See [`Sequencer.h`](src/sequencer/Sequencer.h:24) for full details. Key methods:
- `bool init();`
- `void start();`
- `void stop();`
- `void reset();`
- `void advanceStep();`
- `void toggleStep(uint8_t stepIdx);`
- `void setStepNote(uint8_t stepIdx, uint8_t note);`
- `const Step& getStep(uint8_t stepIdx) const;`
- `uint8_t getPlayhead() const;`
- `bool isRunning() const;`
- `int8_t getLastNote() const;`
- `void setLastNote(int8_t note);`
- `const SequencerState& getState() const;`

### Usage Patterns & Best Practices

- **Always call `init()`** before use.
- **Advance steps** only in response to clock events (see uClock integration).
- **Use `getStep()`** to inspect or display step state.
- **Use `toggleStep()` and `setStepNote()`** for live editing (e.g., from matrix input).

### Example: Basic Sequencer Usage

```cpp
#include "Sequencer.h"  // see [`Sequencer.h`](src/sequencer/Sequencer.h:24)

Sequencer seq;

void setup() {
    seq.init();
    seq.start();
}

void loop() {
    // Advance step on external clock or timer
    if (should_advance_step()) {
        seq.advanceStep();
    }
}
```

---

## Matrix Module

### Matrix Architecture & Data Structures

- **4x8 (32-button) matrix** mapped to MPR121 capacitive touch sensor.
- **Button mapping**: Each button is a (row, col) pair or single col (for row 4).
- **Debounced state**: Internal logic ensures reliable button state changes.

### Matrix Initialization & Configuration

- **Initialize**:
  ```cpp
  Adafruit_MPR121 touchSensor;
  Matrix_init(&touchSensor);
  ```
- **Set event handler** (optional):
  ```cpp
  void myEventHandler(const MatrixButtonEvent &evt) {
      // Handle button press/release
  }
  Matrix_setEventHandler(myEventHandler);
  ```

### Scanning, Debouncing, and Event Handling

- **Scan**: Call `Matrix_scan();` frequently (e.g., every 1ms).
- **Query state**: `bool pressed = Matrix_getButtonState(idx);`
- **Print state**: `Matrix_printState();` (for debugging)

### Matrix API Reference

See [`Matrix.h`](src/matrix/Matrix.h:58) for full details. Key functions:
- `void Matrix_init(Adafruit_MPR121 *sensor);`
- `void Matrix_scan();`
- `bool Matrix_getButtonState(uint8_t idx);`
- `void Matrix_setEventHandler(void (*handler)(const MatrixButtonEvent &));`
- `void Matrix_printState();`

### Example: Matrix Usage

```cpp
#include "Matrix.h"  // see [`Matrix.h`](src/matrix/Matrix.h:58)
#include <Adafruit_MPR121.h>

Adafruit_MPR121 touchSensor;

void setup() {
    Matrix_init(&touchSensor);
    Matrix_setEventHandler(myEventHandler);
}

void myEventHandler(const MatrixButtonEvent &evt) {
    if (evt.type == MATRIX_BUTTON_PRESSED) {
        // Respond to button press
    }
}

void loop() {
    Matrix_scan();
}
```

---

## uClock Integration

### uClock’s Role

- **uClock** provides MIDI/clock synchronization and timing callbacks.
- **Sequencer step advancement** is synchronized to clock events via uClock.

### uClock Function Calls

Key calls in [`Pico2DSP2CoreWorks.ino`](Pico2DSP2CoreWorks.ino:421):
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

- **onStepCallback** is called on each clock step:
  ```cpp
  void onStepCallback(uint32_t tick) {
      uint16_t step = tick % SEQUENCER_NUM_STEPS;
      const auto& currentStep = seq.getStep(step);
      // ... handle note/gate logic ...
      seq.setLastNote(currentStep.note);
  }
  ```
- **Best practice**: All timing-sensitive sequencer operations should be performed in uClock callbacks.

### Example: Clock-Synced Sequencer

```cpp
void onStepCallback(uint32_t tick) {
    seq.advanceStep();
    // Additional note/gate/MIDI logic here
}

void setup() {
    uClock.init();
    uClock.setOnStep(onStepCallback);
    uClock.start();
}
```

---

## Sequencer–Matrix Interaction

### Mapping Matrix Buttons to Sequencer Steps

- **Typical pattern**: Map each matrix button to a sequencer step for live editing.
- **Example mapping**:
  ```cpp
  void myEventHandler(const MatrixButtonEvent &evt) {
      if (evt.type == MATRIX_BUTTON_PRESSED) {
          seq.toggleStep(evt.buttonIndex);
      }
  }
  ```

### Example: Live Step Editing

```cpp
Matrix_setEventHandler([](const MatrixButtonEvent &evt) {
    if (evt.type == MATRIX_BUTTON_PRESSED) {
        seq.toggleStep(evt.buttonIndex);
    }
});
```

---

## Performance Considerations & Optimization Tips

- **Matrix scanning**: Call `Matrix_scan()` as frequently as possible (ideally every 1ms) for responsive input.
- **Debouncing**: The matrix module handles debouncing internally; avoid redundant debouncing in your code.
- **Sequencer step advancement**: Always advance steps in response to clock events, not in the main loop, to ensure tight timing.
- **Audio processing**: Keep audio buffer filling and sequencer logic efficient to avoid audio dropouts.
- **Profiling**: Use serial prints or hardware timers to measure and optimize critical code paths.

---

## Troubleshooting Common Integration Issues

- **Clock sync/timing problems**:
    - Ensure uClock is properly initialized and started.
    - Verify that `onStepCallback` is being called as expected.
- **Matrix input issues**:
    - Check MPR121 wiring and I2C address.
    - Use `Matrix_printState()` to debug button states.
- **MIDI/gate output issues**:
    - Confirm MIDI interface is initialized.
    - Check that note/gate logic is handled in the correct callback.
- **General tips**:
    - Use serial output for debugging.
    - Isolate modules to test them independently before full integration.

---

## Extending and Integrating

- **Adding new step properties**: Extend the `Step` struct and update sequencer logic accordingly.
- **Custom matrix mappings**: Modify the event handler to implement custom behaviors (e.g., set note, velocity, etc.).
- **Integration with other modules**: Use the sequencer’s state query methods to drive other outputs (e.g., CV/gate, LEDs).
- **Best practices**:
    - Keep timing-sensitive code in clock callbacks.
    - Use the provided APIs for state changes to maintain consistency.

---