# Pico2DSP2Core

# Pico2DSP2CoreWorks Firmware Documentation

## Overview

Pico2DSP2CoreWorks is a modular firmware designed for the Raspberry Pi Pico platform, providing a digital synthesizer with integrated sequencing, touch input, MIDI, and OLED visualization. The firmware features:

- Audio synthesis with multiple oscillators and ADSR envelopes
- A 16-step sequencer with note, gate, velocity, and filter control
- OLED display for sequencer visualization
- Capacitive touch matrix input using MPR121 sensor
- MIDI input/output and clock synchronization
- Distance sensor integration for expressive control

This documentation covers the main firmware file (`Pico2DSP2CoreWorks.ino`), the sequencer module (`src/sequencer`), and the touch matrix module (`src/matrix`), explaining their functionality, usage, and key components.

---

## Table of Contents

- [Main Firmware (`Pico2DSP2CoreWorks.ino`)](#main-firmware-pico2dsp2coreworksino)
  - [Audio Synthesis](#audio-synthesis)
  - [Sequencer Integration](#sequencer-integration)
  - [OLED Display](#oled-display)
  - [Touch Matrix Input](#touch-matrix-input)
  - [MIDI and Clock Handling](#midi-and-clock-handling)
  - [Distance Sensor Integration](#distance-sensor-integration)
  - [Setup and Loop Functions](#setup-and-loop-functions)
- [Sequencer Module (`src/sequencer`)](#sequencer-module-srcsequencer)
  - [Sequencer Overview](#sequencer-overview)
  - [Key Classes and Data Structures](#key-classes-and-data-structures)
  - [Core Functionality](#core-functionality)
  - [Usage Example](#usage-example)
- [Touch Matrix Module (`src/matrix`)](#touch-matrix-module-srcmatrix)
  - [Matrix Overview](#matrix-overview)
  - [Matrix Scanning and Event Dispatch](#matrix-scanning-and-event-dispatch)
  - [Event Handling](#event-handling)
  - [API Functions](#api-functions)

---

## Main Firmware (`Pico2DSP2CoreWorks.ino`)

This is the central firmware file that integrates audio synthesis, sequencing, display, touch input, MIDI, and sensor data.

### Audio Synthesis

- Uses multiple oscillators (`osc1` to `osc4`) from the DaisySP library to generate sound waves.
- ADSR envelopes (`env1`, `env2`) shape the amplitude of the oscillators.
- Audio output is handled via I2S protocol with a buffer pool for efficient streaming.
- The `fill_audio_buffer()` function synthesizes stereo audio samples by mixing oscillator outputs modulated by envelopes.
- Oscillator frequencies are set based on MIDI notes derived from the sequencer.

### Sequencer Integration

- The `Sequencer` class instance `seq` manages a 16-step sequencer.
- Steps have properties: gate (on/off), slide, note (scale index), velocity, and filter.
- The sequencer advances steps in sync with the clock (`uClock`), triggering MIDI note on/off and envelopes.
- The playhead position and step states are visualized on the OLED display.
- The sequencer state can be modified via touch matrix button presses.

### OLED Display

- Uses an SSD1306 OLED display (128x64 pixels) for sequencer visualization.
- The display shows either the note page or gate page, with a playhead underline indicating the current step.
- The `drawSequencerOLED()` function updates the display based on the sequencer state.
- Initialization is done in `initOLED()`.

### Touch Matrix Input

- Uses an Adafruit MPR121 capacitive touch sensor to scan a 4x8 button matrix (32 buttons).
- The matrix is initialized and scanned regularly.
- Button events are handled by `matrixEventHandler()`, which toggles sequencer steps or modifies note/velocity when specific buttons are held.
- The matrix module dispatches button press and release events via callbacks.
- A new rising edge callback feature triggers a user-defined function when a button is first pressed.
- Debouncing is handled externally by the application.

### MIDI and Clock Handling

- MIDI communication is handled via USB MIDI using the Adafruit TinyUSB and MIDI libraries.
- The clock system (`uClock`) synchronizes sequencer steps and MIDI clock messages.
- Callbacks handle clock start, stop, step advance, and MIDI clock sync.
- The BPM LED indicator blinks in sync with the clock.

### Distance Sensor Integration

- A VL53L1X time-of-flight distance sensor measures distance in millimeters.
- Distance data is mapped to note and velocity values when specific buttons are held.
- This allows expressive control of the sequencer notes and velocities based on hand position.

### Setup and Loop Functions

- `setup()` initializes oscillators, audio output, and envelopes.
- `setup1()` initializes serial communication, USB MIDI, clock system, distance sensor, touch matrix, and sequencer.
- `loop()` handles audio buffer processing.
- `loop1()` handles MIDI input, sensor updates, matrix scanning, and mapping sensor data to sequencer parameters.

---

## Sequencer Module (`src/sequencer`)

### Sequencer Overview

The sequencer module implements a modular 16-step sequencer with step toggling, note assignment, velocity, and filter control. It integrates with the clock system and MIDI output.

### Key Classes and Data Structures

- `Step` struct: Represents a single step with properties:
  - `gate` (bool): Step on/off
  - `slide` (bool): Slide to next note
  - `note` (int): Scale index (0-24)
  - `velocity` (float): 0.0 to 1.0
  - `filter` (float): 0.0 to 1.0
- `SequencerState` struct: Holds an array of 16 `Step`s, playhead position, and running state.
- `Sequencer` class: Manages sequencer state, step toggling, playhead advancement, MIDI note handling, and envelope triggering.

### Core Functionality

- `init()`: Initializes sequencer state and validates integrity.
- `start()`, `stop()`, `reset()`: Control sequencer running state.
- `advanceStep(step)`: Advances playhead, sends MIDI note on/off, triggers envelopes.
- `toggleStep(stepIdx)`: Toggles gate on/off for a step.
- `setStepNote()`, `setStepVelocity()`, `setStep()`: Set step parameters.
- `getStep()`, `getPlayhead()`, `isRunning()`: Query sequencer state.
- Internal handling of last played note to ensure proper MIDI note off/on sequencing.

### Usage Example

```cpp
#include "Sequencer.h"

Sequencer seq;

void setup() {
  if (!seq.init()) {
    Serial.println("Sequencer initialization failed");
  }
  seq.start();

  // Configure step 0
  seq.setStep(0, true, false, 8, 0.75f, 0.3f);
}

void onClockTick(uint8_t step) {
  seq.advanceStep(step);
  const Step& stepData = seq.getStep(step);
  // Use stepData for MIDI or synth control
}
```

---

## Touch Matrix Module (`src/matrix`)

### Matrix Overview

This module manages a 32-button capacitive touch matrix using a single MPR121 sensor. It handles scanning and event dispatching.

### Matrix Scanning and Event Dispatch

- The matrix is logically arranged as 4 rows by 8 columns.
- Each button corresponds to a unique (row, column) pair.
- The matrix is scanned frequently (e.g., every 1ms) by reading the MPR121 sensor's touch bits.
- Button state changes (pressed or released) are detected by comparing current and previous states.
- When a button state changes, an event is dispatched via a user-registered callback.

### Event Handling

- Button press and release events are dispatched via a user-registered event handler callback.
- Events include the button index (0-31) and event type (pressed or released).
- A dedicated rising edge callback can be registered to trigger on the first press of a button (transition from released to pressed).
- The matrix module does not implement application logic; it only provides clean event notifications.

### API Functions

- `Matrix_init(Adafruit_MPR121 *sensor)`: Initialize matrix with MPR121 sensor instance.
- `Matrix_scan()`: Scan matrix and update states; call frequently.
- `Matrix_getButtonState(uint8_t idx)`: Query current state of a button.
- `Matrix_setEventHandler(void (*handler)(const MatrixButtonEvent &))`: Register event handler callback.
- `Matrix_setRisingEdgeHandler(void (*handler)(uint8_t buttonIndex))`: Register rising edge callback.
- `Matrix_printState()`: Print current button states to serial for debugging.

---

## Summary

Pico2DSP2CoreWorks firmware integrates audio synthesis, sequencing, touch input, MIDI, and sensor data into a cohesive digital synthesizer platform. The modular design separates concerns into audio, sequencer, and matrix modules, facilitating maintainability and extensibility. Users can interact with the sequencer via the touch matrix, control notes and velocities with a distance sensor, and visualize the sequence on an OLED display. MIDI and clock synchronization enable integration with external gear.

This documentation should assist users and developers in understanding the firmware architecture, key components, and usage patterns to effectively utilize and extend the software.