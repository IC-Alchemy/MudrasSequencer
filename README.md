# Pico2DSP2CoreWorks Sequencer Documentation

## Overview

The Pico2DSP2CoreWorks project includes a modular 16-step sequencer implemented primarily in the `src/sequencer` directory and controlled from the main firmware file `Pico2DSP2CoreWorks.ino`. This sequencer provides step-based sequencing of musical notes with parameters such as gate, slide, velocity, and filter frequency, designed for MIDI and audio synthesis integration.

---

## Sequencer Architecture

- **Step Representation:** Each sequencer step is represented by a `Step` struct containing:
  - `gate` (bool): Whether the step triggers a note.
  - `slide` (bool): Slide/legato flag (reserved for future use).
  - `note` (int): Note index relative to a musical scale.
  - `velocity` (float): Normalized velocity (0.0 to 1.0).
  - `filter` (float): Normalized filter cutoff frequency (0.0 to 1.0).

- **Sequencer State:** The `SequencerState` struct holds:
  - An array of 16 `Step` objects representing the sequence.
  - `playhead` (uint8_t): Current step index (0-15).
  - `running` (bool): Whether the sequencer is actively running.

- **Sequencer Class:** The `Sequencer` class encapsulates all sequencer logic and state, providing:
  - Initialization and reset.
  - Play control (start, stop).
  - Step advance processing.
  - Step parameter editing functions.
  - Query methods for step and playhead state.

---

## Workflow and Control Flow

### Initialization

- The sequencer is initialized via `Sequencer::init()`, which resets all steps to default states, resets the playhead, and validates internal state integrity.

### Step Advancement

- The core sequencing occurs in `Sequencer::advanceStep(current_uclock_step, mm, recordButtonHeld)`, called regularly with the current step index from the external clock (`uClock`).
- At each step advance:
  - The playhead index is updated to `current_uclock_step`.
  - The current step is fetched from the state array.
  - If the sequencer is running and the current step's gate is high:
    - The note is translated from the step's note index using a scale lookup to a MIDI note number.
    - The note is sent to the synth engine and MIDI output.
    - The envelope is triggered to articulate the note.
    - Velocity and filter parameters are applied.
  - If the gate is low, the envelope is released (note off).
  - If no step is selected for manual editing (`selectedStepForEdit == -1`) and the current step's gate is high, the current distance sensor reading (`mm`) is mapped and written automatically to the step's note, velocity, and filter parameters, allowing real-time sensor-driven modulation.

### Manual Step Editing

- Manual editing is controlled externally in `Pico2DSP2CoreWorks.ino`.
- When a step is selected (`selectedStepForEdit != -1`), the user can adjust parameters (note, velocity, filter) via button holds.
- The sensor reading (`mm`) is mapped to these parameters and written directly to the selected step.
- This editing overrides the automatic sensor-driven writing.

---

## Input and Output Processing

- **Inputs:**
  - `current_uclock_step`: The current step index driven by an external clock source.
  - `mm`: Distance sensor reading (millimeters) used for dynamic parameter modulation.
  - User inputs (buttons and step selection) for manual parameter editing.

- **Outputs:**
  - MIDI note messages sent via USB MIDI.
  - Synth engine frequency, velocity, and filter parameters updated.
  - Gate and envelope signals controlling articulation.

---

## Timing and Control Mechanisms

- The sequencer is synchronized to an external clock (`uClock`) which drives the step advancement.
- Each step is processed once per clock cycle.
- The playhead moves sequentially through the 16-step sequence, wrapping around.
- The running state controls whether the sequencer processes steps or remains idle.
- Manual editing state (`selectedStepForEdit`) gates whether automatic sensor modulation occurs.

---

## Interaction with Other Components

- **Distance Sensor:** Provides real-time data (`mm`) used to modulate step parameters automatically or during manual editing.
- **User Interface:** Step selection and button holds control manual editing of step parameters.
- **MIDI Interface:** Sends MIDI note on/off messages corresponding to the sequencer steps.
- **Synth Engine:** Receives frequency, velocity, and filter updates based on sequencer step parameters.

---

## Example Sequence Processing Flow

1. External clock signals step advance to step 0.
2. `advanceStep(0, mm, recordButtonHeld)` is called.
3. Playhead updates to step 0; current step fetched.
4. If running and gate is high:
   - Map step note to MIDI note.
   - Trigger envelope and send MIDI note on.
   - Apply velocity and filter.
5. If no step is selected for editing, map current `mm` sensor value and write to this step.
6. On subsequent steps, repeat above with updated playhead index.

---

## Diagram (Simplified Data Flow)

```
External Clock (uClock)
        |
        v
  Sequencer::advanceStep()
        |
        v
  Playhead & Step State
        |
        +----------------+
        |                |
        v                v
MIDI Out           Synth Engine
(note on/off, freq, velocity, filter)
        |
        v
Distance Sensor (mm) ---> Sequencer Step Parameter Auto-Write
        |
User Interface (step select, buttons) ---> Manual Editing Overrides
```

---

This documentation summarizes the sequencer's architecture, workflow, and integration points, providing a complete picture of how input signals and user interaction drive musical output.

For detailed API usage, refer to the header files in `src/sequencer`.