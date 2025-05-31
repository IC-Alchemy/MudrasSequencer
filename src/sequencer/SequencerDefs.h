/**
 * @file SequencerDefs.h
 * @brief Data structures and constants for the modular step sequencer.
 *
 * Defines step state, note, gate, and playhead types for use in the Sequencer
 * module.
 *
 * Usage:
 *   #include "SequencerDefs.h"
 *   // Used internally by Sequencer class and for integration with
 * matrix/output modules.
 */

#ifndef SEQUENCER_DEFS_H
#define SEQUENCER_DEFS_H

#include <stdint.h>

// Number of steps per sequencer (fixed at 16 for this project)
constexpr uint8_t SEQUENCER_NUM_STEPS = 16;

// Step state: ON = step will trigger, OFF = step is skipped
enum class StepState : uint8_t { OFF = 0, ON = 1 };

// Represents a single step in the sequencer
struct Step {
  StepState state; // ON/OFF
  uint8_t note;    // Scale index (e.g., 0-14) for note selection from 'scale[]' array.
  bool gate;       // Gate output state (true = gate on)
  Step()
      : state(StepState::ON), note(0), gate(true) {
  } // Default: OFF, first note in scale (index 0), gate off
};

// Playhead position (0..SEQUENCER_NUM_STEPS-1)
using Playhead = uint8_t;

// Sequencer state (for future extensibility)
struct SequencerState {
  Step steps[SEQUENCER_NUM_STEPS];
  Playhead playhead; // Current step index
  bool running;      // Is the sequencer running?
  SequencerState() : playhead(0), running(false) {}
};

#endif // SEQUENCER_DEFS_H