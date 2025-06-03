/**
 * @file SequencerDefs.h
 * @brief Data structures and constants for the modular step sequencer.
 *
 * Defines step state, note, gate, and playhead types for use in the Sequencer
 * module.
 *
 * Example:
 *   #include "SequencerDefs.h"
 *   // Create default Step
 *   Step defaultStep;
 *   // defaultStep.gate==false, slide==false, note==0, velocity==0.0, filter==0.0
 *
 *   // Create and configure a custom Step
 *   Step customStep(true, true, 7, 0.9f, 0.3f);
 *
 *   // Use in SequencerState
 *   SequencerState state;
 *   state.steps[0] = customStep;
 *   state.running = true;
 */

#ifndef SEQUENCER_DEFS_H
#define SEQUENCER_DEFS_H

#include <stdint.h>

// Number of steps per sequencer (fixed at 16 for this project)
constexpr uint8_t SEQUENCER_NUM_STEPS = 16;

// Size of the global 'scale' array defined in the main .ino file
constexpr uint8_t SCALE_ARRAY_SIZE = 40;

// Represents a single step in the sequencer
struct Step {
 bool gate = false;      // Gate ON (true) or OFF (false)
  bool slide = false;     // Slide ON (true) or OFF (false)
  int note = 0;           // Note value, 0-24
  float velocity = 0.5f;  // Velocity, 0.0f - 1.0f (normalized)
  float filter = 0.5f;    // Filter value, 0.0f - 1.0f (normalized)

  // Default constructor initializes to sensible defaults
  Step() = default;

  // Parameterized constructor for convenience
  Step(bool g, bool s, int n, float v, float f)
      : gate(g), slide(s), note(n), velocity(v), filter(f) {}
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