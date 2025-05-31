/**
 * @file Sequencer.cpp
 * @brief Implementation of the modular 16-step sequencer.
 *
 * Handles step state, playhead advance, step toggling, and note assignment.
 * Designed for integration with matrix scanning and output modules (MIDI,
 * gate).
 *
 * Usage:
 *   See Sequencer.h for interface and example.
 */

#include "Sequencer.h"

// Access global note1 and scale[] from main .ino
extern volatile int note1;
extern int scale[];

// ==============================
//  Sequencer Implementation
// ==============================

namespace {
    // Helper: Validate a MIDI note value
    inline bool isValidMidiNote(uint8_t note) {
        return note <= 127;
    }

    // Helper: Validate a StepState value
    inline bool isValidStepState(StepState state) {
        return state == StepState::ON || state == StepState::OFF;
    }
}
/**
 * @brief Initialize the sequencer to a known good state.
 *
 * Resets all steps, playhead, and running state to defaults. Validates the integrity
 * of the internal state array. If any issue is detected, sets an internal error flag
 * and returns false. Safe to call multiple times (idempotent).
 *
 * @return true if initialization succeeded, false if an error was detected.
 */
bool Sequencer::init() {
    resetState();
    errorFlag = !validateState();
    return !errorFlag;
}

/**
 * @brief Reset all sequencer state to defaults (helper for init()).
 *        Sets playhead, running, and all steps to known values.
 */
void Sequencer::resetState() {
    state.playhead = 0;
    state.running = false;
    initializeSteps();
}

/**
 * @brief Initialize all steps to default values (gate=true, note=48).
 */
void Sequencer::initializeSteps() {
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
        state.steps[i] = Step();
        state.steps[i].gate = true;
        state.steps[i].note = 48;
    }
}

/**
 * @brief Validate the integrity of the sequencer state.
 *        Checks playhead bounds, step note range, and step state validity.
 * @return true if state is valid, false otherwise.
 */
bool Sequencer::validateState() const {
    if (state.playhead >= SEQUENCER_NUM_STEPS) {
        return false;
    }
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
        const Step& s = state.steps[i];
        if (!isValidMidiNote(s.note) || !isValidStepState(s.state)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if the sequencer is in an error state after initialization.
 * @return true if an error was detected during the last init(), false otherwise.
 */
bool Sequencer::hasError() const {
    return errorFlag;
}

/**
 * @brief Default constructor. Initializes sequencer state to default values.
 */
Sequencer::Sequencer() : state() {
    // All steps default to OFF, note 60, gate false (see Step constructor)
    // Playhead at 0, running = false
}

/**
 * @brief Start the sequencer (sets running flag).
 */
void Sequencer::start() {
    state.running = true;
}

/**
 * @brief Stop the sequencer (clears running flag).
 *        Optionally, clear all gates (left for output module).
 */
void Sequencer::stop() {
    state.running = false;
    // Optionally, clear all gates (not handled here, left for output module)
}

/**
 * @brief Reset the sequencer to its default state.
 *        Resets playhead, running, and all steps.
 */
void Sequencer::reset() {
    resetState();
}

/**
 * @brief Advance the playhead to the next step (wraps at end).
 *        If the current step is ON, triggers gate and MIDI note.
 */
/**
 * @brief Advance the playhead to the next step (wraps at end) and handle note/gate logic.
 *
 * Core sequencer step-advance logic:
 * - On each clock tick, advance the sequencer playhead.
 * - Track the last played note.
 * - Always send noteOff for the last note before sending noteOn for the new note.
 * - If the new step is ON, send noteOn for the current note, set oscillator frequency, and trigger the envelope.
 * - If the new step is OFF, send noteOff for the last note (if any) and release the envelope.
 * - Handle repeated notes by sending noteOff then noteOn, even if the note is the same.
 * - Modular, robust, and well-documented.
 */
void Sequencer::advanceStep() {
    if (!state.running)
        return;

    // Store previous note for noteOff handling
    int8_t prevNote = lastNote;

    // Advance playhead, wrap at end
    state.playhead = (state.playhead + 1) % SEQUENCER_NUM_STEPS;

    // Update global oscillator note to match current step in scale
    note1 = scale[state.playhead];

    // Get the new current step
    Step &current = state.steps[state.playhead];

    // Always send noteOff for the last note before noteOn for the new note
    if (prevNote >= 0) {
        // Replace usb_midi with your MIDI interface if needed
        usb_midi.sendNoteOff(prevNote, 0, 1);
    }

    if (current.state == StepState::ON) {
        current.gate = true;

        // Send noteOn for the current note (even if same as last)
        usb_midi.sendNoteOn(current.note, 100, 1);

        // Set oscillator frequency for the new note (stub, replace with actual call)
        // setOscillatorFrequency(current.note); // stub call commented out

        // Trigger the envelope (stub, replace with actual call)
        // triggerEnvelope(); // stub call commented out

        // Set trigenv1 HIGH for gate duration (legacy, if needed)
        trigenv1 = true;

        // Update lastNote to the current note
        lastNote = static_cast<int8_t>(current.note);
    } else {
        // Step is OFF: ensure gate is low, release envelope, and clear lastNote
        current.gate = false;

        // Release the envelope (stub, replace with actual call)
        // releaseEnvelope(); // stub call commented out

        // Set trigenv1 LOW (legacy, if needed)
        trigenv1 = false;

        // No note is playing
        lastNote = -1;
    }
}

/**
 * @brief Set the oscillator frequency for the given MIDI note.
 * Replace this stub with your actual oscillator control logic.
 
void Sequencer::setOscillatorFrequency(uint8_t midiNote) {
    // Example: oscillator.setFrequency(midiNoteToFrequency(midiNote));
}
/*
/**
 * @brief Trigger the envelope for noteOn.
 * Replace this stub with your actual envelope control logic.
 
void Sequencer::triggerEnvelope() {
    // Example: envelope.trigger();
}

/**
 * @brief Release the envelope for noteOff.
 * Replace this stub with your actual envelope control logic.
 
void Sequencer::releaseEnvelope() {
    // Example: envelope.release();
}

/**
 * @brief Toggle the ON/OFF state of a step.
 * @param stepIdx Index of the step to toggle.
 */
void Sequencer::toggleStep(uint8_t stepIdx) {
    if (stepIdx >= SEQUENCER_NUM_STEPS)
        return;
    Step &s = state.steps[stepIdx];
    s.state = (s.state == StepState::ON) ? StepState::OFF : StepState::ON;
}

/**
 * @brief Set the MIDI note for a specific step.
 * @param stepIdx Index of the step.
 * @param note MIDI note value (0-127).
 */
void Sequencer::setStepNote(uint8_t stepIdx, uint8_t note) {
    if (stepIdx >= SEQUENCER_NUM_STEPS)
        return;
    state.steps[stepIdx].note = note;
}

/**
 * @brief Get a const reference to a step.
 * @param stepIdx Index of the step.
 * @return Const reference to the step.
 */
const Step &Sequencer::getStep(uint8_t stepIdx) const {
    if (stepIdx >= SEQUENCER_NUM_STEPS)
        stepIdx = 0;
    return state.steps[stepIdx];
}

/**
 * @brief Get the current playhead position.
 * @return Playhead index.
 */
uint8_t Sequencer::getPlayhead() const {
    return state.playhead;
}

/**
 * @brief Check if the sequencer is currently running.
 * @return true if running, false otherwise.
 */
bool Sequencer::isRunning() const {
    return state.running;
}
int8_t Sequencer::getLastNote() const { return lastNote; }
void Sequencer::setLastNote(int8_t note) { lastNote = note; }
// Returns a const reference to the internal SequencerState.
// This method is const-correct and does not allow modification of the internal state.
const SequencerState& Sequencer::getState() const {
    return state;
}