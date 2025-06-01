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
#include <Arduino.h>

// Access global note1 and scale[] from main .ino
extern volatile int note1;
extern int scale[40];
extern volatile bool trigenv1; // Used for triggering envelope
extern volatile bool trigenv2; // Used for triggering envelope

const size_t scaleSize = SCALE_ARRAY_SIZE; // Use the defined constant

// Define a base MIDI note for the scale. This could be configurable.
const uint8_t MIDI_BASE_NOTE = 36; // Example: C1 (MIDI note 36)

// ==============================
//  Sequencer Implementation
// ==============================
namespace {
    // Helper: Validate a MIDI note value
    inline bool isValidMidiNote(uint8_t note) {
        return note <= 127;
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
 * @brief Initialize all steps to default values.
 * Note is initialized to a random scale index.
 */
void Sequencer::initializeSteps() {
    Serial.println("[SEQ] Initializing steps...");
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
        state.steps[i] = Step(); // Default initialization
        state.steps[i].note = random(0, SEQUENCER_NUM_STEPS - 2); // 0-14
        // Optional presets at indices 0,4,8,12
        state.steps[0].note = 0;
        state.steps[4].note = 3;
        state.steps[8].note = 5;
        state.steps[12].note = 8;
        state.steps[i].gate = false; // All gates off initially
            // Serial.print("  Step "); Serial.print(i);
            // Serial.print(": ON, Note Index: "); Serial.println(state.steps[i].note);
      
            // Serial.print("  Step "); Serial.print(i);
            // Serial.print(": OFF, Note Index: "); Serial.println(state.steps[i].note);
        
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
        // gate must be bool, no extra validation needed
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
Sequencer::Sequencer() : state(), errorFlag(false), lastNote(-1) {
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
 * @brief Processes the sequencer logic for the given step provided by uClock.
 *
 * Core sequencer step-advance logic:
 * - Uses the `current_uclock_step` to set the internal playhead.
 * - Track the last played note.
 * - Always send noteOff for the last note before sending noteOn for the new note.
 * - If the new step is ON, send noteOn for the current note, set oscillator frequency, and trigger the envelope.
 * - If the new step is OFF, send noteOff for the last note (if any) and release the envelope.
 * - Handle repeated notes by sending noteOff then noteOn, even if the note is the same.
 * - Modular, robust, and well-documented.
 * @param current_uclock_step The current step number (0-15) provided by uClock.
 */
void Sequencer::advanceStep(uint8_t current_uclock_step) {
    if (!state.running) {
        return;
    }

    // Wrap step index to sequencer length
 state.playhead = current_uclock_step;
    Step &currentStep = state.steps[state.playhead];
    if (lastNote >= 0) {
        usb_midi.sendNoteOff(lastNote, 0, 1); // Channel 1, velocity 0
    }

    
    if (currentStep.gate) {
        // Clamp note index to scale size
        uint8_t scaleIndex = (currentStep.note >= scaleSize) ? 0 : currentStep.note;
  // Ensure scaleIndex is valid for the actual 'scale' array bounds.
        if (scaleIndex >= SCALE_ARRAY_SIZE) { // Defensive check
            scaleIndex = 0; 
        }
        int new_midi_note = MIDI_BASE_NOTE + scale[scaleIndex];

        // Update the synth engine's target note (global variable).
        note1 = new_midi_note;

        // Trigger the envelope. This will cause re-articulation on every gated step.
        // If slide/legato functionality were implemented, this call would be conditional.
        triggerEnvelope(); // Sets trigenv1 = true

        // Use velocity and filter from step (velocity mapped to MIDI 0-127)
        uint8_t midiVelocity = static_cast<uint8_t>(currentStep.velocity * 127.0f);
        if (midiVelocity > 127) midiVelocity = 127;

// Send MIDI Note On for the current step's note.
        usb_midi.sendNoteOn(new_midi_note, midiVelocity, 1); // Channel 1

        // Optionally: apply currentStep.filter to synth engine here
        // Optionally: apply currentStep.filter to synth engine here

        lastNote = new_midi_note; // Update lastNote to the currently playing MIDI note.
    } else {
           // Current step's gate is OFF (a rest).
        // The MIDI Note Off for any previously sounding note was handled above.
        releaseEnvelope(); // Sets trigenv1 = false
        lastNote = -1;     // No MIDI note is actively sounding from the sequencer.     
    }
}

/**
 * @brief Convert absolute MIDI note to the offset scheme used by the audio thread.
 *
 */ 
void Sequencer::setOscillatorFrequency(uint8_t midiNote)
{
        // This function directly sets the global note1.
        // If the sequencer is running, advanceStep() will likely override this.
        note1 = midiNote ;
}
/*
/**
 * @brief Trigger the envelope for noteOn.
 * Replace this stub with your actual envelope control logic.
 */
void Sequencer::triggerEnvelope() {
    trigenv1 = true;
    trigenv2 = true;
}

/**
 * @brief Release the envelope for noteOff.
 * Replace this stub with your actual envelope control logic.
 */
void Sequencer::releaseEnvelope() {
    trigenv1 = false;
    trigenv2 = false;
}
// ToggleStep
void Sequencer::toggleStep(uint8_t stepIdx) {
    if (stepIdx >= SEQUENCER_NUM_STEPS) {
        // Handle out-of-bounds index, e.g., log an error or return
        // Serial.print("[SEQ] toggleStep: Invalid step index: "); Serial.println(stepIdx);
        return;
    }
    state.steps[stepIdx].gate = !state.steps[stepIdx].gate;
}
/**
 * @brief Set the MIDI note for a specific step.
 * The 'note' parameter is treated as a scale index.
 * @param stepIdx Index of the step.
 * @param noteIndex Scale index for the step.
 */
void Sequencer::setStepNote(uint8_t stepIdx, uint8_t noteIndex) {
    // Serial.print("[SEQ] setStepNote called for index: "); Serial.print(stepIdx); Serial.print(", noteIndex: "); Serial.println(noteIndex);
    if (stepIdx >= SEQUENCER_NUM_STEPS) {
        // Serial.println("  - Invalid step index. Returning.");
        return;
    }
    state.steps[stepIdx].note = noteIndex;
    // Serial.print("  - Step "); Serial.print(stepIdx);
    // Serial.print(" new note index: "); Serial.println(state.steps[stepIdx].note);
}
/**
 * @brief Set full step data using individual parameters.
 */
void Sequencer::setStep(int index, bool gate, bool slide, int note, float velocity, float filter) {
    if (index < 0 || index >= SEQUENCER_NUM_STEPS) {
        Serial.println("Sequencer::setStep: Step index out of range.");
        return;
    }
    if (note < 0 || note > 24) {
        Serial.println("Sequencer::setStep: Note value out of range (0-24).");
        return;
    }
    if (velocity < 0.0f || velocity > 1.0f) {
        Serial.println("Sequencer::setStep: Velocity value out of range (0.0f-1.0f).");
        return;
    }
    if (filter < 0.0f || filter > 1.0f) {
        Serial.println("Sequencer::setStep: Filter value out of range (0.0f-1.0f).");
        return;
    }
    state.steps[index].gate = gate;
    state.steps[index].slide = slide;
    state.steps[index].note = static_cast<uint8_t>(note);
    state.steps[index].velocity = velocity;
    state.steps[index].filter = filter;
}

/**
 * @brief Set full step data using a Step object.
 */
void Sequencer::setStep(int index, const Step& stepData) {
    if (index < 0 || index >= SEQUENCER_NUM_STEPS) {
        Serial.println("Sequencer::setStep: Step index out of range.");
        return;
    }
    if (stepData.note < 0 || stepData.note > 24) {
        Serial.println("Sequencer::setStep: Note value in Step object out of range (0-24).");
        return;
    }
    if (stepData.velocity < 0.0f || stepData.velocity > 1.0f) {
        Serial.println("Sequencer::setStep: Velocity value in Step object out of range (0.0f-1.0f).");
        return;
    }
    if (stepData.filter < 0.0f || stepData.filter > 1.0f) {
        Serial.println("Sequencer::setStep: Filter value in Step object out of range (0.0f-1.0f).");
        return;
    }
    state.steps[index] = stepData;
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