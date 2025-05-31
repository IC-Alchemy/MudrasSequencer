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
extern volatile bool trigenv1; // Used for triggering envelope

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
 * @brief Initialize all steps to default values.
 * Note is initialized to a random scale index.
 */
void Sequencer::initializeSteps() {
    Serial.println("[SEQ] Initializing steps...");
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
        state.steps[i] = Step(); // Step is now OFF by default from constructor
        state.steps[i].note = random(0, 15); // Assign a random scale index (0-14)

        // Randomly turn ON some steps and set their gate to true
        if (random(0, 2) == 0) {
            state.steps[i].state = StepState::ON;
            state.steps[i].gate = true; // If step is ON, its gate should be true
            // Serial.print("  Step "); Serial.print(i);
            // Serial.print(": ON, Note Index: "); Serial.println(state.steps[i].note);
        } else {
            // Serial.print("  Step "); Serial.print(i);
            // Serial.print(": OFF, Note Index: "); Serial.println(state.steps[i].note);
        }
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
        // s.note is now a scale index. Validation of index range should ideally consider
        // the actual size of the 'scale' array. For now, we rely on initialization
        // and setStepNote to manage this. We only check StepState here.
        if (!isValidStepState(s.state)) {
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
    Serial.print("[SEQ] advanceStep called with uClock step: "); Serial.println(current_uclock_step);
    if (!state.running) {
        // Serial.println("  - Sequencer not running. Returning."); // Keep this one for now if needed
        return;
    }

    // Validate the incoming step from uClock
    if (current_uclock_step >= SEQUENCER_NUM_STEPS) {
        // Serial.print("  - WARN: uClock step out of bounds: "); Serial.print(current_uclock_step);
        current_uclock_step = 0; // Or handle error more gracefully
        // Serial.print(", corrected to: "); Serial.println(current_uclock_step);
    }

    // --- Previous Note Handling ---
    // Send Note Off for the previously sounding MIDI note.
    if (lastNote >= 0) {
        // Serial.print("  - Sending NoteOff for previous note: "); Serial.println(lastNote);
        usb_midi.sendNoteOff(lastNote, 0, 1); // Channel 1, velocity 0
    }

    // --- Set Playhead based on uClock's current step ---
    // The playhead is now directly set by the step provided by uClock.
    state.playhead = current_uclock_step;
    Step &currentStep = state.steps[state.playhead]; // Get the step uClock says is current

    // Serial.print("  - Playhead set to: "); Serial.print(state.playhead);
    // Serial.print(" - Step State: "); Serial.print(currentStep.state == StepState::ON ? "ON" : "OFF");
    // Serial.print(", Step Gate: "); Serial.println(currentStep.gate ? "TRUE" : "FALSE");

    // --- Current Step Processing ---
    if (currentStep.state == StepState::ON) {
        // Serial.println("    - Step IS ON. Processing note.");
        currentStep.gate = true; // Gate is conceptually high for an ON step

        // Calculate the actual MIDI note for this step
        // currentStep.note is a scale index (e.g., 0-14)
        uint8_t scaleIndex = currentStep.note;

        // Validate scaleIndex. Since initializeSteps uses random(0,15),
        // scaleIndex will be 0-14. A check against the actual size of `scale[]`
        // or a defined MAX_SCALE_INDEX would be more robust if setStepNote could set higher values.
        // Serial.print("      - Scale Index from step.note: "); Serial.println(scaleIndex);
        if (scaleIndex >= 15) { // Max index from random(0,15) is 14. Safeguard. Should match scale array bounds.
            // Serial.print("      - WARN: scaleIndex out of expected range (0-14), resetting to 0. Was: "); Serial.println(scaleIndex);
            scaleIndex = 0; // Default to a safe index if somehow out of expected range.
        }

        // Ensure scaleIndex is within bounds of the actual scale array.
        // sizeof(scale)/sizeof(scale[0]) gives number of elements. Max index is size-1.
        const size_t scaleSize = SCALE_ARRAY_SIZE; // Use the defined constant
        if (scaleIndex >= scaleSize) {
            // Serial.print("      - WARN: scaleIndex ("); Serial.print(scaleIndex);
            // Serial.print(") is out of bounds for scale[] array (size: "); Serial.print(scaleSize);
            // Serial.println("). Resetting to 0.");
            scaleIndex = 0;
        }
        // Serial.print("      - Value from scale["); Serial.print(scaleIndex); Serial.print("]: "); Serial.println(scale[scaleIndex]);

        // Assuming scale[] contains offsets from MIDI_BASE_NOTE
        uint8_t actualMidiNote = MIDI_BASE_NOTE + scale[scaleIndex];
        // Serial.print("      - Calculated MIDI Note (base + offset): "); Serial.println(actualMidiNote);

        // Send MIDI Note On
        usb_midi.sendNoteOn(actualMidiNote, 100, 1); // Velocity 100, Channel 1
        // Serial.println("      - MIDI NoteOn SENT.");

        // Update global synth parameter (note1) with the absolute MIDI note
        note1 = actualMidiNote;
        // Serial.print("      - Global note1 set to: "); Serial.println(note1);

        triggerEnvelope();
        // Serial.println("      - Envelope TRIGGERED.");

        // Store the actual MIDI note that was just turned on
        lastNote = actualMidiNote;
    } else { // Current step is OFF
        // Serial.println("    - Step IS OFF. Releasing envelope.");
        currentStep.gate = false; // Gate is low
        releaseEnvelope();
        // No new note is played, lastNote remains from previous NoteOff or is -1
        lastNote = -1; // Explicitly no MIDI note is sounding from the sequencer
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

/**
 * @brief Toggle the ON/OFF state of a step.
 * @param stepIdx Index of the step to toggle.
 */
void Sequencer::toggleStep(uint8_t stepIdx) {
    // Serial.print("[SEQ] toggleStep called for index: "); Serial.println(stepIdx);
    if (stepIdx >= SEQUENCER_NUM_STEPS) {
        // Serial.println("  - Invalid step index. Returning.");
        return;
    }
    Step &s = state.steps[stepIdx];
    s.state = (s.state == StepState::ON) ? StepState::OFF : StepState::ON;
    s.gate = (s.state == StepState::ON); // Update gate to match the new state
    // Serial.print("  - Step "); Serial.print(stepIdx);
    // Serial.print(" new state: "); Serial.print(s.state == StepState::ON ? "ON" : "OFF");
    // Serial.print(", new gate: "); Serial.println(s.gate ? "TRUE" : "FALSE");
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
    // Add validation for noteIndex if MAX_SCALE_INDEX is known
    // For now, assume noteIndex is valid (e.g., 0-14 if that's the intended range for scale[])
    state.steps[stepIdx].note = noteIndex;
    // Serial.print("  - Step "); Serial.print(stepIdx);
    // Serial.print(" new note index: "); Serial.println(state.steps[stepIdx].note);
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