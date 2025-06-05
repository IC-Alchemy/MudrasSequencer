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
#include <cstdint>
#include <stdlib.h> // for random()

// g_synthVoiceState is now directly used, declared as extern in Sequencer.h
// scale data is now passed via constructor and stored in scale_data_

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
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
        state.steps[i] = Step(); // Default initialization
        state.steps[i].note = 0;
        state.steps[i].filter = 600.5f; // Default filter value as float (matches SequencerDefs.h change)

        state.steps[i].gate = true; // All gates off initially
        
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
 * @brief Constructor. Initializes sequencer state and stores MIDI interface and scale data.
 */
Sequencer::Sequencer(midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>>& midi_interface, ScaleArrayRefType scale_data)
    : midi_interface_(midi_interface), scale_data_(scale_data), state(), errorFlag(false), lastNote(-1) {
    // Default member initializers handle state, errorFlag, and lastNote.
    // Step struct defaults (via SequencerDefs.h) handle initial step values upon state construction.
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

         midi_interface_.sendNoteOff(lastNote, 0, 1); // Channel 1, velocity 0
  
  }

    
    if (currentStep.gate) {
        // Clamp note index to scale size
    
  uint8_t scaleIndex = (currentStep.note >= scaleSize) ? 0 : currentStep.note;
  // Ensure scaleIndex is valid for the actual 'scale' array bounds.
        if (scaleIndex >= SCALE_ARRAY_SIZE) { // Defensive check
            scaleIndex = 0; 
        }
        int new_midi_note = MIDI_BASE_NOTE + scale_data_[0][scaleIndex];

        // Update the synth engine's target state via g_synthVoiceState
        g_synthVoiceState.note = new_midi_note;
        g_synthVoiceState.velocity = currentStep.velocity;
        // Map normalized filter value (0.0-1.0) from step to Hz for synth state.
        g_synthVoiceState.filterCutoff = currentStep.filter * 5000.0f;

        // Trigger the envelope for the current step.
        triggerEnvelope(); // Sets g_synthVoiceState.gateOn = true

        // Send MIDI Note On for the current step's note.
        uint8_t midiVelocity = static_cast<uint8_t>(g_synthVoiceState.velocity * 127.0f);
        if (midiVelocity > 127) midiVelocity = 127;
        midi_interface_.sendNoteOn(new_midi_note, midiVelocity, 1); // Channel 1

        lastNote = new_midi_note; // Update lastNote to the currently playing MIDI note.
    } else {
           // Current step's gate is OFF (a rest).
        // The MIDI Note Off for any previously sounding note was handled above.
        releaseEnvelope(); // Sets g_synthVoiceState.gateOn = false
        lastNote = -1;     // No MIDI note is actively sounding from the sequencer.     
    }
}
/**
 * @brief Instantly play a step for real-time feedback (does not advance playhead).
 */
void Sequencer::playStepNow(uint8_t stepIdx) {
    if (stepIdx >= SEQUENCER_NUM_STEPS) return;
    Step &currentStep = state.steps[stepIdx];

    // Clamp note index to scale size
    uint8_t scaleIndex = (currentStep.note >= scaleSize) ? 0 : currentStep.note;
    if (scaleIndex >= SCALE_ARRAY_SIZE) scaleIndex = 0;
    int new_midi_note = MIDI_BASE_NOTE + scale_data_[0][scaleIndex];

    // Update the synth engine's target state via g_synthVoiceState
    g_synthVoiceState.note = new_midi_note;
    g_synthVoiceState.velocity = currentStep.velocity;
    // Map normalized filter value (0.0-1.0) from step to Hz for synth state.
    g_synthVoiceState.filterCutoff = currentStep.filter * 5000.0f;

    // Trigger the envelope for instant audio feedback.
    triggerEnvelope(); // Sets g_synthVoiceState.gateOn = true
}

/**
 * @brief Convert absolute MIDI note to the offset scheme used by the audio thread.
 *
 */ 
void Sequencer::setOscillatorFrequency(uint8_t midiNote)
{
        // Directly set the note for the synth voice.
        // Useful for external control or immediate note changes outside of sequence playback.
        // If the sequencer is running, advanceStep() will override this on the next step.
        g_synthVoiceState.note = midiNote;
}
/*
/**
 * @brief Activates the synth envelope (sets gateOn to true).
 */
void Sequencer::triggerEnvelope() {
    g_synthVoiceState.gateOn = true;
}

/**
 * @brief Deactivates the synth envelope (sets gateOn to false).
 */
void Sequencer::releaseEnvelope() {
    g_synthVoiceState.gateOn = false;
}

// ToggleStep
void Sequencer::toggleStep(uint8_t stepIdx) {
    if (stepIdx >= SEQUENCER_NUM_STEPS) {
        // Handle out-of-bounds index, e.g., log an error or return
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
   
    if (stepIdx >= SEQUENCER_NUM_STEPS) {
        return;
    }
    state.steps[stepIdx].note = noteIndex;
}

void Sequencer::setStepVelocity(uint8_t stepIdx, float velocity) { 
    // velocity is float 0.0f - 1.0f
    if (stepIdx >= SEQUENCER_NUM_STEPS) {
        return;
    }
  
    state.steps[stepIdx].velocity = velocity;
}


void Sequencer::setStepFiltFreq(uint8_t stepIdx, float filter) {
 
    if (stepIdx >= SEQUENCER_NUM_STEPS) {
        return;
    }
    state.steps[stepIdx].filter = filter; // filter is normalized float 0.0f - 1.0f
}




/**
 * @brief Set full step data using individual parameters.
 */
void Sequencer::setStep(int index, bool gate, bool slide, int note, float velocity, float filter) {
    if (index < 0 || index >= SEQUENCER_NUM_STEPS) {
        return;
    }
    if (note < 0 || note > 24) {
        return;
    }
    if (velocity < 0.0f || velocity > 1.0f) {
        return;
    }
    if (filter < 0.0f || filter > 1.0f) {
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
        return;
    }
    if (stepData.note < 0 || stepData.note > 24) {
        return;
    }
    if (stepData.velocity < 0.0f || stepData.velocity > 1.0f) {
        return;
    }
    if (stepData.filter < 0.0f || stepData.filter > 1.0f) {
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

