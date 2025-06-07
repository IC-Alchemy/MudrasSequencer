/*
 * Sequencer.cpp
 * Core logic for 16-step modular sequencer.
 * - Manages step states, playhead, note assignment.
 * - Integrates with MIDI and gate output modules.
 * - Designed for embedded DSP synth environments.
 */

#include "Sequencer.h"
#include <Arduino.h>
#include <cstdint>
#include <stdlib.h> // for random()
// Access global note1 and scale[] from main .ino
extern volatile int note1;
extern volatile float freq1;

extern int currentScale; // Newly added scale index variable

// ParameterMapping registry for distance sensor to step parameter mapping
static ParameterMapping parameterMappings[] = {
    {
        // Note mapping (button16Held)
        []() { return button16Held; },
        [](Step& step, int mm) {
            int mmNote = map(mm, 0, 1400, 0, 24);
            if (mmNote < 0) mmNote = 0;
            if (mmNote > 24) mmNote = 0;
            step.note = mmNote;
        }
    },
    {
        // Velocity mapping (button17Held)
        []() { return button17Held; },
        [](Step& step, int mm) {
            int mmVelocity = map(mm, 0, 1400, 0, 127);
            if (mmVelocity < 0) mmVelocity = 0;
            if (mmVelocity > 127) mmVelocity = 127;
            step.velocity = mmVelocity / 127.0f;
        }
    },
    {
        // Filter mapping (button18Held)
        []() { return button18Held; },
        [](Step& step, int mm) {
            int mmFiltFreq = map(mm, 0, 1400, 0, 4000);
            if (mmFiltFreq < 0) mmFiltFreq = 0;
            if (mmFiltFreq > 5000) mmFiltFreq = 4000;
            step.filter = mmFiltFreq;
        }
    }
    // Add more mappings here as needed
};

const size_t numParameterMappings = sizeof(parameterMappings) / sizeof(ParameterMapping);


// Access step editing state and distance sensor reading from main .ino
extern int selectedStepForEdit;
extern int mm;
extern volatile float vel1;
extern int scale[5][48]; 
     extern volatile bool button16Held, button17Held, button18Held;


extern volatile bool trigenv1; // Used for triggering envelope
extern volatile bool trigenv2; // Used for triggering envelope

const size_t scaleSize = SCALE_ARRAY_SIZE; // Use the defined constant

// Define a base MIDI note for the scale. This could be configurable.
const uint8_t MIDI_BASE_NOTE = 36; // Example: C1 (MIDI note 36)

// ==============================
//  Sequencer Implementation
// ==============================


/**
 * @brief Initialize the sequencer to a known good state.
 *
 * Resets all steps, playhead, and running state to defaults. Validates the integrity
 * of the internal state array. If any issue is detected, sets an internal error flag
 * and returns false. Safe to call multiple times (idempotent).
 *
 * @return true if initialization succeeded, false if an error was detected.
 */
void Sequencer::init() {
    state.playhead = 0;
    state.running = false;
    initializeSteps(); 
}

void Sequencer::initializeSteps() {
    // Serial output removed due to missing Serial definition
    for (uint8_t i = 0; i < stepLength; ++i) {
        state.steps[i] = Step(); // Default initialization
        state.steps[i].note = 0;
        state.steps[i].gate = true; // All gates ON
        state.steps[i].velocity = 100.0f / 127.0f; // Velocity at 100 (MIDI scale)
        state.steps[i].filter = random(200,4444); // Filter freq at 2000 Hz (normalized)
 
    }
    // Clear any unused steps
    for (uint8_t i = stepLength; i < SEQUENCER_NUM_STEPS; ++i) {
        state.steps[i] = Step();
        state.steps[i].gate = false;
    }
    
}



/**
 * @brief Default constructor. Initializes sequencer state to default values.
 */
Sequencer::Sequencer() : state(), errorFlag(false), lastNote(-1) {

}

/**
 * @brief Start the sequencer (sets running flag).
 */
void Sequencer::start() {
    state.running = true;
}

/**
 * @brief Stop the sequencer (clears running flag).
 *      
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

 * @param current_uclock_step The current step number (0-15) provided by uClock.
 */
void Sequencer::advanceStep(uint8_t current_uclock_step, int mm_distance,
                   bool is_button16_held, bool is_button17_held, bool is_button18_held,
                   int current_selected_step_for_edit,
                   VoiceState* voiceState) {
  // Always send NoteOff for the last note before starting a new one (monophonic)
    // But delay if slide is active to allow legato/glide
    if (currentNote >= 0 && !state.steps[state.playhead].slide) {
        handleNoteOff();
    }
    // Wrap step index to sequencer length
 // Wrap step index to stepLength
 state.playhead = current_uclock_step % stepLength;
 Step &currentStep = state.steps[state.playhead];

 // --- Auto-write distance sensor to step if no step is selected for edit and gate is high ---
 if (selectedStepForEdit == -1 && currentStep.gate) {
     // Iterate over all parameter mappings and apply those whose button is active
     for (size_t i = 0; i < numParameterMappings; ++i) {
         if (parameterMappings[i].isActive()) {
             parameterMappings[i].apply(currentStep, mm);
         }
     }
 }

   // Update per-voice state if provided
   if (voiceState) {
       voiceState->note = currentStep.note;
       voiceState->velocity = currentStep.velocity;
       voiceState->filter = currentStep.filter;
       voiceState->gate = currentStep.gate;
       voiceState->slide = currentStep.slide;
   }

   if (currentStep.gate) {
       // Clamp note index to scale size
       uint8_t scaleIndex = (currentStep.note >= scaleSize) ? 0 : currentStep.note;
       if (scaleIndex >= SCALE_ARRAY_SIZE) { // Defensive check
           scaleIndex = 0;
       }
       int new_midi_note = MIDI_BASE_NOTE + scale[currentScale][scaleIndex];

       // Update the synth engine's target note (global variable).
       note1 = new_midi_note;

       // Trigger the envelope. This will cause re-articulation on every gated step.

       // Use velocity and filter from step (velocity mapped to MIDI 0-127)
       vel1 = currentStep.velocity;
       freq1 = currentStep.filter * 1.f; // Map filter 0.0-1.0 to 0-5000 Hz (adjust as needed)

       // Start the note with a fixed duration (e.g., 24 ticks for a 16th note at 96 PPQN)
       startNote(new_midi_note, vel1*127, 24);

       lastNote = new_midi_note; // Update lastNote to the currently playing MIDI note.
   } else {
       // Current step's gate is OFF (a rest).
       handleNoteOff();
       lastNote = -1;     // No MIDI note is actively sounding from the sequencer.
   }
}
/**
 * @brief Instantly play a step for real-time feedback (does not advance playhead).
 */
void Sequencer::playStepNow(uint8_t stepIdx) {
    if (stepIdx >= stepLength) return;
    Step &currentStep = state.steps[stepIdx];

    // Clamp note index to scale size
    uint8_t scaleIndex = (currentStep.note >= scaleSize) ? 0 : currentStep.note;
    if (scaleIndex >= SCALE_ARRAY_SIZE) scaleIndex = 0;
    int new_midi_note = MIDI_BASE_NOTE + scale[0][scaleIndex];

    // Update the synth engine's target note (global variable).
    note1 = new_midi_note;

    // Optionally set velocity and filter globals if needed
    // (Assuming Velocity and FiltFreq are global variables used in audio rendering)
    extern volatile uint8_t Velocity;
    extern volatile float FiltFreq;
    Velocity = static_cast<uint8_t>(currentStep.velocity * 127.0f);
    FiltFreq = currentStep.filter * 5000.0f; // Map filter 0.0-1.0 to 0-5000 Hz (adjust as needed)

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
    if (stepIdx >= stepLength) {
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
    if (stepIdx >= stepLength) {
        // Serial.println("  - Invalid step index. Returning.");
        return;
    }
    state.steps[stepIdx].note = noteIndex;
    // Serial.print("  - Step "); Serial.print(stepIdx);
    // Serial.print(" new note index: "); Serial.println(state.steps[stepIdx].note);
}

void Sequencer::setStepVelocity(uint8_t stepIdx, uint8_t velocityByte) { // velocityByte is 0-127
    if (stepIdx >= stepLength) {
        return;
    }
    // Convert 0-127 byte to 0.0f-1.0f float
    state.steps[stepIdx].velocity = static_cast<float>(velocityByte) / 127.0f;
}
void Sequencer::setStepFiltFreq(uint8_t stepIdx, float filter) {
 
    if (stepIdx >= stepLength) {
        // Serial.println("  - Invalid step index. Returning.");
        return;
    }
    state.steps[stepIdx].filter = filter;
    // Serial.print("  - Step "); Serial.print(stepIdx);
    // Serial.print(" new note index: "); Serial.println(state.steps[stepIdx].note);
}
/**
 * @brief Set full step data using individual parameters.
 */
void Sequencer::setStep(int index, bool gate, bool slide, int note, float velocity, float filter) {
    if (index < 0 || index >= stepLength) {
        // Serial.println("Sequencer::setStep: Step index out of range.");
        return;
    }
    if (note < 0 || note > 24) {
        // Serial.println("Sequencer::setStep: Note value out of range (0-24).");
        return;
    }
    if (velocity < 0.0f || velocity > 1.0f) {
        // Serial.println("Sequencer::setStep: Velocity value out of range (0.0f-1.0f).");
        return;
    }
    if (filter < 0.0f || filter > 1.0f) {
        // Serial.println("Sequencer::setStep: Filter value out of range (0.0f-1.0f).");
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
    if (index < 0 || index >= stepLength) {
        // Serial.println("Sequencer::setStep: Step index out of range.");
        return;
    }
    if (stepData.note < 0 || stepData.note > 24) {
        // Serial.println("Sequencer::setStep: Note value in Step object out of range (0-24).");
        return;
    }
    if (stepData.velocity < 0.0f || stepData.velocity > 1.0f) {
        // Serial.println("Sequencer::setStep: Velocity value in Step object out of range (0.0f-1.0f).");
        return;
    }
    if (stepData.filter < 0.0f || stepData.filter > 1.0f) {
        // Serial.println("Sequencer::setStep: Filter value in Step object out of range (0.0f-1.0f).");
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
    if (stepIdx >= stepLength)
        stepIdx = 0;
    return state.steps[stepIdx];
}

/**
 * @brief Get the current playhead position.
 * @return Playhead index.
void Sequencer::debugPrintStepGates() const {
#ifdef ARDUINO
    Serial.print("Step Gates: ");
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
        Serial.print("[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(state.steps[i].gate ? "ON" : "OFF");
        if (i < SEQUENCER_NUM_STEPS - 1) Serial.print(", ");
    }
    Serial.println();
#endif
}
 */
void Sequencer::debugPrintStepGates() const {
    Serial.print("Step Gates: ");
    for (uint8_t i = 0; i < SEQUENCER_NUM_STEPS; ++i) {
        Serial.print("[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(state.steps[i].gate ? "ON" : "OFF");
        if (i < SEQUENCER_NUM_STEPS - 1) Serial.print(", ");
    }
    Serial.println();
}
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


// === Monophonic Note Duration Tracking (Step 2 integration plan) ===

/**
 * @brief Start a monophonic note with a specified duration (in ticks).
 * @param note MIDI note number to play.
 * @param duration Number of ticks the note should last.
 */
void Sequencer::startNote(uint8_t note, uint8_t velocity, uint16_t duration) {
    currentNote = note;
    noteDuration.start(duration);
    // Send NoteOn (velocity hardcoded to 100 for now, channel 1)
    // usb_midi.sendNoteOn(currentNote, velocity, 1); // (commented out for modularization)
    envelope.trigger();
}

void Sequencer::tickNoteDuration() {
    if (currentNote >= 0 && noteDuration.isActive()) {
        noteDuration.tick();
        if (!noteDuration.isActive()) {
            handleNoteOff();
        }
    }
}

void Sequencer::handleNoteOff() {
    if (currentNote >= 0) {
        // usb_midi.sendNoteOff(currentNote, 0, 1); // (commented out for modularization)
        currentNote = -1;
        envelope.release();
        noteDuration.reset();
    }
}
