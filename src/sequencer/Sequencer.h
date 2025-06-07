/*
 * Sequencer.h
 * Modular step sequencer for embedded synths.
 * - Supports per-step gate, slide, note, velocity, filter.
 * - Designed for integration with matrix scanning, MIDI, and DSP output.
 * - Easily extensible for polyphony and parameter automation.
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "SequencerDefs.h"
#include <cstdint>
extern volatile bool trigenv1;
extern volatile bool trigenv2;
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#define SEQUENCER_NUM_STEPS 16

extern midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>> usb_midi;

#include <functional>

// Modular envelope controller
class EnvelopeController {
public:
    void trigger() { triggered = true; released = false; }
    void release() { triggered = false; released = true; }
    bool isTriggered() const { return triggered; }
    bool isReleased() const { return released; }
private:
    bool triggered = false;
    bool released = true;
};

// Modular note duration tracker
class NoteDurationTracker {
public:
    void start(uint16_t duration) { counter = duration; active = true; }
    void tick() { if (active && counter > 0) { --counter; if (counter == 0) active = false; } }
    bool isActive() const { return active && counter > 0; }
    void reset() { counter = 0; active = false; }
private:
    uint16_t counter = 0;
    bool active = false;
};

struct ParameterMapping {
    std::function<bool()> isActive;
    std::function<void(Step&, int)> apply;
};

class Sequencer {
public:
  Sequencer();

  // Step length (number of steps in the sequence, user-adjustable, max 16)
  uint8_t getStepLength() const { return stepLength; }
  void setStepLength(uint8_t len) { stepLength = (len > 0 && len <= SEQUENCER_NUM_STEPS) ? len : SEQUENCER_NUM_STEPS; }

  // Instantly play a step for real-time feedback (does not advance playhead)
  void playStepNow(uint8_t stepIdx);

 
  void init();

 
  // Start/stop sequencer
  void start();
  void stop();
  void reset();

  /**
   * @brief Processes the sequencer logic for the given step.
   * @param current_uclock_step The current step number (0-15) provided by uClock.
   */
  void advanceStep(uint8_t current_uclock_step, int mm_distance,
                    bool is_button16_held, bool is_button17_held, bool is_button18_held,
                   int current_selected_step_for_edit,
                   struct VoiceState* voiceState = nullptr);

  // Toggle step ON/OFF
  void toggleStep(uint8_t stepIdx);

  // Set note for a step
  void setStepNote(uint8_t stepIdx, uint8_t note);
void setStepVelocity(uint8_t stepIdx, uint8_t velocity);
void setStepFiltFreq(uint8_t stepIdx, float filter);
  // Set full step data (overloads)
  void setStep(int index, bool gate, bool slide, int note, float velocity, float filter);
  void setStep(int index, const Step& stepData);

// Convert absolute MIDI note (0-127) to the semitone-offset scheme used by the audio thread
    void setOscillatorFrequency(uint8_t midiNote);
  // Query step and playhead state
  const Step &getStep(uint8_t stepIdx) const;
  uint8_t getPlayhead() const;
  bool isRunning() const;
  
public:
  int8_t getLastNote() const;
  void setLastNote(int8_t note);

const SequencerState& getState() const;
void triggerEnvelope();
  void releaseEnvelope();


private:
  // Sequencer state now stored in SequencerState from SequencerDefs.h
  void resetState();
  void initializeSteps();
  bool validateState() const;
  SequencerState state;
  bool errorFlag = false;
  EnvelopeController envelope;
  NoteDurationTracker noteDuration;

  /**
   * @brief Tracks the last played MIDI note for proper noteOff handling.
 * Stores the actual MIDI note value sent. -1 means no note is currently playing.
   */
  int8_t lastNote = -1;
private:
  uint8_t stepLength = SEQUENCER_NUM_STEPS; // Default 16, user-adjustable
public:
    // Monophonic note duration tracking (Step 2 integration plan)
    /**
     * @brief Start a monophonic note with a specified duration (in ticks).
     * @param note MIDI note number to play.
     * @param duration Number of ticks the note should last.
     */
    void startNote(uint8_t note, uint8_t velocity,  uint16_t duration);

    /**
     * @brief Decrement the note duration counter. If zero, sends NoteOff and clears state.
     */
    void tickNoteDuration();

    /**
     * @brief Sends NoteOff for the current note and clears the active note state.
     */
    void handleNoteOff();

private:
    // Monophonic note duration tracking variables
    int8_t currentNote = -1;           // -1 means no note is currently active
    uint16_t noteDurationCounter = 0;  // Remaining duration in ticks

};

#ifdef ARDUINO
#include <Arduino.h>
#endif

#endif // SEQUENCER_H