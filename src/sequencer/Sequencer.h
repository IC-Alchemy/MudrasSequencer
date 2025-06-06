/**
 * @file Sequencer.h
 * @brief Modular 16-step sequencer class interface.
 *
 * Provides a step sequencer with step toggle, note assignment, playhead
 * advance, and state query. Designed for integration with matrix scanning and
 * output modules (MIDI, gate).
 *
 * Example:
 *   #include "Sequencer.h"
 *   Sequencer seq;
 *   // Initialize and start
 *   if (!seq.init()) { Serial.println("Sequencer init failed"); }
 *   seq.start();
 *
 *   // Configure steps
 *   // step 0: gate on, slide off, note index 8, velocity 0.75, filter 0.3
 *   seq.setStep(0, true, false, 8, 0.75f, 0.3f);
 *   // step 1 using Step object
 *   seq.setStep(1, Step(true, true, 12, 1.0f, 0.5f));
 *
 *   // In clock callback
 *   void onClockTick(uint8_t beat) {
 *       seq.advanceStep(beat);
 *       const Step& stepData = seq.getStep(beat);
 *       // Use stepData.gate, stepData.note, stepData.velocity, stepData.filter
 *   }
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "SequencerDefs.h"
extern volatile bool trigenv1;
extern volatile bool trigenv2;
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#define SEQUENCER_NUM_STEPS 16

extern midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>> usb_midi;

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
                   int current_selected_step_for_edit);

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
    void startNote(uint8_t note, float velocity, uint16_t duration);

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

#endif // SEQUENCER_H