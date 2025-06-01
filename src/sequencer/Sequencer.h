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

  /**
   * @brief Initialize the sequencer to a known good state.
   *
   * Resets all steps, playhead, and running state to defaults. Validates the integrity
   * of the internal state array. If any issue is detected, sets an internal error flag
   * and returns false. Safe to call multiple times (idempotent).
   *
   * @return true if initialization succeeded, false if an error was detected.
   */
  bool init();

  /**
   * @brief Check if the sequencer is in an error state after initialization.
   * @return true if an error was detected during the last init(), false otherwise.
   */
  bool hasError() const;

  // Start/stop sequencer
  void start();
  void stop();
  void reset();

  /**
   * @brief Processes the sequencer logic for the given step.
   * @param current_uclock_step The current step number (0-15) provided by uClock.
   */
  void advanceStep(uint8_t current_uclock_step);

  // Toggle step ON/OFF
  void toggleStep(uint8_t stepIdx);

  // Set note for a step
  void setStepNote(uint8_t stepIdx, uint8_t note);

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
};

#endif // SEQUENCER_H