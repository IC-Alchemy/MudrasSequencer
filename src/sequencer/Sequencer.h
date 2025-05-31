/**
 * @file Sequencer.h
 * @brief Modular 16-step sequencer class interface.
 *
 * Provides a step sequencer with step toggle, note assignment, playhead
 * advance, and state query. Designed for integration with matrix scanning and
 * output modules (MIDI, gate).
 *
 * Usage:
 *   #include "Sequencer.h"
 *   Sequencer seq;
 *   seq.start();
 *   seq.advanceStep();
 *   seq.toggleStep(3);
 *   uint8_t ph = seq.getPlayhead();
 *   const Step& s = seq.getStep(3);
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "SequencerDefs.h"
extern volatile bool trigenv1;
extern volatile bool trigenv2;
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

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

  // Advance playhead to next step (wraps at end)
  void advanceStep();

  // Toggle step ON/OFF
  void toggleStep(uint8_t stepIdx);

  // Set note for a step
  void setStepNote(uint8_t stepIdx, uint8_t note);

  // Query step and playhead state
  const Step &getStep(uint8_t stepIdx) const;
  uint8_t getPlayhead() const;
  bool isRunning() const;

public:
    int8_t getLastNote() const;
    void setLastNote(int8_t note);

const SequencerState& getState() const;

private:
    void resetState();
    void initializeSteps();
    bool validateState() const;
    SequencerState state;
    bool errorFlag = false;

    /**
     * @brief Tracks the last played MIDI note for proper noteOff handling.
     * -1 means no note is currently playing.
     */
    int8_t lastNote = -1;
};

#endif // SEQUENCER_H