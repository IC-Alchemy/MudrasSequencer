/**
 * @file Sequencer.h
 * @brief Modular 16-step sequencer class interface.
 *
 * Provides a step sequencer with step toggle, note assignment, playhead
 * advance, and state query. It receives dependencies like the MIDI interface
 * and musical scale data via its constructor.
 *
 * Example:
 *   #include "Sequencer.h"
 *   #include "SynthState.h" // For g_synthVoiceState
 *   // ... other includes and global objects for MIDI, scale ...
 *   Sequencer seq(usb_midi_interface, scale_array);
 *
 *   // Initialize and start
 *   if (!seq.init()) { Serial.println("Sequencer init failed"); }
 *   seq.start();
 *
 *   // Configure steps
 *   // step 0: gate on, slide off, note index 8, velocity 0.75, filter 0.3
 *   seq.setStep(0, true, false, 8, 0.75f, 0.3f);
 *
 *   // In clock callback
 *   void onClockTick(uint8_t uclock_step) { // uclock_step is the uClock's current step
 *       seq.advanceStep(uclock_step); // Sequencer updates g_synthVoiceState
 *       // g_synthVoiceState can now be used by the audio engine
 *   }
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "SequencerDefs.h"
#include <Adafruit_TinyUSB.h> // For MIDI types if not fully opaque
#include <MIDI.h>             // For MIDI types if not fully opaque
#include "../SynthState.h"    // For SynthVoiceState

// Make g_synthVoiceState available to the sequencer (defined in main .ino)
extern volatile SynthVoiceState g_synthVoiceState;

#define SEQUENCER_NUM_STEPS 16

// Type alias for the scale array reference
using ScaleArrayRefType = const int (&)[5][48];

class Sequencer {
public:
  /**
   * @brief Constructor for the Sequencer.
   * @param midi_interface Reference to the MIDI interface for sending MIDI messages.
   * @param scale_data Reference to the array defining musical scales.
   */
  Sequencer(midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>>& midi_interface, ScaleArrayRefType scale_data);

  /**
   * @brief Instantly plays a step for real-time feedback. Does not advance the main playhead.
   * Updates g_synthVoiceState with the parameters of the specified step.
   * @param stepIdx The index of the step to play (0 to SEQUENCER_NUM_STEPS - 1).
   */
  void playStepNow(uint8_t stepIdx);

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

  /**
   * @brief Set the note for a specific step.
   * @param stepIdx Index of the step (0 to SEQUENCER_NUM_STEPS - 1).
   * @param note Note value for the step (typically a scale index, 0-24 or similar).
   */
  void setStepNote(uint8_t stepIdx, uint8_t note);

  /**
   * @brief Set the velocity for a specific step.
   * @param stepIdx Index of the step (0 to SEQUENCER_NUM_STEPS - 1).
   * @param velocity Velocity value for the step (normalized float 0.0f - 1.0f).
   */
  void setStepVelocity(uint8_t stepIdx, float velocity);

  /**
   * @brief Set the filter cutoff for a specific step.
   * @param stepIdx Index of the step (0 to SEQUENCER_NUM_STEPS - 1).
   * @param filter Filter value for the step (normalized float 0.0f - 1.0f).
   */
  void setStepFiltFreq(uint8_t stepIdx, float filter);

  // Set full step data (overloads)
  // TODO: Review Doxygen for these setStep overloads if they are primary interaction points.
  void setStep(int index, bool gate, bool slide, int note, float velocity, float filter);
  void setStep(int index, const Step& stepData);

  /**
   * @brief Sets the synth's current note directly. Used by playStepNow or external MIDI input.
   * This updates g_synthVoiceState.note.
   * @param midiNote The MIDI note value (0-127).
   */
  void setOscillatorFrequency(uint8_t midiNote); // Name is a bit misleading, it sets the MIDI note.

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
  midi::MidiInterface<midi::SerialMIDI<Adafruit_USBD_MIDI>>& midi_interface_;
  ScaleArrayRefType scale_data_;
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