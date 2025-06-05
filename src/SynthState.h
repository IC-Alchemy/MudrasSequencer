// src/SynthState.h
#ifndef SYNTH_STATE_H
#define SYNTH_STATE_H

/**
 * @brief Represents the state of a single synthesizer voice.
 *
 * This struct holds parameters like MIDI note, velocity, filter cutoff frequency,
 * and gate status, which are essential for controlling a synth voice.
 */
struct SynthVoiceState {
    int note;             ///< MIDI note number (e.g., 0-127).
    float velocity;       ///< Note velocity, normalized (0.0f to 1.0f).
    float filterCutoff;   ///< Filter cutoff frequency, typically in Hz.
    bool gateOn;          ///< True if the gate is currently open (note is active), false otherwise.
};

#endif // SYNTH_STATE_H
