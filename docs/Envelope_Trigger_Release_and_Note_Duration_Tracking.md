# Envelope Triggering, Release, and Note Duration Tracking

This document provides a comprehensive overview of the concepts of envelope triggering and release, as well as note duration tracking as implemented in the sequencer system. It includes definitions, detailed explanations of the mechanisms involved, practical examples from sound synthesis and music production, and illustrative code snippets.

---

## Table of Contents

- [1. Envelope Triggering and Release](#1-envelope-triggering-and-release)  
  - [1.1 Definitions](#11-definitions)  
  - [1.2 How Envelope Triggering Works](#12-how-envelope-triggering-works)  
  - [1.3 How Envelope Release Works](#13-how-envelope-release-works)  
  - [1.4 Practical Example](#14-practical-example)

- [2. Note Duration Tracking](#2-note-duration-tracking)  
  - [2.1 Definition](#21-definition)  
  - [2.2 Implementation Details](#22-implementation-details)  
  - [2.3 Practical Example](#23-practical-example)

- [3. Integration in the Sequencer System](#3-integration-in-the-sequencer-system)  
  - [3.1 Triggering and Release Code Snippets](#31-triggering-and-release-code-snippets)  
  - [3.2 Note Duration Management Example](#32-note-duration-management-example)

---

## 1. Envelope Triggering and Release

### 1.1 Definitions

- **Envelope Triggering:** The initiation of a sound envelope which shapes the amplitude, filter cutoff, or other parameters of a sound over time starting from a note-on event.
- **Envelope Release:** The phase following the release of a note, where the envelope transitions from sustaining the sound to silence or another state, typically involving a decay.

### 1.2 How Envelope Triggering Works

Envelope triggering occurs at the start of a note or step in the sequencer. It sets an internal flag or signal that activates the envelope generator, causing it to begin the attack phase. In the sequencer code, this is managed by setting global boolean flags (`trigenv1`, `trigenv2`) to `true`, which the audio engine monitors to start shaping the sound.

### 1.3 How Envelope Release Works

When a note is turned off or its duration expires, the envelope release phase begins. This is triggered by clearing the envelope flags (`trigenv1`, `trigenv2` set to `false`) signaling the envelope generator to move towards silence. This handling ensures smooth transitions and avoids abrupt cuts in the sound.

### 1.4 Practical Example

```cpp
// Trigger envelope on note start
void Sequencer::triggerEnvelope() {
    trigenv1 = true;
    trigenv2 = true;
}

// Release envelope on note end
void Sequencer::releaseEnvelope() {
    trigenv1 = false;
    trigenv2 = false;
}
```

---

## 2. Note Duration Tracking

### 2.1 Definition

Note duration tracking is the process of measuring and controlling how long a note remains active before automatically triggering its release. This is important for precise timing and articulation in sequenced music.

### 2.2 Implementation Details

The sequencer uses a `NoteDurationTracker` class that maintains a counter representing the number of ticks remaining for a note. This counter decrements with each call to `tick()`. When the counter reaches zero, the note is considered inactive, and the release process is initiated.

```cpp
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
```

### 2.3 Practical Example

Each sequencer tick calls `tickNoteDuration()`, which decrements the note duration counter and triggers a note-off if duration has ended:

```cpp
void Sequencer::tickNoteDuration() {
    if (currentNote >= 0 && noteDuration.isActive()) {
        noteDuration.tick();
        if (!noteDuration.isActive()) {
            handleNoteOff();
        }
    }
}
```

---

## 3. Integration in the Sequencer System

### 3.1 Triggering and Release Code Snippets

In the main step advancement routine (`advanceStep()`), notes are started and envelopes triggered as follows:

```cpp
void Sequencer::advanceStep(uint8_t current_uclock_step, int mm_distance, bool is_button16_held, bool is_button17_held, bool is_button18_held, int current_selected_step_for_edit, VoiceState* voiceState) {
    // ...
    if (currentStep.gate) {
        int new_midi_note = MIDI_BASE_NOTE + scale[currentScale][scaleIndex];
        note1 = new_midi_note;
        vel1 = currentStep.velocity;
        freq1 = currentStep.filter;

        startNote(new_midi_note, vel1 * 127, 24); // Start note with fixed duration
        lastNote = new_midi_note;
    } else {
        handleNoteOff();
        lastNote = -1;
    }
}
```

### 3.2 Note Duration Management Example

## 4. Flowchart of Envelope Triggering and Note Duration Tracking

```mermaid
flowchart TD

  A[Sequencer Step Advances]
  B[Check if Step Gate is ON]
  C[Calculate MIDI Note]
  D[Set Global Note Parameters (note1, vel1, freq1)]
  E[Start Note with Fixed Duration]
  F[Trigger Envelope (trigenv1, trigenv2 = true)]
  G[Note Duration Active?]
  H[Decrement Note Duration Counter]
  I[Note Duration Expired?]
  J[Handle Note Off]
  K[Release Envelope (trigenv1, trigenv2 = false)]
  L[Reset Note Duration Tracker]
  M[End of Step Processing]

  A --> B
  B -- Yes --> C
  C --> D
  D --> E
  E --> F
  F --> G
  G -- Yes --> H
  H --> I
  I -- Yes --> J
  J --> K
  K --> L
  L --> M
  I -- No --> M
  B -- No --> J
  J --> K
  K --> L
  L --> M
```
The `startNote()` method initializes the note, including starting the duration counter and triggering the envelope, while `handleNoteOff()` releases the envelope:

```cpp
void Sequencer::startNote(int midiNote, uint8_t velocity, uint16_t durationTicks) {
    currentNote = midiNote;
    vel1 = velocity / 127.0f;
    noteDuration.start(durationTicks);
    triggerEnvelope();
}

void Sequencer::handleNoteOff() {
    releaseEnvelope();
    currentNote = -1;
    noteDuration.reset();
}
```

---

## Summary

This sequencer system uses a straightforward, modular approach to manage envelope triggering and note duration tracking. It efficiently supports fixed-length notes, provides clear trigger and release phases for envelopes, and maintains precise note length control suitable for embedded DSP environments. The design is performant, but could be expanded for added flexibility and polyphonic support.

---

*End of Document*