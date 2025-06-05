# Step Length and Note Duration Integration Plan

## Objective

Integrate a flexible step length and note duration mechanism into the Pico2DSP2CoreWorks sequencer, inspired by the AcidStep.ino TB-303 style sequencer. This will allow for variable pattern lengths and accurate note-off timing, supporting features like glide and per-step gate length.

---

## 1. Analysis of AcidStep.ino

- **Step Length:**  
  - Controlled by `_step_length`, which sets the number of active steps in the sequence.
  - The playhead advances as `tick % _step_length`, enabling patterns shorter than the maximum step count.

- **Note Duration:**  
  - `NOTE_LENGTH` sets the base note duration.
  - Glide extends the note duration if consecutive steps have glide enabled.
  - Notes are tracked in a stack with their remaining durations decremented on each clock tick. NoteOff is sent when duration reaches zero.

---

## 2. Integration Plan

### 2.1. Add Step Length Parameter

- Add a `stepLength` member to the `Sequencer` class (default 16, user-adjustable).
- Use `stepLength` for playhead wrapping and step advancement.

### 2.2. Implement Note Duration Tracking

- Add a note stack (array of structs) to the sequencer to track currently playing notes and their remaining durations.
- On each clock tick (PPQN), decrement durations and send NoteOff when needed.
- Allow note duration to be extended for glide/slide steps.

### 2.3. Modify Step Advance Logic

- In `advanceStep`, use `playhead = tick % stepLength`.
- When a step is triggered, calculate its duration (considering glide).
- Add note to stack with duration.
- Only send NoteOn if the step is not a rest/gate-off.

### 2.4. Architectural Adjustments

- Decouple note triggering from immediate envelope/MIDI output; instead, use the note stack to manage NoteOn/NoteOff timing.
- Optionally, allow per-step gate length for more expressiveness.

### 2.5. User Interface/Parameter Control

- Expose step length as a user-adjustable parameter (via UI, MIDI CC, or button combo).
- Optionally, allow per-step glide and accent, as in AcidStep.ino.

---

## 3. Mermaid Diagram

```mermaid
flowchart TD
    subgraph Sequencer
        direction LR
        ClockTick[Clock Tick (PPQN)]
        AdvanceStep[Advance Step (tick % stepLength)]
        CheckStep[Check Step (gate/rest/glide)]
        NoteStack[Note Stack: Track note, duration]
        DecrementStack[Decrement durations]
        SendNoteOn[Send NoteOn]
        SendNoteOff[Send NoteOff]
    end

    ClockTick --> AdvanceStep
    AdvanceStep --> CheckStep
    CheckStep -- If gate ON --> NoteStack
    NoteStack --> SendNoteOn
    ClockTick --> DecrementStack
    DecrementStack -- If duration==0 --> SendNoteOff
```

---

## 4. Required Code Changes

### 4.1. Sequencer Class

- Add `uint8_t stepLength` (default 16).
- Add `struct NoteStackEntry { int note; int duration; } noteStack[SIZE];`
- Add methods for managing note stack and durations.

### 4.2. Step Advance Logic

- Use `playhead = tick % stepLength`.
- On step trigger, calculate note duration (consider glide).
- Add note to stack with duration.

### 4.3. Clock Handler

- On each PPQN tick, decrement durations in note stack.
- Send NoteOff when duration reaches zero.

### 4.4. UI/Parameter Handling

- Provide a way to set `stepLength` at runtime.

---

## 5. Trade-offs and Considerations

- **Pros:**  
  - Enables variable pattern lengths and expressive note timing.
  - Supports TB-303 style glide and accents.
  - Decouples note-on/off timing from immediate step events.

- **Cons:**  
  - Increases sequencer complexity.
  - Requires careful synchronization between step advancement and note stack management.

---

## 6. Next Steps

1. Implement the above changes in the sequencer codebase.
2. Test with various step lengths and glide configurations.
3. Update documentation and UI to reflect new features.