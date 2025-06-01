# Refactoring Plan: Integrate New Step API into Sequencer Module

## Overview

This document outlines a plan to replace the existing `StepData`/`StepState` approach in the Sequencer module with a unified `Step` struct and add two `setStep(...)` overloads.

## Steps

1. **Update Step Data Definition**  
   - Edit [`src/sequencer/SequencerDefs.h`](src/sequencer/SequencerDefs.h:29)  
   - Remove the old `StepState` enum and legacy `struct Step` definition.  
   - Add:
   ```cpp
   struct Step {
     bool gate = false;      // Gate ON (true) or OFF (false)
     bool slide = false;     // Slide ON (true) or OFF (false)
     int note = 0;           // Note value, 0-24
     float velocity = 0.0f;  // Velocity, 0.0f - 1.0f
     float filter = 0.0f;    // Filter value, 0.0f - 1.0f

     Step() = default;
     Step(bool g, bool s, int n, float v, float f)
         : gate(g), slide(s), note(n), velocity(v), filter(f) {}
   };
   ```

2. **Refactor Sequencer Interface**  
   - Edit [`src/sequencer/Sequencer.h`](src/sequencer/Sequencer.h:29)  
   - Remove `typedef StepData` and private `StepData steps[...]`.  
   - Add public overloads:
   ```cpp
   void setStep(int index, bool gate, bool slide, int note, float velocity, float filter);
   void setStep(int index, const Step& stepData);
   ```

3. **Implement New Methods & Migrate Logic**  
   - Edit [`src/sequencer/Sequencer.cpp`](src/sequencer/Sequencer.cpp:130)  
   - For each overload, perform range checks (`note` ∈ [0,24], `velocity/filter` ∈ [0.0,1.0]) and throw `std::out_of_range` on invalid.  
   - Assign validated values to `state.steps[index]`.  
   - Remove `StepState` usage: test ON/OFF via `step.gate`.  
   - Deprecate `setStepNote()`.

4. **Update Main Sketch Usage**  
   - Edit [`Pico2DSP2CoreWorks.ino`](Pico2DSP2CoreWorks.ino:280)  
   - Replace calls to `toggleStep()` / `setStepNote()` with the new `setStep(...)` overloads.

## Class Diagram
```mermaid
classDiagram
    class Step {
      +bool gate
      +bool slide
      +int note
      +float velocity
      +float filter
      +Step()
      +Step(bool,bool,int,float,float)
    }
    class Sequencer {
      +Sequencer()
      +bool init()
      +void start(), stop(), reset()
      +void advanceStep(uint8_t)
      +void setStep(int,bool,bool,int,float,float)
      +void setStep(int,const Step&)
      +const Step& getStep(uint8_t) const
      +uint8_t getPlayhead() const
    }
    Sequencer "1" -- "16" Step : contains >