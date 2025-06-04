#include "LEDMatrixFeedback.h"
#include <Arduino.h>
#include <FastLED.h>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <Arduino.h>
#include <math.h>

void setStepLedColor(uint8_t step, uint8_t r, uint8_t g, uint8_t b) {
    extern LEDMatrix ledMatrix; // Use the global ledMatrix instance
    ledMatrix.setLED(step, 0, CRGB(r, g, b));
}

void updateStepLEDs(
    LEDMatrix& ledMatrix,
    const Sequencer& seq,
    int selectedStepForEdit,
    bool button16Held,
    bool button17Held,
    bool button18Held,
    bool recordButtonHeld,
    int mm // Lidar distance
) {
    const SequencerState& state = seq.getState();

    bool paramEditActive = (button16Held || button17Held || button18Held || recordButtonHeld);

    // Main step LED logic
    for (int step = 0; step < SEQUENCER_NUM_STEPS; ++step) {
        const Step& s = state.steps[step];
        bool isPlayhead = (state.playhead == step);
        bool isSelected = (step == selectedStepForEdit);
        bool isParamEdit = isSelected && paramEditActive;

        CRGB color = CRGB::Black;

        // Priority logic (highest to lowest)
        if (isParamEdit) {
            // Yellow, brightness modulated by Lidar (mm: 0-1400 mapped to 32-255)
            uint8_t brightness = map(mm, 0, 1400, 32, 255);
            color = CRGB(brightness, brightness, 0);
        } else if (isSelected) {
            // Cyan pulse for selected step
            float t = millis() / 1000.0f;
            float pulse = 0.5f * (1.0f + sinf(2.0f * 3.1415926f * 1.5f * t));
            uint8_t b = (uint8_t)(pulse * 255.0f);
            color = CRGB(0, b, b);
        } else if (isPlayhead && s.gate) {
            // White chase flash for playhead on active step
            color = CRGB::White;
        } else if (s.gate) {
            // Solid green for gate ON
            color = CRGB::Green;
        } else {
            // Dim blue for gate OFF
            color = CRGB(0, 0, 48);
        }

        ledMatrix.setLED(step, 0, color); // Assuming row 0 for step LEDs
    }

    // Idle breathing effect if sequencer stopped and no steps selected
    if (!state.running && selectedStepForEdit == -1) {
        float t = millis() / 2000.0f;
        float breath = 0.5f * (1.0f + sinf(2.0f * 3.1415926f * t));
        uint8_t b = (uint8_t)(breath * 64.0f + 16.0f);
        for (int step = 0; step < SEQUENCER_NUM_STEPS; ++step) {
            ledMatrix.setLED(step, 0, CRGB(0, 0, b));
        }
    }

    ledMatrix.show();
}