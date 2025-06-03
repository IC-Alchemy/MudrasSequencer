#ifndef LEDMATRIX_FEEDBACK_H
#define LEDMATRIX_FEEDBACK_H

#include "ledMatrix.h"
#include "src/sequencer/Sequencer.h"

// Call this regularly (e.g., in loop1) to update step LEDs based on sequencer and UI state
void updateStepLEDs(
    LEDMatrix& ledMatrix,
    const Sequencer& seq,
    int selectedStepForEdit,
    bool button16Held,
    bool button17Held,
    bool button18Held,
    bool recordButtonHeld,
    int mm // Lidar distance
);

#endif // LEDMATRIX_FEEDBACK_H