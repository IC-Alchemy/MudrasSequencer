#ifndef LEDMATRIX_FEEDBACK_H
#define LEDMATRIX_FEEDBACK_H

#include "ledMatrix.h"
#include "../sequencer/Sequencer.h"

/**
 * @brief Updates the LED matrix display based on sequencer state, UI interactions, and sensor data.
 * This function should be called regularly (e.g., in a loop) to refresh the LED display.
 * It handles different visual feedback scenarios: parameter editing, step selection,
 * playhead position, active gates, and an idle breathing effect.
 *
 * @param ledMatrix Reference to the LEDMatrix object to control.
 * @param seq Const reference to the Sequencer object to get state from.
 * @param selectedStepForEdit Index of the step currently selected for editing (-1 if none).
 * @param button16Held True if the 'note edit' button (or similar) is held.
 * @param button17Held True if the 'velocity edit' button (or similar) is held.
 * @param button18Held True if the 'filter edit' button (or similar) is held.
 * @param recordButtonHeld True if the 'record' button is held.
 * @param mm Current Lidar distance reading, used for modulating LED brightness during parameter edits.
 */
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

/**
 * @brief Sets a specific LED for a sequencer step to a given RGB color.
 * This is a utility function that assumes step LEDs are on a specific row (e.g., row 0) of the matrix.
 * It relies on a global `ledMatrix` instance being available.
 *
 * @param step The sequencer step index (0-15) whose LED is to be set.
 * @param r Red component of the color (0-255).
 * @param g Green component of the color (0-255).
 * @param b Blue component of the color (0-255).
 */
void setStepLedColor(uint8_t step, uint8_t r, uint8_t g, uint8_t b);

#endif // LEDMATRIX_FEEDBACK_H