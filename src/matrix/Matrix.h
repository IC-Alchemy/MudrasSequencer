/*
 * Matrix.h - Modular 32-button matrix scanning, debouncing, and event dispatch
 *
 * Provides a clean interface for initializing, scanning, and handling events
 * from a 4x8 button matrix using a single MPR121 capacitive touch sensor.
 *
 * Usage:
 *   #include "matrix/Matrix.h"
 *   ...
 *   Matrix_init(&touchSensor); // Pass pointer to Adafruit_MPR121 instance
 *   Matrix_setEventHandler(myEventHandler); // Optional: set your event handler
 *   ...
 *   void loop() {
 *     Matrix_scan(); // Call frequently (e.g., every 1ms)
 *     ...
 *   }
 *   // To query debounced state:
 *   bool pressed = Matrix_getButtonState(idx);
 *
 * See matrix_layout.md for mapping details.
 *
 * Only matrix logic is included; no application-specific code.
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <Adafruit_MPR121.h>
#include <Arduino.h>


// Logical button count
#define MATRIX_BUTTON_COUNT 32

// MPR121 input numbers for rows and columns
extern const uint8_t MATRIX_ROW_INPUTS[4];
extern const uint8_t MATRIX_COL_INPUTS[8];

// MatrixButton: maps a logical button to its MPR121 input(s)
typedef struct {
  uint8_t rowInput; // Always used (0-3)
  uint8_t colInput; // Always used (0-7)
} MatrixButton;

// Button event type
typedef enum {
  MATRIX_BUTTON_PRESSED,
  MATRIX_BUTTON_RELEASED
} MatrixButtonEventType;

// ButtonEvent: describes a button state change
typedef struct {
  uint8_t buttonIndex;        // 0–31 (logical button number)
  MatrixButtonEventType type; // Pressed or Released
} MatrixButtonEvent;

// Initialize the matrix system (call in setup)
void Matrix_init(Adafruit_MPR121 *sensor);

// Scan the matrix, update debounced state, and dispatch events (call
// frequently)
void Matrix_scan();

// Get the debounced state of a button (0–31)
bool Matrix_getButtonState(uint8_t idx);

/**
 * Set the event handler for button events (optional)
 */
void Matrix_setEventHandler(void (*handler)(const MatrixButtonEvent &));

/**
 * Set the rising edge (button pressed) callback.
 * This will be called when a button transitions from not pressed to pressed.
 */
void Matrix_setRisingEdgeHandler(void (*handler)(uint8_t buttonIndex));

// Print the current button matrix state to Serial (for debugging)
void Matrix_printState();

#endif // MATRIX_H