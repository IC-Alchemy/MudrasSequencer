/*
 * Matrix.cpp - Implementation of modular 32-button matrix scanning, debouncing,
 * and event dispatch See Matrix.h for usage and interface.
 */

#include "Matrix.h"

// --- Matrix Mapping Definitions ---
const uint8_t MATRIX_ROW_INPUTS[3] = {3, 2, 1};
const uint8_t MATRIX_COL_INPUTS[8] = {4, 5, 6, 7, 8, 9, 10, 11};

// --- Internal State ---
static MatrixButton matrixButtons[MATRIX_BUTTON_COUNT];
static bool buttonState[MATRIX_BUTTON_COUNT];  // Debounced state
static bool lastRawState[MATRIX_BUTTON_COUNT]; // Last raw state (for debounce)
static uint32_t debounceStartMs[MATRIX_BUTTON_COUNT]; // Debounce timer
static Adafruit_MPR121 *mpr121 = nullptr;
static void (*eventHandler)(const MatrixButtonEvent &) = nullptr;

// Debounce settings (ms)
static const uint16_t DEBOUNCE_MS = 10;

// --- Matrix Mapping Initialization ---
static void setupMatrixMapping() {
  uint8_t idx = 0;
  // Rows 1–3: Each button is a (row, col) pair
  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 8; ++col) {
      matrixButtons[idx].rowInput = MATRIX_ROW_INPUTS[row];
      matrixButtons[idx].colInput = MATRIX_COL_INPUTS[col];
      ++idx;
    }
  }
  // Row 4: Each button is a single col input (rowInput = 0)
  for (uint8_t col = 0; col < 8; ++col) {
    matrixButtons[idx].rowInput = 0;
    matrixButtons[idx].colInput = MATRIX_COL_INPUTS[col];
    ++idx;
  }
}

// --- Matrix Scanning (raw, not debounced) ---
static bool scanMatrixButton(const MatrixButton &btn, uint16_t touchBits) {
  if (btn.rowInput == 0) {
    // Row 4: Only column input, but must ensure no Row 1-3 button in this
    // column is pressed
    bool colActive = (touchBits & (1 << btn.colInput));
    if (!colActive)
      return false;
    // Check if any Row 1-3 button in this column is pressed
    for (uint8_t row = 0; row < 3; ++row) {
      uint8_t rowInput = MATRIX_ROW_INPUTS[row];
      if ((touchBits & (1 << rowInput)) && colActive) {
        return false; // A Row 1-3 button in this column is pressed, so Row 4 is
                      // not
      }
    }
    return true; // Only the column is active, so Row 4 is pressed
  } else {
    // Rows 1–3: Both row and column must be touched
    return (touchBits & (1 << btn.rowInput)) &&
           (touchBits & (1 << btn.colInput));
  }
}

// --- Debouncing and State Update ---
static void updateButtonStates(uint16_t touchBits) {
  uint32_t now = millis();
  for (uint8_t i = 0; i < MATRIX_BUTTON_COUNT; ++i) {
    bool raw = scanMatrixButton(matrixButtons[i], touchBits);
    if (raw != lastRawState[i]) {
      // State changed, start debounce timer
      debounceStartMs[i] = now;
      lastRawState[i] = raw;
    }
    if (raw != buttonState[i]) {
      // Only update if debounce interval has elapsed
      if (now - debounceStartMs[i] >= DEBOUNCE_MS) {
        buttonState[i] = raw;
        // Dispatch event if handler is set
        if (eventHandler) {
          MatrixButtonEvent evt;
          evt.buttonIndex = i;
          evt.type = raw ? MATRIX_BUTTON_PRESSED : MATRIX_BUTTON_RELEASED;
          eventHandler(evt);
        }
      }
    }
  }
}

// --- Public API ---

void Matrix_init(Adafruit_MPR121 *sensor) {
  mpr121 = sensor;
  setupMatrixMapping();
  for (uint8_t i = 0; i < MATRIX_BUTTON_COUNT; ++i) {
    buttonState[i] = false;
    lastRawState[i] = false;
    debounceStartMs[i] = 0;
  }
  eventHandler = nullptr;
}

void Matrix_scan() {
  if (!mpr121)
    return;
  uint16_t touchBits = mpr121->touched();
  updateButtonStates(touchBits);
}

bool Matrix_getButtonState(uint8_t idx) {
  if (idx >= MATRIX_BUTTON_COUNT)
    return false;
  return buttonState[idx];
}

void Matrix_setEventHandler(void (*handler)(const MatrixButtonEvent &)) {
  eventHandler = handler;
}

void Matrix_printState() {
  Serial.println("Button Matrix State (1=pressed, 0=not pressed):");
  for (uint8_t row = 0; row < 4; ++row) {
    for (uint8_t col = 0; col < 8; ++col) {
      uint8_t idx = row * 8 + col;
      Serial.print(buttonState[idx] ? "1 " : "0 ");
    }
    Serial.println();
  }
  Serial.println();
}