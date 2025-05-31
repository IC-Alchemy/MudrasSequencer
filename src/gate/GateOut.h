/**
 * @file GateOut.h
 * @brief Arduino-compatible gate output abstraction.
 *
 * Provides a simple interface for controlling digital gate outputs (e.g., for
 * triggering envelopes or steps). Designed for use by sequencer or main
 * program, independent of matrix, sequencer, or encoder code.
 *
 * Usage Example:
 *   #include "GateOut.h"
 *   GateOut gate(5); // Use digital pin 5
 *   gate.begin();    // Call in setup()
 *   gate.setHigh();  // Set gate high (on)
 *   gate.setLow();   // Set gate low (off)
 */

#ifndef GATEOUT_H
#define GATEOUT_H

#include <Arduino.h>

class GateOut {
public:
  /**
   * @brief Construct a GateOut for a specific digital pin.
   * @param pin Arduino digital pin number
   */
  explicit GateOut(uint8_t pin);

  /**
   * @brief Initialize the gate pin (sets pinMode to OUTPUT).
   * Call this in setup().
   */
  void begin();

  /**
   * @brief Set the gate output high (on).
   */
  void setHigh();

  /**
   * @brief Set the gate output low (off).
   */
  void setLow();

  /**
   * @brief Set the gate output to a specific state.
   * @param state true for high, false for low
   */
  void set(bool state);

  /**
   * @brief Get the pin number associated with this gate.
   */
  uint8_t pin() const;

private:
  uint8_t _pin;
};

#endif // GATEOUT_H