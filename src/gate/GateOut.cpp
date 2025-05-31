/**
 * @file GateOut.cpp
 * @brief Implementation of Arduino-compatible gate output abstraction.
 */

#include "GateOut.h"

GateOut::GateOut(uint8_t pin) : _pin(pin) {}

void GateOut::begin() {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
}

void GateOut::setHigh() { digitalWrite(_pin, HIGH); }

void GateOut::setLow() { digitalWrite(_pin, LOW); }

void GateOut::set(bool state) { digitalWrite(_pin, state ? HIGH : LOW); }

uint8_t GateOut::pin() const { return _pin; }