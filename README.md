# Pico2DSP2Core

# Pico2DSP2CoreWorks Firmware Documentation

## Overview

Pico2DSP2CoreWorks is a modular firmware designed for the Pico2

- Dual Core, uses DMA w/ PIO for audio synthesis on core0 and core1 used for everything else
- Audio synthesis with multiple oscillators and ADSR envelopes
- A 16-step sequencer with note, gate, velocity, and filter control
- OLED display for sequencer visualization
- Capacitive touch matrix input using MPR121 sensor
- MIDI input/output and clock synchronization
- Distance sensor integration paramter editing


## Main Firmware (`Pico2DSP2CoreWorks.ino`)

This is the central firmware file that integrates audio synthesis, sequencing, display, touch input, MIDI, and sensor data.


### MIDI and Clock Handling

- MIDI communication is handled via USB MIDI using the Adafruit TinyUSB and MIDI libraries.
- The clock system (`uClock`) synchronizes sequencer steps and MIDI clock messages.

### Distance Sensor Integration

- A VL53L1X time-of-flight distance sensor measures distance in millimeters.
- Distance data is used to sequence step data per step or in realtime as the sequence plays.
---

## Touch Matrix Module (`src/matrix`)

### Matrix Overview

This module manages a 32-button capacitive touch matrix using a single MPR121 sensor. It handles scanning and event dispatching.





License and Acknowledgements
This project uses several open-source libraries. Please see their respective licenses for details:

FastLED — MIT License — https://github.com/FastLED/FastLED
Adafruit MPR121 — BSD License — https://github.com/adafruit/Adafruit_MPR121_Library
Adafruit SSD1306 — BSD License — https://github.com/adafruit/Adafruit_SSD1306
Melopero VL53L1X — MIT License — https://github.com/melopero/Melopero_VL53L1X
DaisySP — MIT License — https://github.com/electro-smith/DaisySP
Arduino MIDI Library — MIT License — https://github.com/FortySevenEffects/arduino_midi_library
TinyUSB — BSD License — https://github.com/hathach/tinyusb
Please refer to the individual library repositories for full license texts and compliance requirements.