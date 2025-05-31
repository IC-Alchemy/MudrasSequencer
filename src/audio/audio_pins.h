#pragma once
// Override pico-audio I2S pin assignments for your hardware
#define PICO_AUDIO_I2S_DATA_PIN 15
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16
// Note: MCLK (pin 17) is not handled by pico-audio by default; generate
// separately if needed.