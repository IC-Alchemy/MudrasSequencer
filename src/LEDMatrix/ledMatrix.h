#ifndef LEDMATRIX_H
#define LEDMATRIX_H

#include <Arduino.h>
#include <FastLED.h>

class LEDMatrix {
public:
    static constexpr uint8_t WIDTH = 16;
    static constexpr uint8_t HEIGHT = 8;
    static constexpr uint8_t DATA_PIN = 15;
    static constexpr uint8_t CLOCK_PIN = 14;
    static constexpr uint8_t LATCH_PIN = 13;
    static constexpr uint8_t OE_PIN = 12;

    // Color aliases for convenience
    static constexpr CRGB blue = CRGB::Blue;
    static constexpr CRGB red = CRGB::Red;
    static constexpr CRGB green = CRGB::Green;

    LEDMatrix();
    void begin(uint8_t brightness = 64);
    void setLED(int x, int y, const CRGB& color);
    void setAll(const CRGB& color);
    void show();
    void clear();

    // Optional: direct access for advanced use
    CRGB* getLeds();

private:
    CRGB leds[WIDTH * HEIGHT];
};

#endif // LEDMATRIX_H
