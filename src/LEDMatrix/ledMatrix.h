#ifndef LEDMATRIX_H
#define LEDMATRIX_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Forward declaration for CRGB compatibility
struct CRGB {
    uint8_t r, g, b;
    CRGB() : r(0), g(0), b(0) {}
    CRGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
    static const CRGB Black;
    static const CRGB Blue;
    static const CRGB Red;
    static const CRGB Green;
    operator uint32_t() const { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; }
};

class LEDMatrix {
public:
    static constexpr uint8_t WIDTH = 16;
    static constexpr uint8_t HEIGHT = 8;
    static constexpr uint8_t DATA_PIN = 15;

    // Color aliases for convenience (using CRGB for API compatibility)
    static const CRGB blue;
    static const CRGB red;
    static const CRGB green;

    LEDMatrix();
    void begin(uint8_t brightness = 64);
    void setLED(int x, int y, const CRGB& color);
    void setAll(const CRGB& color);
    void show();
    void clear();

    // Optional: direct access for advanced use (returns nullptr for compatibility)
    CRGB* getLeds();

private:
    Adafruit_NeoPixel strip;
    void setPixelColorInternal(int idx, const CRGB& color);
};


#endif // LEDMATRIX_H
