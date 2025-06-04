#include "ledMatrix.h"

LEDMatrix::LEDMatrix() {
    clear();
}

void LEDMatrix::begin(uint8_t brightness) {
    // FastLED setup for parallel or SPI-based matrix
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, WIDTH * HEIGHT);
    FastLED.setBrightness(brightness);
    clear();
    show();
}

void LEDMatrix::setLED(int x, int y, const CRGB& color) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    leds[x + y * WIDTH] = color;
}

void LEDMatrix::setAll(const CRGB& color) {
    for (int i = 0; i < WIDTH * HEIGHT; ++i) {
        leds[i] = color;
    }
}

void LEDMatrix::show() {
    FastLED.show();
}

void LEDMatrix::clear() {
    setAll(CRGB::Black);
}

CRGB* LEDMatrix::getLeds() {
    return leds;
}