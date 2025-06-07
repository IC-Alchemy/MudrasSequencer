#include "ledMatrix.h"

const CRGB CRGB::Black = CRGB(0, 0, 0);
const CRGB CRGB::Blue  = CRGB(0, 0, 255);
const CRGB CRGB::Red   = CRGB(255, 0, 0);
const CRGB CRGB::Green = CRGB(0, 255, 0);

const CRGB LEDMatrix::blue  = CRGB::Blue;
const CRGB LEDMatrix::red   = CRGB::Red;
const CRGB LEDMatrix::green = CRGB::Green;

LEDMatrix::LEDMatrix()
    : strip(WIDTH * HEIGHT, DATA_PIN, NEO_GRB + NEO_KHZ800)
{
    // No need to clear here; handled in begin()
}

void LEDMatrix::begin(uint8_t brightness) {
    strip.begin();
    strip.setBrightness(brightness);
    clear();
    show();
}

void LEDMatrix::setPixelColorInternal(int idx, const CRGB& color) {
    strip.setPixelColor(idx, strip.Color(color.r, color.g, color.b));
}

void LEDMatrix::setLED(int x, int y, const CRGB& color) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    int idx = x + y * WIDTH;
    setPixelColorInternal(idx, color);
}

void LEDMatrix::setAll(const CRGB& color) {
    for (int i = 0; i < WIDTH * HEIGHT; ++i) {
        setPixelColorInternal(i, color);
    }
}

void LEDMatrix::show() {
    strip.show();
}

void LEDMatrix::clear() {
    setAll(CRGB::Black);
}

CRGB* LEDMatrix::getLeds() {
    // Not supported with NeoPixel, return nullptr for compatibility
    return nullptr;
}