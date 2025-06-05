#ifndef LEDMATRIX_H
#define LEDMATRIX_H

#include <Arduino.h>
#include <FastLED.h>

/**
 * @class LEDMatrix
 * @brief Manages an LED matrix display using the FastLED library.
 * Assumes a WS2812B-style LED matrix connected via a single data pin.
 */
class LEDMatrix {
public:
    static constexpr uint8_t WIDTH = 16;     ///< Width of the LED matrix in pixels.
    static constexpr uint8_t HEIGHT = 8;     ///< Height of the LED matrix in pixels.
    static constexpr uint8_t DATA_PIN = 0;   ///< Data pin for FastLED communication (e.g., for WS2812B).
    // CLOCK_PIN, LATCH_PIN, OE_PIN are defined but may not be used by current WS2812B implementation.
    static constexpr uint8_t CLOCK_PIN = 14; ///< Clock pin, potentially for other matrix types.
    static constexpr uint8_t LATCH_PIN = 13; ///< Latch pin, potentially for other matrix types.
    static constexpr uint8_t OE_PIN = 12;    ///< Output Enable pin, potentially for other matrix types.

    // Color aliases for convenience
    static constexpr CRGB blue = CRGB::Blue;   ///< Alias for CRGB::Blue.
    static constexpr CRGB red = CRGB::Red;     ///< Alias for CRGB::Red.
    static constexpr CRGB green = CRGB::Green; ///< Alias for CRGB::Green.

    /** @brief Default constructor. Initializes LED buffer to black. */
    LEDMatrix();

    /**
     * @brief Initializes the FastLED library and sets the initial brightness.
     * @param brightness Initial brightness level for the LEDs (0-255). Default is 64.
     */
    void begin(uint8_t brightness = 64);

    /**
     * @brief Sets the color of a specific LED in the matrix.
     * @param x The x-coordinate (column) of the LED.
     * @param y The y-coordinate (row) of the LED.
     * @param color The CRGB color to set the LED to.
     */
    void setLED(int x, int y, const CRGB& color);

    /**
     * @brief Sets all LEDs in the matrix to the same color.
     * @param color The CRGB color to set all LEDs to.
     */
    void setAll(const CRGB& color);

    /** @brief Sends the current LED data to the matrix to update the display. */
    void show();

    /** @brief Clears the matrix by setting all LEDs to black and then calls show(). */
    void clear();

    /**
     * @brief Provides direct access to the underlying LED buffer.
     * Use with caution, as direct manipulation bypasses class methods.
     * @return Pointer to the CRGB array representing the LED states.
     */
    CRGB* getLeds();

private:
    CRGB leds[WIDTH * HEIGHT]; ///< Internal buffer storing the state of each LED.
};

#endif // LEDMATRIX_H
