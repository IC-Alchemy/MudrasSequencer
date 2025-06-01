#ifndef DISTANCE_READ_H
#define DISTANCE_READ_H

#include <cstdint>

// Initialize the VL53L1X sensor
bool distanceRead_init();

// Calibrate the sensor's crosstalk value
bool distanceRead_calibrate();

// Check if new distance data is ready
bool distanceRead_dataReady();

// Get the latest distance measurement in mm
int16_t distanceRead_getDistance();

#endif // DISTANCE_READ_H