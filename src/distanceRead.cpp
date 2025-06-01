#include "distanceRead.h"
#include <Wire.h>
#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3

static Adafruit_VL53L1X vl53(XSHUT_PIN, IRQ_PIN);
static bool sensorInitialized = false;

bool distanceRead_init() {
    Wire.begin();
    if (!vl53.begin(0x29, &Wire)) {
        Serial.print(F("Error on init of VL53L1X sensor: "));
        Serial.println(vl53.vl_status);
        sensorInitialized = false;
        return false;
    }
    Serial.println(F("VL53L1X sensor OK!"));

    if (!vl53.startRanging()) {
        Serial.print(F("Couldn't start ranging: "));
        Serial.println(vl53.vl_status);
        sensorInitialized = false;
        return false;
    }
    Serial.println(F("Ranging started"));

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    vl53.setTimingBudget(15);
    delay(750);

    Serial.print(F("Timing budget (ms): "));
    Serial.println(vl53.getTimingBudget());
    vl53.VL53L1X_SetDistanceMode(2);

    sensorInitialized = true;
    return true;
}


bool distanceRead_dataReady() {
    if (!sensorInitialized) return false;
    return vl53.dataReady();
}

int16_t distanceRead_getDistance() {
    if (!sensorInitialized) return -1;
    if (!vl53.dataReady()) return -1;

    int16_t distance = vl53.distance();
    if (distance == -1) {
        Serial.print(F("Couldn't get distance: "));
        Serial.println(vl53.vl_status);
        return -1;
    }
    vl53.clearInterrupt();
    return distance;
}