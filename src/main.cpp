#include <Arduino.h>
#include <Wire.h>
#include "const/pins.h"

#include "sensors/Sensor.h"
#include "sensors/BME680/BME680Sensor.h"

void setup() {
    // Serial port initialization.
    Serial.begin(115200);
}

void loop() {
}
