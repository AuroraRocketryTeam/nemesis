#include <Wire.h>
#include "sensors/BME680/BME680Sensor.h"

BME680Sensor bme680;

void setup()
{
    Serial.begin(115200);
    while (!Serial){;}
    Serial.println(F("BME680 test"));
    bme680.init();
}

void loop()
{
    Serial.print(bme680.getData().toString());
    delay(2000);
}
