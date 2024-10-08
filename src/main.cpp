#include <Wire.h>
#include "sensors/BME680/BME680Sensor.h"

BME680Sensor bme680;
BME680Sensor bme680_2;
void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println(F("BME680 test"));
    bme680.init(BME680_I2C_ADDR_1);
    bme680_2.init(BME680_I2C_ADDR_2);
}

void loop()
{
    bme680.readData();
    Serial.println("BME680: ");
    Serial.print(bme680.getData().toString());
    bme680_2.readData();
    Serial.println("BME680_2: ");
    Serial.print(bme680_2.getData().toString());
    delay(2000);
}
