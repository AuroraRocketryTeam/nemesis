#include <Wire.h>
#include "sensors/BME680/BME680Sensor.h"
#include "sensors/BNO055/BNO055Sensor.h"

BME680Sensor bme680;
BME680Sensor bme680_2;
BNO055Sensor bno055;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("BME680 & BNO055 test");
    bme680.init(BME680_I2C_ADDR_1);
    bme680_2.init(BME680_I2C_ADDR_2);
    bno055.init();
}

void loop()
{
    bme680.readData();
    Serial.println("################# BME680 ################# ");
    Serial.println(bme680.getData().toString());
    bme680_2.readData();
    Serial.println("################# BME680_2 #################");
    Serial.println(bme680_2.getData().toString());
    bno055.readData();
    Serial.println("################# BNO055 #################");
    Serial.println(bno055.getData().toString());
    delay(1000);
}
