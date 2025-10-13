/*
#include <vector>
#include <tuple>
#include <config.h>
#include <BNO055Sensor.hpp>
 
static BNO055Sensor bno;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Initializing BNO055...");
    
    if (!bno.init()) {
        Serial.println("Failed to initialize BNO055 - continuing with error messages");
    } else {
        Serial.println("BNO055 initialized successfully");
    }
}
 
void loop()
{
    Serial.println("Reading BNO055 data...");
    auto bnoDataOpt = bno.getData();
    if (!bnoDataOpt.has_value()) {
        Serial.println("BNO055 data not available - sensor not initialized or failed");
    } else {
        Serial.println("BNO055 data retrieved successfully");
    }
    auto bnoData = bnoDataOpt.value();

    float bno_gravity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["x"]);
    float bno_gravity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["y"]);
    float bno_gravity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["z"]);

    float bno_gravity_xyz = sqrt(pow(bno_gravity_x, 2) + pow(bno_gravity_y, 2) + pow(bno_gravity_z, 2));

    Serial.println("Gravity x: " + String(bno_gravity_x) + 
                   ", y: " + String(bno_gravity_y) + 
                   ", z: " + String(bno_gravity_z) +
                   ", xyz: " + String(bno_gravity_xyz));
    
    delay(100);
}*/