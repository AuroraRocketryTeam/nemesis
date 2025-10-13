#include "LIS3DHTRSensor.hpp"

LIS3DHTRSensor::LIS3DHTRSensor() {}

bool LIS3DHTRSensor::init()
{
    lis.begin(Wire, 0x18); // Try also withouth 0x19
    
    if (!lis.isConnection())
    {
        return false;
    }
    
    lis.setFullScaleRange(LIS3DHTR_RANGE_2G);
    lis.setOutputDataRate(LIS3DHTR_DATARATE_50HZ); // Could be 100Hz, 200Hz, 1_6KHZ, 5KHZ, it was 25HZ
    lis.setHighSolution(true);
    
    return true;
}

std::optional<SensorData> LIS3DHTRSensor::getData()
{
    if (!lis.isConnection())
    {
        return std::nullopt;
    }
    
    float accelX = lis.getAccelerationX() * GRAVITY;
    float accelY = lis.getAccelerationY() * GRAVITY;
    float accelZ = lis.getAccelerationZ() * GRAVITY;
    float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    
    SensorData data = SensorData("LIS3DHTR");
    data.setData("accel_x", accelX);
    data.setData("accel_y", accelY);
    data.setData("accel_z", accelZ);
    data.setData("accel_magnitude", accelMagnitude);
    
    return data;
}