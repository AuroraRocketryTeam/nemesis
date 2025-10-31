#include "LIS3DHTRSensor.hpp"

LIS3DHTRSensor::LIS3DHTRSensor() {}

bool LIS3DHTRSensor::init()
{
    _lis.begin(Wire, 0x18); // Try also withouth 0x18
    
    if (!_lis.isConnection())
    {
        return false;
    }
    
    _lis.setFullScaleRange(LIS3DHTR_RANGE_2G);
    _lis.setOutputDataRate(LIS3DHTR_DATARATE_50HZ); // Could be 100Hz, 200Hz, 1_6KHZ, 5KHZ, it was 25HZ
    _lis.setHighSolution(true);
    
    return true;
}

bool LIS3DHTRSensor::updateData()
{
    if (!_lis.isConnection())
    {
        return false;
    }

    _data = std::make_shared<LIS3DHTRData>();
    
    // Read acceleration data
    _data->acceleration_x = _lis.getAccelerationX() * GRAVITY;
    _data->acceleration_y = _lis.getAccelerationY() * GRAVITY;
    _data->acceleration_z = _lis.getAccelerationZ() * GRAVITY;
    
    // Update timestamp
    _data->timestamp = millis();

    return true;
}

std::shared_ptr<LIS3DHTRData> LIS3DHTRSensor::getData()
{
    return _data;
}