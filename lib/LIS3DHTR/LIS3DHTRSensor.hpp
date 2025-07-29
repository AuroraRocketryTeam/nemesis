#pragma once
#include <ISensor.hpp>
#include <LIS3DHTR.h>
#include <Wire.h>
#include <config.h>

class LIS3DHTRSensor : public ISensor
{
public:
    LIS3DHTRSensor();
    bool init() override;
    std::optional<SensorData> getData() override;

private:
    LIS3DHTR<TwoWire> lis;
};