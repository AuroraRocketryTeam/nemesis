#pragma once
#include <ISensor.hpp>
#include <Wire.h>

// MS5611 Commands
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_256  0x40
#define MS5611_CMD_CONV_D1_512  0x42
#define MS5611_CMD_CONV_D1_1024 0x44
#define MS5611_CMD_CONV_D1_2048 0x46
#define MS5611_CMD_CONV_D1_4096 0x48
#define MS5611_CMD_CONV_D2_256  0x50
#define MS5611_CMD_CONV_D2_512  0x52
#define MS5611_CMD_CONV_D2_1024 0x54
#define MS5611_CMD_CONV_D2_2048 0x56
#define MS5611_CMD_CONV_D2_4096 0x58
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0

class MS561101BA03Data : public SensorData
{
public:
    MS561101BA03Data() : SensorData("MS561101BA03") {}

    // Pressure in mbar
    float pressure;
    
    // Temperature in °C
    float temperature;

    // Metadata
    uint32_t timestamp;

    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["pressure"] = pressure;
        sensorDataJson["temperature"] = temperature;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};

class MS561101BA03 : public ISensor
{
public:
    MS561101BA03(uint8_t address = 0x77);
    bool init() override;
    std::optional<SensorData> getData() override;

private:
    uint8_t _address;
    uint16_t _calibrationData[8];
    
    bool readCalibrationData();
    void reset();
    uint32_t readADC();
    uint16_t readPROM(uint8_t address);
    void writeCommand(uint8_t command);
    uint32_t readRawPressure();
    uint32_t readRawTemperature();
    void calculatePressureAndTemperature(uint32_t D1, uint32_t D2, float& pressure, float& temperature);
    
    float _pressure;
    float _temperature;
};