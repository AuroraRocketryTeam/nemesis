#include "MS561101BA03.hpp"
#include <Arduino.h>

MS561101BA03::MS561101BA03(uint8_t address) : _address(address)
{
    memset(_calibrationData, 0, sizeof(_calibrationData));
}

bool MS561101BA03::init()
{
    Wire.begin();
    
    // Reset the sensor
    reset();
    delay(10);
    
    // Read calibration data
    if (!readCalibrationData()) {
        return false;
    }
    
    return true;
}

bool MS561101BA03::updateData()
{
    // Read raw pressure and temperature
    uint32_t D1 = readRawPressure();
    uint32_t D2 = readRawTemperature();
    
    if (D1 == 0 || D2 == 0) {
        return false;
    }
    
    _data = std::make_shared<MS561101BA03Data>();

    // Calculate compensated pressure and temperature
    calculatePressureAndTemperature(D1, D2, _data->pressure, _data->temperature);    

    _data->timestamp = millis();

    return true;
}

void MS561101BA03::reset()
{
    writeCommand(MS5611_CMD_RESET);
}

bool MS561101BA03::readCalibrationData()
{
    for (uint8_t i = 0; i < 8; i++) {
        _calibrationData[i] = readPROM(MS5611_CMD_PROM_READ + (i * 2));
        if (i > 0 && _calibrationData[i] == 0) {
            return false; // Invalid calibration data
        }
    }
    return true;
}

uint16_t MS561101BA03::readPROM(uint8_t address)
{
    Wire.beginTransmission(_address);
    Wire.write(address);
    Wire.endTransmission();
    
    Wire.requestFrom(_address, (uint8_t)2);
    
    if (Wire.available() >= 2) {
        uint16_t result = Wire.read() << 8;
        result |= Wire.read();
        return result;
    }
    
    return 0;
}

void MS561101BA03::writeCommand(uint8_t command)
{
    Wire.beginTransmission(_address);
    Wire.write(command);
    Wire.endTransmission();
}

uint32_t MS561101BA03::readADC()
{
    writeCommand(MS5611_CMD_ADC_READ);
    
    Wire.requestFrom(_address, (uint8_t)3);
    
    if (Wire.available() >= 3) {
        uint32_t result = (uint32_t)Wire.read() << 16;
        result |= (uint32_t)Wire.read() << 8;
        result |= Wire.read();
        return result;
    }
    
    return 0;
}

uint32_t MS561101BA03::readRawPressure()
{
    writeCommand(MS5611_CMD_CONV_D1_2048);
    delay(10); // Wait for conversion
    return readADC();
}

uint32_t MS561101BA03::readRawTemperature()
{
    writeCommand(MS5611_CMD_CONV_D2_4096);
    delay(10); // Wait for conversion
    return readADC();
}

void MS561101BA03::calculatePressureAndTemperature(uint32_t D1, uint32_t D2, float& pressure, float& temperature)
{
    // Extract calibration coefficients
    uint16_t C1 = _calibrationData[1];
    uint16_t C2 = _calibrationData[2];
    uint16_t C3 = _calibrationData[3];
    uint16_t C4 = _calibrationData[4];
    uint16_t C5 = _calibrationData[5];
    uint16_t C6 = _calibrationData[6];
    
    // Calculate temperature
    int32_t dT = D2 - ((uint32_t)C5 << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * C6) >> 23);
    
    // Calculate pressure
    int64_t OFF = ((int64_t)C2 << 16) + (((int64_t)C4 * dT) >> 7);
    int64_t SENS = ((int64_t)C1 << 15) + (((int64_t)C3 * dT) >> 8);
    
    // Second order temperature compensation
    int32_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    
    if (TEMP < 2000) {
        T2 = (dT * dT) >> 31;
        OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = OFF2 >> 1;
        
        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 = SENS2 + ((11 * (TEMP + 1500) * (TEMP + 1500)) >> 1);
        }
    }
    
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    
    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;
    
    temperature = TEMP / 100.0f;
    pressure = P; // Convert to hPa/mbar
}

std::shared_ptr<MS561101BA03Data> MS561101BA03::getData()
{
    return _data;
}