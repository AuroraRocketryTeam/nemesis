#pragma once
#include <Arduino.h>
#include <ISensor.hpp>
#include <config.h>

#define THERMISTOR_PIN A0
#define SERIES_RESISTOR 1000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25.0
#define B_COEFFICIENT 3500

class TermoresistenzeData : public SensorData
{
public:
    TermoresistenzeData() : SensorData("Termoresistenze") {}

    // ADC Value
    int adcValue;

    // Temperature in °C
    double temperature;

    // Metadata
    uint32_t timestamp;

    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["adcValue"] = adcValue;
        sensorDataJson["temperature"] = temperature;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};

class Termoresistenze : public ISensor
{
public:
    Termoresistenze(int pin = THERMISTOR_PIN, 
                   double seriesResistor = SERIES_RESISTOR, 
                   double nominalResistance = NOMINAL_RESISTANCE, 
                   double nominalTemperature = NOMINAL_TEMPERATURE, 
                   double bCoefficient = B_COEFFICIENT);
    bool init() override;
    std::optional<SensorData> getData() override;

private:
    int thermistorPin;
    double seriesResistor;
    double nominalResistance;
    double nominalTemperature;
    double bCoefficient;
    
    double calculateTemperature(int adcValue);
};