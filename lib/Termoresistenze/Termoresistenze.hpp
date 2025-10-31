#pragma once
#include <Arduino.h>
#include <ISensor.hpp>
#include <config.h>

#define THERMISTOR_PIN A0
#define SERIES_RESISTOR 1000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25.0
#define B_COEFFICIENT 3500

/**
 * @brief Data structure for Termoresistenze sensor readings
 * 
 */
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

/**
 * @brief Termoresistenze sensor class
 * 
 */
class Termoresistenze : public ISensor
{
public:
    /**
     * @brief Construct a new Termoresistenze object
     * 
     * @param pin The analog pin connected to the thermistor
     * @param seriesResistor The value of the series resistor
     * @param nominalResistance The nominal resistance of the thermistor
     * @param nominalTemperature The nominal room temperature
     * @param bCoefficient The B coefficient of the thermistor
     */
    Termoresistenze(int pin = THERMISTOR_PIN, 
                   double seriesResistor = SERIES_RESISTOR, 
                   double nominalResistance = NOMINAL_RESISTANCE, 
                   double nominalTemperature = NOMINAL_TEMPERATURE, 
                   double bCoefficient = B_COEFFICIENT);
    bool init() override;
    bool updateData() override;

    /**
     * @brief Getter for the sensor data
     * 
     * @return a shared pointer to the TermoresistenzeData structure containing the latest readings
     */
    std::shared_ptr<TermoresistenzeData> getData();

private:
    int _thermistorPin;
    double _seriesResistor;
    double _nominalResistance;
    double _nominalTemperature;
    double _bCoefficient;

    std::shared_ptr<TermoresistenzeData> _data;

    double calculateTemperature(int adcValue);
};