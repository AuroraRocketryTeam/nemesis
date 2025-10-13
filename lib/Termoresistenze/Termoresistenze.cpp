#include "Termoresistenze.hpp"
#include <Arduino.h>
#include <math.h>

Termoresistenze::Termoresistenze(int pin, 
                                double seriesRes, 
                                double nominalRes, 
                                double nominalTemp, 
                                double bCoeff)
    : thermistorPin(pin)
    , seriesResistor(seriesRes)
    , nominalResistance(nominalRes)
    , nominalTemperature(nominalTemp)
    , bCoefficient(bCoeff)
{
}

bool Termoresistenze::init()
{
    pinMode(thermistorPin, INPUT);
    return true;
}

std::optional<SensorData> Termoresistenze::getData()
{
    int adcValue = analogRead(thermistorPin);
    
    if (adcValue == 0) {
        return std::nullopt;
    }
    
    double temperature = calculateTemperature(adcValue);
    
    SensorData data = SensorData("Termoresistenze");
    data.setData("Temperature", temperature);
    data.setData("ADC_Value", adcValue);
    
    return data;
}

double Termoresistenze::calculateTemperature(int adcValue)
{
    // Calculate thermistor resistance
    double resistance = seriesResistor * (4095.0 / adcValue - 1.0);

    // Steinhart-Hart equation (simplified Beta model)
    double steinhart;
    steinhart = resistance / nominalResistance;      // (R/R0)
    steinhart = log(steinhart);                      // ln(R/R0)
    steinhart /= bCoefficient;                       // 1/B * ln(R/R0)
    steinhart += 1.0 / (nominalTemperature + 273.15); // + (1/T0)
    steinhart = 1.0 / steinhart;                     // Invert
    steinhart -= 273.15;                             // Convert to °C

    return steinhart;
}