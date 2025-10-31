#include "Termoresistenze.hpp"
#include <Arduino.h>
#include <math.h>

Termoresistenze::Termoresistenze(int pin, 
                                double seriesRes, 
                                double nominalRes, 
                                double nominalTemp, 
                                double bCoeff)
    : _thermistorPin(pin)
    , _seriesResistor(seriesRes)
    , _nominalResistance(nominalRes)
    , _nominalTemperature(nominalTemp)
    , _bCoefficient(bCoeff)
{
}

bool Termoresistenze::init()
{
    pinMode(_thermistorPin, INPUT);
    return true;
}

bool Termoresistenze::updateData()
{
    _data = std::make_shared<TermoresistenzeData>();

    _data->adcValue = analogRead(_thermistorPin);

    if (_data->adcValue == 0) {
        return false;
    }

    _data->temperature = calculateTemperature(_data->adcValue);

    return true;
}

std::shared_ptr<TermoresistenzeData> Termoresistenze::getData()
{
    return _data;
}

double Termoresistenze::calculateTemperature(int adcValue)
{
    // Calculate thermistor resistance
    double resistance = _seriesResistor * (4095.0 / adcValue - 1.0);

    // Steinhart-Hart equation (simplified Beta model)
    double steinhart;
    steinhart = resistance / _nominalResistance;      // (R/R0)
    steinhart = log(steinhart);                      // ln(R/R0)
    steinhart /= _bCoefficient;                       // 1/B * ln(R/R0)
    steinhart += 1.0 / (_nominalTemperature + 273.15); // + (1/T0)
    steinhart = 1.0 / steinhart;                     // Invert
    steinhart -= 273.15;                             // Convert to °C

    return steinhart;
}