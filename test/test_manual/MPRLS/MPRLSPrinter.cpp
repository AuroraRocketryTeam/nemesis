#include "MPRLSPrinter.hpp"

MPRLSPrinter::MPRLSPrinter(ISensor* sensor) : mprls(sensor) {}

void MPRLSPrinter::displayPressure() {
    auto data = mprls->getData();

    if(!data.has_value()) {
        Serial.println("ERROR: COULD NOT GET MPRLS SENSOR DATA");
        return;
    }

    auto optPressureValue = data.value().getData("pressure");

    if(!optPressureValue.has_value()) {
        Serial.println("ERROR: THE SENSOR DATAMAP HAD NO ENTRY FOR THE KEY PRESSURE");
        return;
    }

    auto pressureVariant = optPressureValue.value();

    float pressure = std::get<float>(pressureVariant);
    Serial.println("Pressure:" + String(pressure, 4));
}