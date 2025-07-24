#include "GPS.hpp"

GPSSensor::GPSSensor()
{
    gps = TinyGPSPlus();
}


bool GPSSensor::init() {
    Serial1.begin(9600); // !!! Check if the boud is ok
    return true;
}

std::optional<SensorData> GPSSensor::getData() {
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    if (gps.location.isValid() && gps.location.isUpdated()) {
        SensorData data("GPS");

        data.setData("latitude", gps.location.lat());
        data.setData("longitude", gps.location.lng());
        data.setData("altitude", gps.altitude.meters());
        data.setData("speed", gps.speed.kmph());
        data.setData("satellites", gps.satellites.value());
        data.setData("hdop", gps.hdop.value());

        return data;
    }

    return std::nullopt;
}