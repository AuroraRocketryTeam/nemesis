#include "GPS.hpp"

GPS::GPS() {}

bool GPS::init() {
    Wire.begin();
    if (!myGNSS.begin(Wire)) {
        return false;
    }
    // Optionally configure GNSS settings here
    return true;
}

std::optional<SensorData> GPS::getData() {
    if (!myGNSS.checkUblox()) {
        return std::nullopt;
    }

    if (myGNSS.getLatitude() != 0 && myGNSS.getLongitude() != 0) {
        SensorData data("GPS");
        data.setData("latitude", myGNSS.getLatitude() / 10000000.0);   // degrees
        data.setData("longitude", myGNSS.getLongitude() / 10000000.0); // degrees
        double altitude_ellipsoid = myGNSS.getAltitude() / 1000.0;
        //double geoid_separation = myGNSS.getGeoidSeparation() / 100.0; // often in centimeters
        //double altitude_msl = altitude_ellipsoid - geoid_separation;
        data.setData("altitude", altitude_ellipsoid);
        data.setData("altitude", myGNSS.getAltitude() / 1000.0);       // meters
        data.setData("speed", myGNSS.getGroundSpeed() / 1000.0 * 3.6); // km/h
        data.setData("satellites", myGNSS.getSIV());
        data.setData("hdop", myGNSS.getHorizontalDOP() / 100.0);

        return data;
    }

    return std::nullopt;
}