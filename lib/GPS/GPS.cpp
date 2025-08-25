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
    myGNSS.checkUblox();

    // Add debug information
    int32_t lat = myGNSS.getLatitude();
    int32_t lon = myGNSS.getLongitude();
    uint8_t fixType = myGNSS.getFixType();
    uint8_t satellites = myGNSS.getSIV();

    // Check for valid fix (fix type 2 = 2D, 3 = 3D)
    if (fixType >= 2 && lat != 0 && lon != 0) {
        SensorData data("GPS");
        data.setData("latitude", lat / 10000000.0);
        data.setData("longitude", lon / 10000000.0);
        double altitude_ellipsoid = myGNSS.getAltitude() / 1000.0;
        data.setData("altitude", altitude_ellipsoid);
        // Remove duplicate altitude line
        data.setData("speed", myGNSS.getGroundSpeed() / 1000.0 * 3.6);
        data.setData("satellites", satellites);
        data.setData("hdop", myGNSS.getHorizontalDOP() / 100.0);
        data.setData("fixType", fixType);

        return data;
    }

    // For debugging: you might want to log why it failed
    // Serial.printf("GPS: No fix - Type: %d, Sats: %d, Lat: %d, Lon: %d\n", 
    //               fixType, satellites, lat, lon);

    return std::nullopt;
}