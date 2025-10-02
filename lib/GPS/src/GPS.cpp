#include "GPS.hpp"

GPS::GPS()
{
    this->myGNSS = SFE_UBLOX_GNSS();
}

bool GPS::init()
{
    Wire.begin();
    if (!myGNSS.begin(Wire))
    {
        return false;
    }
    return true;
}

std::optional<SensorData> GPS::getData()
{
    uint8_t fixType = myGNSS.getFixType();
    uint8_t satellites = myGNSS.getSIV();
    SensorData data("GPS");
    data.setData("satellites", satellites);
    data.setData("fix", fixType);

    // Check for valid fix (fix type 2 = 2D, 3 = 3D)
    if (fixType >= 3)
    {
        int32_t lat = myGNSS.getLatitude();
        int32_t lon = myGNSS.getLongitude();
        data.setData("latitude", lat / 10000000.0);         // Check correct conversion formula
        data.setData("longitude", lon / 10000000.0);
        double altitude_ellipsoid = myGNSS.getAltitude() / 1000.0;      // Check correct conversion formula
        data.setData("altitude", altitude_ellipsoid);
        data.setData("speed", myGNSS.getGroundSpeed() / 1000.0 * 3.6);  // Check correct conversion formula
        data.setData("hdop", myGNSS.getHorizontalDOP() / 100.0);
    }

    return data;
}