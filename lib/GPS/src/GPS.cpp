#include "GPS.hpp"

GPS::GPS()
{
    _myGNSS = SFE_UBLOX_GNSS();
}

bool GPS::init()
{
    Wire.begin();
    if (!_myGNSS.begin(Wire))
    {
        return false;
    }
    return true;
}

bool GPS::updateData()
{
    _data = std::make_shared<GPSData>();

    _data->satellites = _myGNSS.getFixType();
    _data->fixType = _myGNSS.getSIV();

    // Check for valid fix (fix type 2 = 2D, 3 = 3D)
    if (_data->fixType >= 3)
    {
        _data->latitude = _myGNSS.getLatitude() / 10000000.0;
        _data->longitude = _myGNSS.getLongitude() / 10000000.0;
        _data->altitude = _myGNSS.getAltitude() / 1000.0;
        _data->ground_speed = _myGNSS.getGroundSpeed() / 1000.0 * 3.6;
        _data->hdop = _myGNSS.getHorizontalDOP() / 100.0;

        _data->timestamp = millis();
    } else {
        return false;
    }

    return true;
}

std::shared_ptr<GPSData> GPS::getData()
{
    return _data;
}