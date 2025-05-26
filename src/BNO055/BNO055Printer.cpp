#include "BNO055Printer.hpp"
#include "../utilscopy/PrintUtils.hpp"

BNO055Printer::BNO055Printer(ISensor* sensor) : bno055(sensor) { }

void BNO055Printer::displayCalibrationStatus() {
    auto data = bno055->getData();
    if (!data.has_value()) {
        Serial.println("ERROR: COULD NOT GET SENSOR DATA");
        return;
    }
    if (!data.value().getData("system_calibration").has_value() ||
        !data.value().getData("gyro_calibration").has_value() ||
        !data.value().getData("accel_calibration").has_value() ||
        !data.value().getData("mag_calibration").has_value()
    ) {
        Serial.println("ERROR: Could not read all calibration status variables");
        return;
    }
    uint8_t system = std::get<uint8_t>(data.value().getData("system_calibration").value()),
        gyro = std::get<uint8_t>(data.value().getData("gyro_calibration").value()),
        accel = std::get<uint8_t>(data.value().getData("accel_calibration").value()),
        mag = std::get<uint8_t>(data.value().getData("mag_calibration").value());

    PrintUtils::printHeader("CALIBRATION STATUS: 0=not calibrated, 3=fully calibrated");
    Serial.println("Sys: " + String(system));
    Serial.println("Gyro: " + String(gyro));
    Serial.println("Accel: " + String(accel));
    Serial.println("Mag: " + String(mag));
    Serial.println("");
}

void BNO055Printer::displayMagnetometer() {
    printXYZMap("magnetometer");
    Serial.println(" uT");
    Serial.println("");
}

void BNO055Printer::displayOrientation() {
    printXYZMap("orientation");
    Serial.println("");
}

void BNO055Printer::displayAccelleration() {
    printXYZMap("accelerometer");
    Serial.println(" m/s²");
    Serial.println("");
}

void BNO055Printer::displayGyroscope() {
    printXYZMap("angular_velocity");
    Serial.println(" rad/s");
    Serial.println("");
}

void BNO055Printer::displayLinearAccelleration() {
    printXYZMap("linear_acceleration");
    Serial.println("");
}

void BNO055Printer::displayGravity() {
    printXYZMap("gravity");
    Serial.println("");
}

void BNO055Printer::printXYZMap(const char* key) {
    auto data = bno055->getData();
    if (!data.has_value()) {
        Serial.println("ERROR: COULD NOT GET SENSOR DATA");
        return;
    }

    auto optVal = data.value().getData(key);
    if (!optVal.has_value()) {
        Serial.println(String("ERROR: Could not read ") + key + " values");
        return;
    }

    auto val = std::get<std::map<std::string, float>>(data.value().getData(key).value());
    PrintUtils::printHeader(key);

    auto x = val.find("x"), y = val.find("y"), z = val.find("z");
    if (x == val.end() || y == val.end() || z == val.end()) {
        Serial.println(String("ERROR: Could not find all the") + key + "values");
    }
    Serial.print("X: " + String(x->second, DECIMAL_PLACES));
    Serial.print(" Y: " + String(y->second, DECIMAL_PLACES));
    Serial.print(" Z: " + String(z->second, DECIMAL_PLACES));    
}
