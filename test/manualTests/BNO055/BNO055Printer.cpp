#include "BNO055Printer.hpp"
#include "utils/PrintUtils.hpp"

BNO055Printer::BNO055Printer(Adafruit_BNO055& sensor) : bno055(sensor) { }

void BNO055Printer::displayCalibrationStatus() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno055.getCalibration(&system, &gyro, &accel, &mag);

  PrintUtils::printHeader("CALIBRATION STATUS: 0=not calibrated, 3=fully calibrated");

  Serial.println("Sys: " + String(system));
  Serial.println("Gyro: " + String(gyro));
  Serial.println("Accel: " + String(accel));
  Serial.println("Mag: " + String(mag));

  Serial.println("");
}

void BNO055Printer::displaySensorDetails() {
    sensor_t sensor;
    bno055.getSensor(&sensor);

    PrintUtils::printHeader("SENSOR DETAILS");

    Serial.println("Sensor name: " + String(sensor.name));
    Serial.println("Driver version: " + String(sensor.version));
    Serial.println("ID: " + String(sensor.sensor_id));
    Serial.println("Min delay (ms): " + String(sensor.min_delay));

    Serial.println("");
}

void BNO055Printer::displayMagnetometer() {
    imu::Vector<3> mag = bno055.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    PrintUtils::printHeader("MAGNETOMETER");

    Serial.print("X: " + String(mag.x(), DECIMAL_PLACES));
    Serial.print(" Y: " + String(mag.y(), DECIMAL_PLACES));
    Serial.print(" Z: " + String(mag.z(), DECIMAL_PLACES));
    Serial.println(" uT");

    Serial.println("");
}

void BNO055Printer::displayOrientation() {
    sensors_event_t event;
    bno055.getEvent(&event);

    PrintUtils::printHeader("ORIENTATION");

    Serial.println("X: " + String(event.orientation.x, DECIMAL_PLACES));
    Serial.println("Y: " + String(event.orientation.y, DECIMAL_PLACES));
    Serial.println("Z: " + String(event.orientation.z, DECIMAL_PLACES));

    Serial.println("");
}

void BNO055Printer::displayAccelleration() {
    imu::Vector<3> accel = bno055.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    PrintUtils::printHeader("ACCELLERATION");

    Serial.println("X: " + String(accel.x(), DECIMAL_PLACES) + " m/s²");
    Serial.println("Y: " + String(accel.y(), DECIMAL_PLACES) + " m/s²");
    Serial.println("Z: " + String(accel.z(), DECIMAL_PLACES) + " m/s²");

    Serial.println("");
}

void BNO055Printer::displayGyroscope() {
    imu::Vector<3> gyro = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    PrintUtils::printHeader("GYROSCOPE");

    Serial.println("X: " + String(gyro.x(), DECIMAL_PLACES) + " rad/s");
    Serial.println("Y: " + String(gyro.y(), DECIMAL_PLACES) + " rad/s");
    Serial.println("Z: " + String(gyro.z(), DECIMAL_PLACES) + " rad/s");

    Serial.println("");
}
