#include <Wire.h>
#include "test_manual/BNO055/BNO055Printer.hpp"
#include "test_manual/MPRLS/MPRLSPrinter.hpp"
#include <ISensor.hpp>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include <config.h>
#include <pins.h>

ISensor* bno;
ISensor* mprls;
BNO055Printer* bnoPrinter;
MPRLSPrinter* mprlsPrinter;

void tcaSelect(uint8_t bus);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        delay(100);
    }
    Wire.begin();

    bno = new BNO055Sensor();
    Serial.println(F("Initializing BNO055"));
    while (!bno->init()) {
        Serial.println(F("Could not initialize BNO055"));
    }
    Serial.println(F("BNO055 initialised"));
    bnoPrinter = new BNO055Printer(bno);

    tcaSelect(I2C_MULTIPLEXER_MPRLS1);
    mprls = new MPRLSSensor();
    Serial.println(F("Initialize MPRLS"));
    while (!bno->init()) {
        Serial.println(F("Could not initialize MPRLS"));
    }
    Serial.println(F("MPRLS initialised"));
    mprlsPrinter = new MPRLSPrinter(mprls);
}

void loop() {
    bnoPrinter->displayCalibrationStatus();
    
    bnoPrinter->displayMagnetometer();

    bnoPrinter->displayAccelleration();

    bnoPrinter->displayOrientation();

    bnoPrinter->displayGyroscope();

    mprlsPrinter->displayPressure();

    delay(1000); 
}

// Function to select the TCA9548A multiplexer bus
void tcaSelect(uint8_t bus)
{
    Wire.beginTransmission(0x70); // TCA9548A address
    Wire.write(1 << bus);         // send byte to select bus
    Wire.endTransmission();
}
