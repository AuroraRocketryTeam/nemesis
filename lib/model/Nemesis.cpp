#include "Nemesis.hpp"

//  Actuator and Buzzer pins
#define ADC_PIN A0
#define MAIN_ACTUATORS_PIN D0
#define DROGUE_ACTUATORS_PIN D1
#define BUZZER_PIN D2

Nemesis::Nemesis(std::shared_ptr<RocketLogger> logger,
            std::shared_ptr<BNO055Sensor> bno,
            std::shared_ptr<LIS3DHTRSensor> lis3dh,
            std::shared_ptr<MS561101BA03> ms56_1,
            std::shared_ptr<MS561101BA03> ms56_2,
            std::shared_ptr<GPS> gps) :
    _logger(logger),
    _bno(bno),
    _lis3dh(lis3dh),
    _ms56_1(ms56_1),
    _ms56_2(ms56_2),
    _gps(gps),
    _isRising(std::make_shared<bool>(false)),
    _heightGainSpeed(std::make_shared<float>(0.0f)),
    _currentHeight(std::make_shared<float>(0.0f))
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setRxBufferSize(2048);

    pinMode(MAIN_ACTUATORS_PIN, OUTPUT);
    pinMode(DROGUE_ACTUATORS_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(MAIN_ACTUATORS_PIN, LOW);
    digitalWrite(DROGUE_ACTUATORS_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
}

void Nemesis::readBattery() {
    _batteryAdc = analogRead(ADC_PIN);
    _batteryVoltage = ((_batteryAdc / 4095.0f) * 3.3f) * 2.0f;
    _batteryPercentage = (_batteryVoltage - 5.6f) / (7.2f - 5.6f) * 100.0f;
    std::string voltageStr = (String(_batteryPercentage, 1) + "%").c_str();

    //auto voltageData = SensorData("Voltage");
    //voltageData.setData("ADC_Value", _batteryAdc);
    //voltageData.setData("Voltage", _batteryVoltage);
    //voltageData.setData("Percentage", _batteryPercentage);
    
    //logger->logSensorData(voltageData);
}

bool Nemesis::updateBNO055() {
    bool result = _bno->updateData();

    _bnoData = _bno->getData();
    _logger->logSensorData(_bnoData);

    return result;
}

bool Nemesis::updateLIS3DHTR() {
    bool result = _lis3dh->updateData();

    _lis3dhData = _lis3dh->getData();
    _logger->logSensorData(_lis3dhData);

    return result;
}

bool Nemesis::updateMS561101BA03_1() {
    bool result = _ms56_1->updateData();

    _ms561101ba03Data_1 = _ms56_1->getData();
    _logger->logSensorData(_ms561101ba03Data_1);

    return result;
}

bool Nemesis::updateMS561101BA03_2() {
    bool result = _ms56_2->updateData();

    _ms561101ba03Data_2 = _ms56_2->getData();
    _logger->logSensorData(_ms561101ba03Data_2);

    return result;
}

bool Nemesis::updateGPS() {
    bool result = _gps->updateData();

    _gpsData = _gps->getData();
    _logger->logSensorData(_gpsData);

    return result;
}

std::shared_ptr<BNO055Data> Nemesis::getBNO055Data() {
    return _bnoData;
}

std::shared_ptr<LIS3DHTRData> Nemesis::getLIS3DHTRData() {
    return _lis3dhData;
}

std::shared_ptr<MS561101BA03Data> Nemesis::getMS561101BA03Data_1() {
    return _ms561101ba03Data_1;
}

std::shared_ptr<MS561101BA03Data> Nemesis::getMS561101BA03Data_2() {
    return _ms561101ba03Data_2;
}

std::shared_ptr<GPSData> Nemesis::getGPSData() {
    return _gpsData;
}

void Nemesis::setSimulatedBNO055Data(std::shared_ptr<BNO055Data> data) {
    _bnoData = data;
}

void Nemesis::setSimulatedLIS3DHTRData(std::shared_ptr<LIS3DHTRData> data) {
    _lis3dhData = data;
}

void Nemesis::setSimulatedMS561101BA03Data_1(std::shared_ptr<MS561101BA03Data> data) {
    _ms561101ba03Data_1 = data;
}

void Nemesis::setSimulatedMS561101BA03Data_2(std::shared_ptr<MS561101BA03Data> data) {
    _ms561101ba03Data_2 = data;
}

void Nemesis::setSimulatedGPSData(std::shared_ptr<GPSData> data) {
    _gpsData = data;
}

std::shared_ptr<bool> Nemesis::getIsRising() {
    return _isRising;
}

std::shared_ptr<float> Nemesis::getHeightGainSpeed() {
    return _heightGainSpeed;
}

std::shared_ptr<float> Nemesis::getCurrentHeight() {
    return _currentHeight;
}