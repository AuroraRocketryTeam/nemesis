 #pragma once

 #include <Arduino.h>
 #include <BNO055Sensor.hpp>
 #include <MPRLSSensor.hpp>
 #include <LIS3DHTRSensor.hpp>
 #include <MS561101BA03.hpp>
 #include <GPS.hpp>
 #include <Termoresistenze.hpp>
 #include <RocketLogger.hpp>
 #include <config.h>

 /**
  * @brief Class representing the Nemesis rocket model, encapsulating sensors and logger.
  * 
  */
 class Nemesis {
 public:
    /**
    * @brief Construct a new Nemesis object
    * 
    * @param logger Shared pointer to the RocketLogger instance.
    * @param bno Shared pointer to the BNO055Sensor instance.
    * @param lis3dh Shared pointer to the LIS3DHTRSensor instance.
    * @param ms56_1 Shared pointer to the first MS561101BA03 instance.
    * @param ms56_2 Shared pointer to the second MS561101BA03 instance.
    * @param gps Shared pointer to the GPS instance.
    */
    Nemesis(std::shared_ptr<RocketLogger> logger,
            std::shared_ptr<BNO055Sensor> bno,
            std::shared_ptr<LIS3DHTRSensor> lis3dh,
            std::shared_ptr<MS561101BA03> ms56_1,
            std::shared_ptr<MS561101BA03> ms56_2,
            std::shared_ptr<GPS> gps);

    /**
     * @brief Update the BNO055 sensor data
     * 
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateBNO055();

    /**
     * @brief Update the LIS3DHTR sensor data
     * 
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateLIS3DHTR();
    
    /**
     * @brief Update the first MS561101BA03 sensor data
     * 
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateMS561101BA03_1();
    
    /**
     * @brief Update the second MS561101BA03 sensor data
     * 
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateMS561101BA03_2();
    
    /**
     * @brief Update the GPS sensor data
     * 
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateGPS();
    
    /**
     * @brief Read the battery status
     * 
     */
    void readBattery();

    /**
     * @brief Get the BNO055 sensor data
     * 
     * @return A shared pointer to the BNO055Data
     */
    std::shared_ptr<BNO055Data> getBNO055Data();

    /**
     * @brief Get the LIS3DHTR sensor data
     * 
     * @return A shared pointer to the LIS3DHTRData
     */
    std::shared_ptr<LIS3DHTRData> getLIS3DHTRData();

    /**
     * @brief Get the first MS561101BA03 sensor data
     * 
     * @return A shared pointer to the MS561101BA03Data
     */
    std::shared_ptr<MS561101BA03Data> getMS561101BA03Data_1();

    /**
     * @brief Get the second MS561101BA03 sensor data
     * 
     * @return A shared pointer to the MS561101BA03Data
     */
    std::shared_ptr<MS561101BA03Data> getMS561101BA03Data_2();

    /**
     * @brief Get the GPS sensor data
     * 
     * @return A shared pointer to the GPSData
     */
    std::shared_ptr<GPSData> getGPSData();

    /**
     * @brief Set the simulated BNO055 sensor data
     * 
     * @param data A shared pointer to the BNO055Data
     */
    void setSimulatedBNO055Data(std::shared_ptr<BNO055Data> data);

    /**
     * @brief Set the simulated LIS3DHTR sensor data
     * 
     * @param data A shared pointer to the LIS3DHTRData
     */
    void setSimulatedLIS3DHTRData(std::shared_ptr<LIS3DHTRData> data);

    /**
     * @brief Set the simulated MS561101BA03 sensor data
     * 
     * @param data A shared pointer to the MS561101BA03Data
     */
    void setSimulatedMS561101BA03Data_1(std::shared_ptr<MS561101BA03Data> data);

    /**
     * @brief Set the simulated MS561101BA03 sensor data
     * 
     * @param data A shared pointer to the MS561101BA03Data
     */
    void setSimulatedMS561101BA03Data_2(std::shared_ptr<MS561101BA03Data> data);

    /**
     * @brief Set the simulated GPS sensor data
     * 
     * @param data A shared pointer to the GPSData
     */
    void setSimulatedGPSData(std::shared_ptr<GPSData> data);

    /**
     * @brief Get the flight state variables
     * 
     * @return A shared pointer to the flight state variables which is true if the rocket is rising, false otherwise
     */
    std::shared_ptr<bool> getIsRising();

    /**
     * @brief Get the estimated Height Gain Speed
     * 
     * @return std::shared_ptr<float> representing the height gain speed
     */
    std::shared_ptr<float> getHeightGainSpeed();

    /**
     * @brief Get the Current Estimated Height
     * 
     * @return std::shared_ptr<float> representing the current estimated height 
     */
    std::shared_ptr<float> getCurrentHeight();
    
 private:
    // Logger instance
    std::shared_ptr<RocketLogger> _logger;
    
    // Sensor instances
    std::shared_ptr<BNO055Sensor> _bno;
    std::shared_ptr<LIS3DHTRSensor> _lis3dh;
    std::shared_ptr<MS561101BA03> _ms56_1;
    std::shared_ptr<MS561101BA03> _ms56_2;
    std::shared_ptr<GPS> _gps;

    // Data related to the rocket state
    std::shared_ptr<BNO055Data> _bnoData;
    std::shared_ptr<LIS3DHTRData> _lis3dhData;
    std::shared_ptr<MS561101BA03Data> _ms561101ba03Data_1;
    std::shared_ptr<MS561101BA03Data> _ms561101ba03Data_2;
    std::shared_ptr<GPSData> _gpsData;

    float _batteryAdc, _batteryVoltage, _batteryPercentage;
    
    // Flight state variables
    std::shared_ptr<bool> _isRising;
    std::shared_ptr<float> _heightGainSpeed;
    std::shared_ptr<float> _currentHeight;
 };