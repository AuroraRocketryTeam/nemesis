// SimulationTask implementation - Line-based tracking
#include "SimulationTask.hpp"
#include <sstream>
#include <algorithm>

bool SimulationTask::firstTime = true;
unsigned long SimulationTask::startTime = millis();
SD SimulationTask::sdManager;
std::string SimulationTask::csvFilePath;
uint32_t SimulationTask::filePosition = 0;  // Track current line number
bool SimulationTask::fileInitialized = false;

SimulationTask::SimulationTask(const std::string& csvFilePathPar,
                                std::shared_ptr<SharedSensorData> sensorData,
                                SemaphoreHandle_t mutex,
                                std::shared_ptr<RocketLogger> rocketLogger,
                                SemaphoreHandle_t loggerMutex)
        : BaseTask("SimulationTask"), sensorData(sensorData),
            dataMutex(mutex), rocketLogger(rocketLogger), loggerMutex(loggerMutex) {
    
    // Only initialize SD and open file once for all instances
    if (!fileInitialized) {
        csvFilePath = csvFilePathPar;
        firstTime = true;
        filePosition = 0;
        
        LOG_INFO("SimulationTask", "Opening CSV file: %s", csvFilePath.c_str());
        
        if (!sdManager.init()) {
            LOG_ERROR("SimulationTask", "Failed to initialize SD card");
            return;
        }
        if (!sdManager.fileExists(csvFilePath)) {
            LOG_ERROR("SimulationTask", "CSV file does not exist: %s", csvFilePath.c_str());
            return;
        }
        if (!sdManager.openFile(csvFilePath)) {
            LOG_ERROR("SimulationTask", "Failed to open CSV file: %s", csvFilePath.c_str());
            return;
        }
        
        fileInitialized = true;
        LOG_INFO("SimulationTask", "Successfully opened CSV file");
    } else {
        LOG_INFO("SimulationTask", "Reusing already opened CSV file at line %u", filePosition);
    }
}

SimulationTask::~SimulationTask() {
    // Do not close file here, keep it open for all instances
}

void SimulationTask::onTaskStart() {
    if (firstTime) {
        startTime = millis();
    }
    
    // If resuming from a previous position, skip to the correct line
    if (filePosition > 0) {
        LOG_INFO("SimulationTask", "Seeking to line number: %u", filePosition);
        // Rewind file to start
        sdManager.closeFile();
        sdManager.openFile(csvFilePath);
        
        // Skip header
        sdManager.readLine();
        
        // Skip lines until we reach filePosition
        for (uint32_t i = 1; i < filePosition; i++) {
            sdManager.readLine();
        }
        LOG_INFO("SimulationTask", "Resumed at line %u", filePosition);
    }
}

void SimulationTask::onTaskStop() {
    // Save current line number when stopping
    LOG_INFO("SimulationTask", "Saved line number: %u", filePosition);
}

void SimulationTask::reset() {
    // Reset simulation to beginning
    filePosition = 0;
    firstTime = true;
    startTime = millis();
    if (fileInitialized) {
        sdManager.closeFile();
        sdManager.openFile(csvFilePath);
        LOG_INFO("SimulationTask", "Reset simulation to beginning");
    }
}

// Helper function to trim whitespace and newlines from string
std::string trimString(const std::string& str) {
    size_t start = 0;
    size_t end = str.length();
    
    // Trim from start
    while (start < end && (std::isspace(str[start]) || str[start] == '\n' || str[start] == '\r')) {
        start++;
    }
    
    // Trim from end
    while (end > start && (std::isspace(str[end - 1]) || str[end - 1] == '\n' || str[end - 1] == '\r')) {
        end--;
    }
    
    return str.substr(start, end - start);
}

void SimulationTask::taskFunction() {
    try {
        while (running) {
            auto now = millis();
            double elapsed = now - startTime;

            if (firstTime) {
                String header = sdManager.readLine(); // skip header
                firstTime = false;
                filePosition = 1; // After header, we're at line 1
            }
            
            String line = sdManager.readLine();
            
            // Preprocess the line: trim whitespace and newlines
            std::string lineStr = trimString(std::string(line.c_str()));
            
            if (lineStr.length() > 0) {
                filePosition++; // Increment line counter
                
                std::stringstream ss(lineStr);
                std::string cell;
                
                // Read time
                std::getline(ss, cell, ',');
                double time_s = std::stod(trimString(cell));
                LOG_INFO("SimulationTask", "READ TIME: %.2f (Line %u)", time_s, filePosition);

                // Parse CSV columns into variables
                #ifdef OLD_DATA
                    float AccBodyX_ms2, AccBodyY_ms2, AccBodyZ_ms2;
                    float AccSensorX_ms2, AccSensorY_ms2, AccSensorZ_ms2;
                    float OmegaBody_degs, OmegaSensor_degs;
                    float OmegaBody_degs_Y, OmegaSensor_degs_Y;
                    float OmegaBody_degs_Z, OmegaSensor_degs_Z;
                    float Z_real_m, Z_sensor_m;
                    float Pressure_real_Pa, Pressure_sensor_Pa;
                #else
                    float ax, ay, az, amag;
                    float pressure;
                    float z;
                #endif

                std::vector<float> values;
                while (std::getline(ss, cell, ',')) {
                    std::string trimmedCell = trimString(cell);
                    if (trimmedCell.length() > 0) {
                        try {
                            values.push_back(std::stof(trimmedCell));
                        } catch (const std::exception& e) {
                            LOG_ERROR("SimulationTask", "Failed to parse value: '%s', error: %s", 
                                     trimmedCell.c_str(), e.what());
                        }
                    }
                }
                
                // Assign variables from values vector
                #ifdef OLD_DATA
                    AccBodyX_ms2 = values[0];
                    AccBodyY_ms2 = values[1];
                    AccBodyZ_ms2 = values[2];
                    AccSensorX_ms2 = values[3];
                    AccSensorY_ms2 = values[4];
                    AccSensorZ_ms2 = values[5];
                    OmegaBody_degs = values[6];
                    OmegaSensor_degs = values[7];
                    OmegaBody_degs_Y = values[8];
                    OmegaSensor_degs_Y = values[9];
                    OmegaBody_degs_Z = values[10];
                    OmegaSensor_degs_Z = values[11];
                    Z_real_m = values[12];
                    Z_sensor_m = values[13];
                    Pressure_real_Pa = values[14];
                    Pressure_sensor_Pa = values[15];
                #else
                    ax = values[0];
                    ay = values[1];
                    az = values[2];
                    amag = values[3];
                    pressure = values[4];
                    z = values[5];
                #endif                
                
                
                #ifdef OLD_DATA
                    // Mappings between values and BNO055 data fields
                    SensorData bnoData("BNO055");
                    bnoData.setData("system_calibration", 3);
                    bnoData.setData("gyro_calibration", 3);
                    bnoData.setData("accel_calibration", 3);
                    bnoData.setData("mag_calibration", 3);

                    auto xAcc = static_cast<float>(AccBodyX_ms2);
                    auto yAcc = static_cast<float>(AccBodyY_ms2);
                    auto zAcc = static_cast<float>(AccBodyZ_ms2);
                    auto accMagnitude = std::sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc);
                    bnoData.setData("accelerometer", std::map<std::string, float>{
                        {"x", xAcc},
                        {"y", yAcc},
                        {"z", zAcc},
                        {"magnitude", accMagnitude}});

                    bnoData.setData("angular_velocity", std::map<std::string, float>{
                        {"x", static_cast<float>(OmegaBody_degs)},
                        {"y", static_cast<float>(OmegaBody_degs_Y)},
                        {"z", static_cast<float>(OmegaBody_degs_Z)}});

                    // Mappings between MS561101BA03 data fields and columns
                    SensorData baroData1 = SensorData("MS561101BA03");
                    baroData1.setData("pressure", Pressure_sensor_Pa);

                    SensorData baroData2 = SensorData("MS561101BA03");
                    baroData2.setData("pressure", Pressure_sensor_Pa);

                    // Mappings between LIS3DHTR data fields and data fields
                    SensorData accData = SensorData("LIS3DHTR");
                    accData.setData("accel_x", AccSensorX_ms2);
                    accData.setData("accel_y", AccSensorY_ms2);
                    accData.setData("accel_z", AccSensorZ_ms2);
                    accData.setData("accel_magnitude", std::sqrt(AccSensorX_ms2 * AccSensorX_ms2 +
                                                                AccSensorY_ms2 * AccSensorY_ms2 +
                                                                AccSensorZ_ms2 * AccSensorZ_ms2));
                    
                    // Mapping between GPS data fields and data fields                                                            
                    SensorData gpsData = SensorData("GPS");
                    gpsData.setData("altitude",  Z_sensor_m);
                #else
                    // Mappings between values and BNO055 data fields
                    SensorData bnoData("BNO055");
                    bnoData.setData("system_calibration", 3);
                    bnoData.setData("gyro_calibration", 3);
                    bnoData.setData("accel_calibration", 3);
                    bnoData.setData("mag_calibration", 3);

                    bnoData.setData("accelerometer", std::map<std::string, float>{
                        {"x", ax},
                        {"y", ay},
                        {"z", az},
                        {"magnitude", amag}});

                    // Mappings between MS561101BA03 data fields and columns
                    SensorData baroData1 = SensorData("MS561101BA03");
                    baroData1.setData("pressure", pressure);

                    SensorData baroData2 = SensorData("MS561101BA03");
                    baroData2.setData("pressure", pressure);

                    // Mappings between LIS3DHTR data fields and data fields
                    SensorData accData = SensorData("LIS3DHTR");
                    accData.setData("accel_x", ax);
                    accData.setData("accel_y", ay);
                    accData.setData("accel_z", az);
                    accData.setData("accel_magnitude", amag);
                    
                    // Mapping between GPS data fields and data fields                                                            
                    SensorData gpsData = SensorData("GPS");
                    gpsData.setData("altitude",  z);
                #endif
                

                // Inject into shared sensor data
                if (sensorData && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                    sensorData->imuData = bnoData;
                    sensorData->baroData1 = baroData1;
                    sensorData->baroData2 = baroData2;
                    //sensorData->accData = accData;
                    sensorData->gpsData = gpsData;
                    sensorData->timestamp = static_cast<uint32_t>(now);
                    sensorData->dataValid = true;

                    xSemaphoreGive(dataMutex);
                }
                
                // Log of all the read data
                LOG_INFO("SimulationTask", 
                    "Time: %.2f s | AccSensor: [%.2f, %.2f, %.2f] m/s² Mag: %.2f |"
                    "pressure: %.2f Pa | Z_sensor: %.2f m",
                    time_s,
                    #ifdef OLD_DATA
                        AccSensorX_ms2, AccSensorY_ms2, AccSensorZ_ms2, accMagnitude,
                        Pressure_sensor_Pa, Z_sensor_m
                    #else
                        ax, ay, az, amag,
                        pressure, z
                    #endif
                );
            } else {
                LOG_INFO("SimulationTask", "End of CSV file reached, stopping simulation.");
                break;
            }

            // Sleep for 100ms, which is the time gap between simulation data points
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } catch (const std::exception &e) {
        while (true) {
            LOG_ERROR("SimulationTask", "Exception in taskFunction: %s", e.what());
            vTaskDelay(pdMS_TO_TICKS(1000)); // Add delay to prevent tight error loop
        }
    }
}