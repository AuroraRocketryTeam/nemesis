#ifndef CSV_LOGGER_HPP
#define CSV_LOGGER_HPP

#include <Arduino.h>
#include <nlohmann/json.hpp>
#include <SD-master.hpp> // La tua classe SD personalizzata

using json = nlohmann::json;

class CSVLogger {
private:
    SD sdCard;
    bool headerWritten = false;
    std::string filename;
    
public:
    CSVLogger(std::string fileName) : filename(fileName) {}
    
    bool init() {
        if (!sdCard.init()) {
            Serial.println("Errore inizializzazione SD per CSV Logger");
            return false;
        }
        return true;
    }
    
    void writeHeader() {
        if (headerWritten) return;
        
        std::string header = "timestamp,"
                           "bno_temp,bno_acc_x,bno_acc_y,bno_acc_z,bno_acc_mag,"
                           "bno_gyro_x,bno_gyro_y,bno_gyro_z,"
                           "bno_mag_x,bno_mag_y,bno_mag_z,"
                           "bno_grav_x,bno_grav_y,bno_grav_z,"
                           "bno_lin_acc_x,bno_lin_acc_y,bno_lin_acc_z,"
                           "bno_orient_x,bno_orient_y,bno_orient_z,"
                           "bno_quat_w,bno_quat_x,bno_quat_y,bno_quat_z,"
                           "bno_accel_cal,bno_gyro_cal,bno_mag_cal,bno_sys_cal,"
                           "lis_acc_x,lis_acc_y,lis_acc_z,lis_acc_mag,"
                           "bar1_pres,bar1_temp,"
                           "bar2_pres,bar2_temp,"
                           "main_actuator_state,drogue_actuator_state,buzzer_state,"
                           "voltage_adc,voltage_v,voltage_perc,"
                           "gps_available\n";
        
        if (sdCard.writeFile(filename, header)) {
            headerWritten = true;
            Serial.println("Header CSV scritto");
        } else {
            Serial.println("Errore scrittura header CSV");
        }
        sdCard.closeFile();
    }
    
    void logSensorData(const json& allData, unsigned long timestamp) {
        if (!headerWritten) {
            writeHeader();
        }
        
        // Struttura per memorizzare i dati dei sensori
        struct SensorData {
            // BNO055
            float bno_temp = 0;
            float bno_acc_x = 0, bno_acc_y = 0, bno_acc_z = 0, bno_acc_mag = 0;
            float bno_gyro_x = 0, bno_gyro_y = 0, bno_gyro_z = 0;
            float bno_mag_x = 0, bno_mag_y = 0, bno_mag_z = 0;
            float bno_grav_x = 0, bno_grav_y = 0, bno_grav_z = 0;
            float bno_lin_acc_x = 0, bno_lin_acc_y = 0, bno_lin_acc_z = 0;
            float bno_orient_x = 0, bno_orient_y = 0, bno_orient_z = 0;
            float bno_quat_w = 0, bno_quat_x = 0, bno_quat_y = 0, bno_quat_z = 0;
            int bno_accel_cal = 0, bno_gyro_cal = 0, bno_mag_cal = 0, bno_sys_cal = 0;
            
            // LIS3DHTR
            float lis_acc_x = 0, lis_acc_y = 0, lis_acc_z = 0, lis_acc_mag = 0;
            
            // Barometri
            float bar1_pres = 0, bar1_temp = 0;
            float bar2_pres = 0, bar2_temp = 0;
            
            // Attuatori
            std::string main_actuator = "OFF";
            std::string drogue_actuator = "OFF";
            std::string buzzer = "OFF";
            
            // Voltaggio
            int voltage_adc = 0;
            float voltage_v = 0;
            std::string voltage_perc = "0%";
            
            // GPS
            bool gps_available = false;
        } data;
        
        // Parsing dei dati JSON dal RocketLogger
        if (allData.is_array()) {
            for (const auto& sensor : allData) {
                if (sensor.contains("content") && sensor["content"].contains("source")) {
                    std::string source = sensor["content"]["source"];
                    
                    if (source == "BNO055") {
                        const auto& sensorData = sensor["content"]["sensorData"];
                        
                        if (sensorData.contains("board_temperature")) {
                            data.bno_temp = sensorData["board_temperature"];
                        }
                        
                        if (sensorData.contains("accelerometer")) {
                            const auto& acc = sensorData["accelerometer"];
                            if (acc.contains("x")) data.bno_acc_x = acc["x"];
                            if (acc.contains("y")) data.bno_acc_y = acc["y"];
                            if (acc.contains("z")) data.bno_acc_z = acc["z"];
                            if (acc.contains("magnitude")) data.bno_acc_mag = acc["magnitude"];
                        }
                        
                        if (sensorData.contains("angular_velocity")) {
                            const auto& gyro = sensorData["angular_velocity"];
                            if (gyro.contains("x")) data.bno_gyro_x = gyro["x"];
                            if (gyro.contains("y")) data.bno_gyro_y = gyro["y"];
                            if (gyro.contains("z")) data.bno_gyro_z = gyro["z"];
                        }
                        
                        if (sensorData.contains("magnetometer")) {
                            const auto& mag = sensorData["magnetometer"];
                            if (mag.contains("x")) data.bno_mag_x = mag["x"];
                            if (mag.contains("y")) data.bno_mag_y = mag["y"];
                            if (mag.contains("z")) data.bno_mag_z = mag["z"];
                        }
                        
                        if (sensorData.contains("gravity")) {
                            const auto& grav = sensorData["gravity"];
                            if (grav.contains("x")) data.bno_grav_x = grav["x"];
                            if (grav.contains("y")) data.bno_grav_y = grav["y"];
                            if (grav.contains("z")) data.bno_grav_z = grav["z"];
                        }
                        
                        if (sensorData.contains("linear_acceleration")) {
                            const auto& linAcc = sensorData["linear_acceleration"];
                            if (linAcc.contains("x")) data.bno_lin_acc_x = linAcc["x"];
                            if (linAcc.contains("y")) data.bno_lin_acc_y = linAcc["y"];
                            if (linAcc.contains("z")) data.bno_lin_acc_z = linAcc["z"];
                        }
                        
                        if (sensorData.contains("orientation")) {
                            const auto& orient = sensorData["orientation"];
                            if (orient.contains("x")) data.bno_orient_x = orient["x"];
                            if (orient.contains("y")) data.bno_orient_y = orient["y"];
                            if (orient.contains("z")) data.bno_orient_z = orient["z"];
                        }
                        
                        if (sensorData.contains("quaternion")) {
                            const auto& quat = sensorData["quaternion"];
                            if (quat.contains("w")) data.bno_quat_w = quat["w"];
                            if (quat.contains("x")) data.bno_quat_x = quat["x"];
                            if (quat.contains("y")) data.bno_quat_y = quat["y"];
                            if (quat.contains("z")) data.bno_quat_z = quat["z"];
                        }
                        
                        if (sensorData.contains("accel_calibration")) {
                            data.bno_accel_cal = sensorData["accel_calibration"];
                        }
                        if (sensorData.contains("gyro_calibration")) {
                            data.bno_gyro_cal = sensorData["gyro_calibration"];
                        }
                        if (sensorData.contains("mag_calibration")) {
                            data.bno_mag_cal = sensorData["mag_calibration"];
                        }
                        if (sensorData.contains("system_calibration")) {
                            data.bno_sys_cal = sensorData["system_calibration"];
                        }
                    }
                    
                    else if (source == "LIS3DHTR") {
                        const auto& sensorData = sensor["content"]["sensorData"];
                        if (sensorData.contains("accel_x")) data.lis_acc_x = sensorData["accel_x"];
                        if (sensorData.contains("accel_y")) data.lis_acc_y = sensorData["accel_y"];
                        if (sensorData.contains("accel_z")) data.lis_acc_z = sensorData["accel_z"];
                        if (sensorData.contains("accel_magnitude")) data.lis_acc_mag = sensorData["accel_magnitude"];
                    }
                    
                    else if (source == "BAR1") {
                        const auto& sensorData = sensor["content"]["sensorData"];
                        if (sensorData.contains("pressure")) data.bar1_pres = sensorData["pressure"];
                        if (sensorData.contains("temperature")) data.bar1_temp = sensorData["temperature"];
                    }
                    
                    else if (source == "BAR2") {
                        const auto& sensorData = sensor["content"]["sensorData"];
                        if (sensorData.contains("pressure")) data.bar2_pres = sensorData["pressure"];
                        if (sensorData.contains("temperature")) data.bar2_temp = sensorData["temperature"];
                    }
                    
                    else if (source == "MainActuators") {
                        if (sensor["content"]["sensorData"].contains("State")) {
                            data.main_actuator = sensor["content"]["sensorData"]["State"];
                        }
                    }
                    
                    else if (source == "DrogueActuators") {
                        if (sensor["content"]["sensorData"].contains("State")) {
                            data.drogue_actuator = sensor["content"]["sensorData"]["State"];
                        }
                    }
                    
                    else if (source == "Buzzer") {
                        if (sensor["content"]["sensorData"].contains("State")) {
                            data.buzzer = sensor["content"]["sensorData"]["State"];
                        }
                    }
                    
                    else if (source == "Voltage") {
                        const auto& sensorData = sensor["content"]["sensorData"];
                        if (sensorData.contains("ADC_Value")) data.voltage_adc = sensorData["ADC_Value"];
                        if (sensorData.contains("Voltage")) data.voltage_v = sensorData["Voltage"];
                        if (sensorData.contains("Percentage")) data.voltage_perc = sensorData["Percentage"];
                    }
                    
                    else if (source == "GPS") {
                        // Se troviamo dati GPS, significa che sono disponibili
                        data.gps_available = true;
                    }
                }
                
                // Gestione errori GPS
                else if (sensor.contains("type") && sensor["type"] == "ERROR" && 
                         sensor.contains("content") && sensor["content"].contains("source") &&
                         sensor["content"]["source"] == "RocketLogger" && 
                         sensor["content"].contains("message")) {
                    std::string message = sensor["content"]["message"];
                    if (message.find("GPS") != std::string::npos) {
                        data.gps_available = false;
                    }
                }
            }
        }
        
        // Crea la riga CSV
        std::string csvLine = std::to_string(timestamp) + "," +
                             std::to_string(data.bno_temp) + "," +
                             std::to_string(data.bno_acc_x) + "," +
                             std::to_string(data.bno_acc_y) + "," +
                             std::to_string(data.bno_acc_z) + "," +
                             std::to_string(data.bno_acc_mag) + "," +
                             std::to_string(data.bno_gyro_x) + "," +
                             std::to_string(data.bno_gyro_y) + "," +
                             std::to_string(data.bno_gyro_z) + "," +
                             std::to_string(data.bno_mag_x) + "," +
                             std::to_string(data.bno_mag_y) + "," +
                             std::to_string(data.bno_mag_z) + "," +
                             std::to_string(data.bno_grav_x) + "," +
                             std::to_string(data.bno_grav_y) + "," +
                             std::to_string(data.bno_grav_z) + "," +
                             std::to_string(data.bno_lin_acc_x) + "," +
                             std::to_string(data.bno_lin_acc_y) + "," +
                             std::to_string(data.bno_lin_acc_z) + "," +
                             std::to_string(data.bno_orient_x) + "," +
                             std::to_string(data.bno_orient_y) + "," +
                             std::to_string(data.bno_orient_z) + "," +
                             std::to_string(data.bno_quat_w) + "," +
                             std::to_string(data.bno_quat_x) + "," +
                             std::to_string(data.bno_quat_y) + "," +
                             std::to_string(data.bno_quat_z) + "," +
                             std::to_string(data.bno_accel_cal) + "," +
                             std::to_string(data.bno_gyro_cal) + "," +
                             std::to_string(data.bno_mag_cal) + "," +
                             std::to_string(data.bno_sys_cal) + "," +
                             std::to_string(data.lis_acc_x) + "," +
                             std::to_string(data.lis_acc_y) + "," +
                             std::to_string(data.lis_acc_z) + "," +
                             std::to_string(data.lis_acc_mag) + "," +
                             std::to_string(data.bar1_pres) + "," +
                             std::to_string(data.bar1_temp) + "," +
                             std::to_string(data.bar2_pres) + "," +
                             std::to_string(data.bar2_temp) + "," +
                             data.main_actuator + "," +
                             data.drogue_actuator + "," +
                             data.buzzer + "," +
                             std::to_string(data.voltage_adc) + "," +
                             std::to_string(data.voltage_v) + "," +
                             data.voltage_perc + "," +
                             (data.gps_available ? "1" : "0") + "\n";
        
        // Scrivi su SD
        if (!sdCard.appendFile(filename, csvLine)) {
            Serial.println("Errore scrittura CSV");
        }
        sdCard.closeFile();
    }
};

#endif // CSV_LOGGER_HPP