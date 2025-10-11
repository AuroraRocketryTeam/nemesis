#include "BarometerTask.hpp"
#include <cmath>
#include <config.h>
#include <Logger.hpp>

#define MAX_ALTITUDE_THRESHOLD 5000.0f // meters

// Max altitude initialization
float BarometerTask::max_altitude_read = -1000.0f;

constexpr float TROPOSPHERE_HEIGHT = 11000.f; // Troposphere height [m]
constexpr float a = 0.0065f;                  // Troposphere temperature gradient [deg/m]
constexpr float R = 287.05f;                  // Air gas constant [J/Kg/K]
#define n (GRAVITY / (R * a))
#define nInv ((R * a) / GRAVITY)

float relAltitude(float pressure, float pressureRef = 101070.0f,
                  float temperatureRef = 299.15f)
{
    return temperatureRef / a * (1 - powf(pressure / pressureRef, nInv));
}

void BarometerTask::taskFunction()
{
    float altitude1 = -1000.0f;
    float altitude2 = -1000.0f;
    float pressure1 = -1.0f;
    float pressure2 = -1.0f;
    float temperature1 = -100.0f;
    float temperature2 = -100.0f;

    // Filtered values
    float filtered_pressure1 = -1.0f;
    float filtered_pressure2 = -1.0f;
    float filtered_altitude1 = -1000.0f;
    float filtered_altitude2 = -1000.0f;

    vTaskDelay(pdMS_TO_TICKS(100)); // Initial delay to allow sensors to stabilize

    LOG_INFO("BarometerTask", "Starting with median filter (window=%d)", BAROMETER_FILTER_WINDOW);

    while (running)
    {
        esp_task_wdt_reset();

        if (baro1)
        {
            auto baroData1 = baro1->getData();
            if (baroData1.has_value())
            {
                // Read raw values
                pressure1 = std::get<float>(baroData1.value().getData("pressure").value());
                temperature1 = std::get<float>(baroData1.value().getData("temperature").value());

                // Apply pressure filter, then calculate altitude from filtered pressure
                filtered_pressure1 = pressureFilter1.update(pressure1);
                addPressureTrendValue(filtered_pressure1);

                // Calculate altitudes for comparison
                altitude1 = relAltitude(pressure1);              // Raw altitude
                filtered_altitude1 = relAltitude(filtered_pressure1);  // Filtered altitude

                // Store filtered values in shared data
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    // Create new SensorData with filtered values
                    SensorData filteredData("MS561101BA03_Filtered");
                    filteredData.setData("pressure", filtered_pressure1);
                    filteredData.setData("temperature", temperature1); // Temperature doesn't need filtering

                    sensorData->baroData1 = filteredData;
                    sensorData->timestamp = millis();
                    sensorData->dataValid = true;
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                LOG_WARNING("BarometerTask", "Failed to read data from Barometer 1");
            }
        }

        if (baro2)
        {
            auto baroData2 = baro2->getData();
            if (baroData2.has_value())
            {
                // Read raw values
                pressure2 = std::get<float>(baroData2.value().getData("pressure").value());
                temperature2 = std::get<float>(baroData2.value().getData("temperature").value());

                // Apply pressure filter, then calculate altitude from filtered pressure
                filtered_pressure2 = pressureFilter2.update(pressure2);
                addPressureTrendValue(filtered_pressure2);
                
                // Calculate altitudes for comparison
                altitude2 = relAltitude(pressure2);              // Raw altitude
                filtered_altitude2 = relAltitude(filtered_pressure2);  // Filtered altitude

                // Store filtered values in shared data
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    // Create new SensorData with filtered values
                    SensorData filteredData("MS561101BA03_Filtered");
                    filteredData.setData("pressure", filtered_pressure2);
                    filteredData.setData("temperature", temperature2); // Temperature doesn't need filtering

                    sensorData->baroData2 = filteredData;
                    sensorData->timestamp = millis();
                    sensorData->dataValid = true;
                    xSemaphoreGive(dataMutex);
                }
            }
            else
            {
                LOG_WARNING("BarometerTask", "Failed to read data from Barometer 2");
            }
        }

        // Da confermare se questa operazione è da fare in questo punto del ciclo
        if (pressureTrendBuffer.size() < trendBufferSize){
            isRising = std::make_shared<bool>(true); // Assume rising until we have enough data
        } else {
            for (size_t i = 0; i < pressureTrendBuffer.size(); ++i) {
                if (relAltitude(pressureTrendBuffer[i]) > max_altitude_read) {
                    isRising = std::make_shared<bool>(true);
                }
            }
            isRising = std::make_shared<bool>(false);
        }

        if (filtered_altitude1 > max_altitude_read)
            max_altitude_read = filtered_altitude1;
        if (filtered_altitude2 > max_altitude_read)
            max_altitude_read = filtered_altitude2;

        LOG_INFO("BarometerTask", "Temperatures: Baro1=%.2f K, Baro2=%.2f K",
                 temperature1, temperature2);
        LOG_INFO("BarometerTask", "Pressures (raw→filtered): Baro1=%.2f→%.2f hPa, Baro2=%.2f→%.2f hPa",
                 pressure1, filtered_pressure1, pressure2, filtered_pressure2);
        LOG_INFO("BarometerTask", "Altitudes (raw→filtered): Baro1=%.2f→%.2f m, Baro2=%.2f→%.2f m, Max=%.2f m",
                 altitude1, filtered_altitude1, altitude2, filtered_altitude2, max_altitude_read);

        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz
    }

}

// Controls if the last readings indicate that the system is rising or not
// True if at least one value in the buffer is higher than the previous maximum 
// This function could be moved to the hpp for cleaner code, but just want to be sure
// possiamo fare in questo modo o mettere questo codice nel loop qui sopra ed avere una variabile bool privata con una funzione getter
/*bool BarometerTask::isStillRising() {
    if (pressureTrendBuffer.size() < trendBufferSize) return false;
    
    for (size_t i = 0; i < pressureTrendBuffer.size(); ++i) {
        if (pressureTrendBuffer[i] > max_altitude_read) {
            return isRising = true;
        }
    }
    return isRising = false;
}*/