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

float relAltitude(float pressure, float pressureRef = 99323.79032f,
                  float temperatureRef = 299.15f)
{
    return temperatureRef / a * (1 - powf(pressure / pressureRef, nInv));
}

void BarometerTask::taskFunction()
{
    LOG_INFO("BarometerTask", "ENTERING THE TASK FUNCTION");
    float altitude1, altitude2;
    float pressure1, pressure2;

    // Filtered values
    float filtered_pressure1, filtered_pressure2;
    float filtered_altitude1, filtered_altitude2;

    vTaskDelay(pdMS_TO_TICKS(100)); // Initial delay to allow sensors to stabilize

    //LOG_INFO("BarometerTask", "Starting with median filter (window=%d)", BAROMETER_FILTER_WINDOW);

    while (running)
    {
        esp_task_wdt_reset();
        
        if(!running) break;
        // Read raw data from barometers
        //if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        //{
            auto presOpt1 = sensorData->baroData1.getData("pressure");
            pressure1 = std::get<float>(presOpt1.value());
            auto presOpt2 = sensorData->baroData2.getData("pressure");
            pressure2 = std::get<float>(presOpt2.value());

        //    xSemaphoreGive(dataMutex);
        //}
        //else
        //{
        //    LOG_ERROR("BarometerTask", "Failed to take sensor data mutex");
        //}        

        // Apply filters and compute altitudes
        filtered_pressure1 = pressureFilter1.update(pressure1);
        filtered_pressure2 = pressureFilter2.update(pressure2);

        //Da cambiare quando si sceglie il barometro da usare
        addPressureTrendValue(filtered_pressure1);
        //addPressureTrendValue(filtered_pressure2);
        
        // Calculate altitudes for comparison
        altitude1 = relAltitude(pressure1);              // Raw altitude
        filtered_altitude1 = relAltitude(filtered_pressure1);  // Filtered altitude
        altitude2 = relAltitude(pressure2);              // Raw altitude
        filtered_altitude2 = relAltitude(filtered_pressure2);  // Filtered altitude
        
        // Log of all the pressure data
        //LOG_INFO("BarometerTask", "Pressures (raw→filtered): Baro1=%.2f→%.2f hPa, Baro2=%.2f→%.2f hPa",
        //         pressure1, filtered_pressure1, pressure2, filtered_pressure2);

        // Da confermare se questa operazione è da fare in questo punto del ciclo
        if (pressureTrendBuffer.size() < trendBufferSize){
            *isRising = true; // Assume rising until we have enough data
        } else {
            for (size_t i = 0; i < pressureTrendBuffer.size(); ++i) {
                if (relAltitude(pressureTrendBuffer[i]) > max_altitude_read) {
                    *isRising = true;
                }
            }
            *isRising = false;
        }
        
        // True se l'ultima lettura del buffer è inferiore alla quota di deploy
        // Da cambiare quando si sceglie il barometro da usare
        *currentHeight = filtered_altitude1;
        // *currentHeight = filtered_altitude2;

        if (filtered_altitude1 > max_altitude_read)
            max_altitude_read = filtered_altitude1;
        if (filtered_altitude2 > max_altitude_read)
            max_altitude_read = filtered_altitude2;

        /*LOG_INFO("BarometerTask", "Temperatures: Baro1=%.2f K, Baro2=%.2f K",
                 temperature1, temperature2);
        LOG_INFO("BarometerTask", "Pressures (raw→filtered): Baro1=%.2f→%.2f hPa, Baro2=%.2f→%.2f hPa",
                 pressure1, filtered_pressure1, pressure2, filtered_pressure2);
        LOG_INFO("BarometerTask", "Altitudes (raw→filtered): Baro1=%.2f→%.2f m, Baro2=%.2f→%.2f m, Max=%.2f m",
                 altitude1, filtered_altitude1, altitude2, filtered_altitude2, max_altitude_read);
        */
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
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