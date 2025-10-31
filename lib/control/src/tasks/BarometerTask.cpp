#include "BarometerTask.hpp"
#include <cmath>
#include <config.h>
#include <Logger.hpp>

#define MAX_ALTITUDE_THRESHOLD 5000.0f // meters

constexpr float TROPOSPHERE_HEIGHT = 11000.f; // Troposphere height [m]
constexpr float a = 0.0065f;                  // Troposphere temperature gradient [deg/m]
constexpr float R = 287.05f;                  // Air gas constant [J/Kg/K]
#define n (GRAVITY / (R * a))
#define nInv ((R * a) / GRAVITY)

float relAltitude(float pressure, float pressureRef = 99725.0f,
                  float temperatureRef = 291.41f)
{
    return temperatureRef / a * (1 - powf(pressure / pressureRef, nInv));
}

void BarometerTask::taskFunction()
{
    float altitude, pressure;
    float filtered_pressure, filtered_altitude;

    //LOG_INFO("BarometerTask", "Starting with median filter (window=%d)", BAROMETER_FILTER_WINDOW);

    while (running)
    {

        esp_task_wdt_reset();
        if(!running) break;
        
        // Read raw data from barometers
        #ifdef BARO_1
            auto baro1Data = _model->getMS561101BA03Data_1();
            pressure = baro1Data->pressure;
        #else
            auto baro2Data = _model->getMS561101BA03Data_2();
            pressure = baro2Data->pressure;
        #endif
        
        // Apply filters and compute altitudes
        filtered_pressure = pressureFilter.update(pressure);
        
        addPressureTrendValue(filtered_pressure);
        
        // Calculate altitudes for comparison
        altitude = relAltitude(pressure);              // Raw altitude
        filtered_altitude = relAltitude(filtered_pressure);  // Filtered altitude

        // Controls if the last readings indicate that the system is rising or not
        // True if at least one value in the buffer is higher than the previous maximum 
        // This function could be moved to the hpp for cleaner code, but just want to be sure
        // possiamo fare in questo modo o mettere questo codice nel loop qui sopra ed avere una variabile bool privata con una funzione getter
        auto isRising = _model->getIsRising();
        if (pressureTrendBuffer.size() < trendBufferSize){
            *isRising = true; // Assume rising until we have enough data
        } else {
            bool rised  = false;
            for (size_t i = 0; i < pressureTrendBuffer.size(); i++) {
                if (relAltitude(pressureTrendBuffer[i]) >= _max_altitude_read) {
                    rised = true;
                    break;
                }
            }
            
            *isRising = rised;            
        }

        
        LOG_INFO("BarometerTask", "Altitude value: %0.2f, Max_Altitude: %0.2f", filtered_altitude, _max_altitude_read);
        
        // True se l'ultima lettura del buffer è inferiore alla quota di deploy
        auto currentHeight = _model->getCurrentHeight();
        *currentHeight = filtered_altitude;

        if (filtered_altitude > _max_altitude_read) {
            _max_altitude_read = filtered_altitude;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }

}