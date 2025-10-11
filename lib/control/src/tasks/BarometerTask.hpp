#pragma once
#include "BaseTask.hpp"

#include <MS561101BA03.hpp>
#include <Logger.hpp>
#include <SharedData.hpp>
#include <config.h>
#include <vector>
#include <algorithm>

// Simple moving average filter for noise reduction
class MovingAverageFilter {
public:
    MovingAverageFilter(size_t windowSize = 10) : windowSize(windowSize) {
        buffer.reserve(windowSize);
    }
    
    float update(float newValue) {
        buffer.push_back(newValue);
        if (buffer.size() > windowSize) {
            buffer.erase(buffer.begin());
        }
        
        float sum = 0.0f;
        for (float val : buffer) {
            sum += val;
        }
        return sum / buffer.size();
    }
    
    void reset() {
        buffer.clear();
    }
    
    bool isReady() const {
        return buffer.size() >= windowSize;
    }
    
private:
    size_t windowSize;
    std::vector<float> buffer;
};

// Median filter for spike rejection (better for outliers than moving average)
class MedianFilter {
public:
    MedianFilter(size_t windowSize = 5) : windowSize(windowSize) {
        buffer.reserve(windowSize);
    }
    
    float update(float newValue) {
        buffer.push_back(newValue);
        if (buffer.size() > windowSize) {
            buffer.erase(buffer.begin());
        }
        
        // Create sorted copy to find median
        std::vector<float> sorted = buffer;
        std::sort(sorted.begin(), sorted.end());
        
        size_t mid = sorted.size() / 2;
        if (sorted.size() % 2 == 0) {
            return (sorted[mid - 1] + sorted[mid]) / 2.0f;
        } else {
            return sorted[mid];
        }
    }
    
    void reset() {
        buffer.clear();
    }
    
    bool isReady() const {
        return buffer.size() >= windowSize;
    }
    
private:
    size_t windowSize;
    std::vector<float> buffer;
};

class BarometerTask : public BaseTask
{
public:
    BarometerTask(std::shared_ptr<SharedSensorData> sensorData,
                    SemaphoreHandle_t sensorDataMutex,
                    std::shared_ptr<ISensor> baro1,
                    std::shared_ptr<ISensor> baro2,
                    std::shared_ptr<bool> isRising,
                    std::shared_ptr<float> heightGainSpeed)
        : BaseTask("BarometerTask"),
          sensorData(sensorData),
          dataMutex(sensorDataMutex),
          baro1(baro1 ? baro1.get() : nullptr),
          baro2(baro2 ? baro2.get() : nullptr),
          isRising(isRising),
          heightGainSpeed(heightGainSpeed)
    {
        LOG_INFO("BarometerTask", "Initialized with Barometers: %s, %s",
                 baro1 ? "OK" : "NULL",
                 baro2 ? "OK" : "NULL");
    }

    void taskFunction() override;

    ~BarometerTask() override
    {
        stop();
    }
 
private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t dataMutex;
    ISensor *baro1;
    ISensor *baro2;
    std::shared_ptr<bool> isRising;
    std::shared_ptr<float> heightGainSpeed;

    // Maximum altitude reached for easy access in BarometerTask
    static float max_altitude_read;
    
    // Noise reduction: Median filters reject spikes better than moving average
    // Window size from config.h - tune BAROMETER_FILTER_WINDOW for your needs
    MedianFilter pressureFilter1{BAROMETER_FILTER_WINDOW};
    MedianFilter pressureFilter2{BAROMETER_FILTER_WINDOW};

    // Buffer for tendency filtering, used in isStillRising()
    std::vector<float> pressureTrendBuffer;
    size_t trendBufferSize = 10; // Apogee detection window size
    size_t mainDeploymentAltitude = 450; // Altitude for main deployment in meters

    // Called in update to add new values to the filter and remove old ones
    void addPressureTrendValue(float value) {
        if (pressureTrendBuffer.size() >= trendBufferSize)
            pressureTrendBuffer.erase(pressureTrendBuffer.begin());
        pressureTrendBuffer.push_back(value);
    }

    // If max altitude reached is needed to be retrived
    float getMaxAltitudeReached() { return max_altitude_read; }

};