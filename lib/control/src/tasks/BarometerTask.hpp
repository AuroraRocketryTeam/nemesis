#pragma once
#include "BaseTask.hpp"

#include <MS561101BA03.hpp>
#include <Logger.hpp>
#include <SharedData.hpp>
#include <Nemesis.hpp>
#include <config.h>
#include <vector>
#include <algorithm>

// If not commented, the Baro1 is used, otherwise Baro2
#define BARO_1

/**
 * @brief Class to implement a median filter.
 * 
 */
class MedianFilter {
public:
    /**
     * @brief Construct a new Median Filter object
     * 
     * @param windowSize The size of the median filter window
     */
    MedianFilter(size_t windowSize) : windowSize(windowSize) {
        buffer.reserve(windowSize);
    }

    /**
     * @brief Update the filter with a new value and get the filtered result
     * 
     * @param newValue The new value to add to the filter vector
     * @return float The filtered result
     */
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
    
    /**
     * @brief Reset the filter buffer
     * 
     */
    void reset() {
        buffer.clear();
    }
    
    /**
     * @brief Check if the filter is ready (i.e., has enough data)
     * 
     * @return true if the filter is ready
     * @return false if the filter is not ready
     */
    bool isReady() const {
        return buffer.size() >= windowSize;
    }
    
private:
    size_t windowSize;
    std::vector<float> buffer;
};

// The whole point of this class is not to deal directly with barometers, refactory needed!!!
/**
 * @brief Class to implement a barometer task.
 * 
 */
class BarometerTask : public BaseTask
{
public:
    /**
     * @brief Construct a new Barometer Task object 
     * 
     * @param model The shared pointer to the rocket model
     * @param modelMutex The mutex to protect access to the model
     */
    BarometerTask(std::shared_ptr<Nemesis> model,
                    SemaphoreHandle_t modelMutex)
        : BaseTask("BarometerTask"),
          _model(model),
          _modelMutex(modelMutex),
          _max_altitude_read(-1000.0f) // This third parameter should be probably removed!!!
    {}

    void taskFunction() override;

    ~BarometerTask() override
    {
        stop();
    }
 
private:
    std::shared_ptr<Nemesis> _model;
    SemaphoreHandle_t _modelMutex;

    // Maximum altitude reached for easy access in BarometerTask
    float _max_altitude_read;
    
    // Noise reduction: Median filters reject spikes better than moving average
    // Window size from config.h - tune BAROMETER_FILTER_WINDOW for your needs
    MedianFilter pressureFilter{BAROMETER_FILTER_WINDOW};

    // Buffer for tendency filtering, used in isStillRising()
    std::vector<float> pressureTrendBuffer;
    size_t trendBufferSize = APOGEE_DETECTION_WINDOW_SIZE;
    size_t mainDeploymentAltitude = 450; // Altitude for main deployment in meters

    // Called in update to add new values to the filter and remove old ones
    void addPressureTrendValue(float value) {
        if (pressureTrendBuffer.size() >= trendBufferSize) {
            pressureTrendBuffer.erase(pressureTrendBuffer.begin());
        }
            
        pressureTrendBuffer.push_back(value);
    }

    // If max altitude reached is needed to be retrived
    float getMaxAltitudeReached() { return _max_altitude_read; }

};