#pragma once

/**
 * @file TelemetryFields.h
 * @brief Standardized field names for telemetry JSON messages.
 * 
 * These macros ensure consistency between sender (TelemetryTask) and 
 * receiver implementations. Changing a field name only requires updating
 * this file.
 */

// Root level fields
#define TELEM_FIELD_TIMESTAMP       "timestamp"
#define TELEM_FIELD_DATA_VALID      "dataValid"
#define TELEM_FIELD_IMU             "imu"
#define TELEM_FIELD_BARO1           "baro1"
#define TELEM_FIELD_BARO2           "baro2"
#define TELEM_FIELD_GPS             "gps"

// IMU sub-fields
#define TELEM_FIELD_IMU_ACCEL       "accel"
#define TELEM_FIELD_IMU_GYRO        "gyro"
#define TELEM_FIELD_IMU_MAG         "mag"

// Vector components (accel, gyro, mag)
#define TELEM_FIELD_VEC_X           "x"
#define TELEM_FIELD_VEC_Y           "y"
#define TELEM_FIELD_VEC_Z           "z"

// Barometer sub-fields
#define TELEM_FIELD_BARO_PRESSURE   "pressure"
#define TELEM_FIELD_BARO_TEMP       "temperature"
#define TELEM_FIELD_BARO_ALTITUDE   "altitude"

// GPS sub-fields
#define TELEM_FIELD_GPS_LAT         "lat"
#define TELEM_FIELD_GPS_LON         "lon"
#define TELEM_FIELD_GPS_ALT         "alt"
#define TELEM_FIELD_GPS_SPEED       "speed"
#define TELEM_FIELD_GPS_HEADING     "heading"
#define TELEM_FIELD_GPS_SATS        "sats"

// Optional: EKF/Filter fields (if you want to add filtered data)
#define TELEM_FIELD_EKF             "ekf"
#define TELEM_FIELD_EKF_ALT         "altitude"
#define TELEM_FIELD_EKF_VEL         "velocity"
#define TELEM_FIELD_EKF_QUAT        "quaternion"

// Flight state (optional - can be added later)
#define TELEM_FIELD_STATE           "state"
#define TELEM_FIELD_PHASE           "phase"
