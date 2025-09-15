#define __DEBUG__

#define SUFFICIENT_SENSOR_CALIBRATION 2
#define SENSOR_LOOKUP_MAX_ATTEMPTS 5
#define SENSOR_LOOKUP_TIMEOUT 1000


#define MS56_I2C_ADDR_1 0x77
#define MS56_I2C_ADDR_2 0x76
#define BNO055_I2C_ADDR 0x28

/* Legacy sensors */
// #define BME680_I2C_ADDR_1 0x77
// #define BME680_I2C_ADDR_2 0x76
// #define MPRLS_I2C_ADDR 0x18
// #define I2C_MULTIPLEXER_ADDRESS 0x70
// #define I2C_MULTIPLEXER_MPRLS1 1
// #define I2C_MULTIPLEXER_MPRLS2 0

/* LoRa configuration */
#define E220_22         // E220-900T22D
#define FREQUENCY_868  // 868 MHz

#define LORA_SERIAL 1 // Serial port for LoRa (1 = Serial1, 2 = Serial2)

/* Transmitter antenna address */
#define LORA_ADDH 0x00
#define LORA_ADDL 0x03
/* Receiver antenna address */
#define LORA_RECEIVER_ADDH 0x00
#define LORA_RECEIVER_ADDL 0x05

#define LORA_CHANNEL 23 // Communication channel

/* Uncomment the following line to enable RSSI */
#define ENABLE_RSSI

#define SERIAL_BAUD_RATE 115200 // Serial baud rate

// #define TRANSMITTER_CONFIG_MODE_ENABLE // Enable configuration mode for the transmitter (needs to be tested on new hardware)

/* Flight parameters configuration */
#define LIFTOFF_TIMEOUT_MS 100      // Threshold for the detection of liftoff when relative_acceleration is > gravity in any direction (relative_acceleration = acceleration - gravity)
#define DROGUE_APOGEE_TIMEOUT 300   // Threshold for opening the drogue parachute after apogee is detected

/* Flight parameters configuration */
#define LIFTOFF_TIMEOUT_MS 100      // Threshold for the detection of liftoff when relative_acceleration is > gravity in any direction (relative_acceleration = acceleration - gravity)
#define DROGUE_APOGEE_TIMEOUT 300   // Threshold for opening the drogue parachute after apogee is detected

// Kalman Constants
#define SEA_LEVEL 165.0f
#define H_BIAS_PRESSURE_SENSOR 2.0f
#define GPS_BIAS 3.0f

// Important math constants
#define GRAVITY 9.80665f