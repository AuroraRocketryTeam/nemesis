/**
 * @file pins.h
 * @author luca.pulga@studio.unibo.it
 * @brief PIN definition.
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/**
 * @brief I2C STD PIN.
 *
 */
// Pin I2C.
#define I2C_SDA 11
#define I2C_SCL 12
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// SD Card pins.
#define SD_CLK D13
#define SD_SO D12
#define SD_SI D11
#define SD_CS D10
#define SD_DET -1

// Actuators
#define DROGUE_ACTUATOR_PIN D0
#define MAIN_ACTUATOR_PIN D1
#define BUZZER_PIN D2

// LED
#define LED_RED_PIN A2
#define LED_GREEN_PIN A3
#define LED_BLUE_PIN A1

#define GPS_LED D6

#define ARMING_PIN D6

