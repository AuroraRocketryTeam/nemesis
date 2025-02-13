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

#define LORA_RX 2
#define LORA_TX 3
#define LORA_AUX 4
#define LORA_M0 8
#define LORA_M1 9
// Add here some other pin.


