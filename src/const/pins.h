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
#ifndef PINS_H
#define PINS_H

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
#define SD_CLK D13 // D9 pin on nano esp32
#define SD_SO D12// D10 pin on nano esp32
#define SD_SI D11 // D11 pin on nano esp32
#define SD_CS D7 // D7 pin on nano esp32


// Add here some other pin.

#endif // PINS_H
