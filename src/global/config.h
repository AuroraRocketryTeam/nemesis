#define SENSOR_LOOKUP_MAX_ATTEMPTS 5
#define SENSOR_LOOKUP_TIMEOUT 1000

#define BME680_I2C_ADDR_1 0x77
#define BME680_I2C_ADDR_2 0x76
#define BNO055_I2C_ADDR 0x28
#define MPRLS_I2C_ADDR 0x18

/* LoRa configuration */
#define LORA_SERIAL 1
#define LORA_RX_PIN 2
#define LORA_TX_PIN 3

#define LORA_ADDH 0x00
#define LORA_ADDL 0x03
#define LORA_DESTINATION_ADDH 0x00
#define LORA_DESTINATION_ADDL 0x05
#define LORA_CHANNEL 23

/* Uncomment the following line to enable RSSI */
#define ENABLE_RSSI

#define SERIAL_BAUD_RATE 115200
