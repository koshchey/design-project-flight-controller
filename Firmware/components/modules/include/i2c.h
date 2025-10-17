#ifndef I2C_H_
#define I2C_H_

#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO 22 // GPIO number used for I2C master clock 
#define I2C_MASTER_SDA_IO 21 // GPIO number used for I2C master data 
#define I2C_MASTER_NUM  I2C_NUM_0 // I2C port number for master dev 
#define I2C_MASTER_FREQ_HZ 400000 // I2C master clock frequency (400kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU6050_INT_GPIO_PIN 19

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

#endif