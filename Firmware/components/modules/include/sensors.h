#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "filter.h"

#define G 9.80665f

typedef struct {
    int16_t x, y, z;
} raw_acce_t;

typedef struct {
    int16_t x, y, z;
} raw_gyro_t;

typedef struct {
    float x, y, z;
} acce_t;

typedef struct {
    float x, y, z;
} gyro_t;

typedef struct {
    gyro_t gyro;
    acce_t acce;
} sensor_data_t;

typedef struct {
    float distance;
} tof_data_t;

extern QueueHandle_t acce_queue;
extern QueueHandle_t gyro_queue;

esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);
esp_err_t mpu6050_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void i2c_mpu_device_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void mpu6050_deinit(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void mpu6050_read_motion_acce(i2c_master_dev_handle_t *dev_handle, raw_acce_t *raw_acce, acce_t *acce, lpf_data *lpf_data);
void mpu6050_read_motion_gyro(i2c_master_dev_handle_t *dev_handle, raw_gyro_t *raw_gyro, gyro_t *gyro, raw_gyro_t *gyro_bias, lpf_data *lpf_data);
void mpu6050_read_motion_raw(i2c_master_dev_handle_t *dev_handle, raw_gyro_t *raw_gyro, raw_acce_t *raw_acce);
void mpu6050_calibrate(i2c_master_dev_handle_t *dev_handle, raw_gyro_t *raw_gyro, raw_acce_t *raw_acce, raw_gyro_t *gyro_bias, raw_acce_t *acce_bias);
void sensor_task_init(void);
bool sensorsIsInit(void);
void waitSensorData(void);
void readIMUData(sensor_data_t *sensor_data);

#endif