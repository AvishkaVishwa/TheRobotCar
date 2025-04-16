#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

static const char *TAG = "IMU";

void imu_init() {
    // Initialize I2C for MPU6050
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21; // Change to your SDA pin
    conf.scl_io_num = 22; // Change to your SCL pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100kHz
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    // Wake up the MPU6050
    uint8_t data = 0;
    i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, &data, 1);
    ESP_LOGI(TAG, "IMU initialized");
}

void imu_read(float *accel, float *gyro) {
    uint8_t data[14];
    i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 14);

    // Convert the data to acceleration and gyroscope values
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
    int16_t gx = (data[8] << 8) | data[9];
    int16_t gy = (data[10] << 8) | data[11];
    int16_t gz = (data[12] << 8) | data[13];

    // Convert to g and degrees/sec
    accel[0] = ax / 16384.0; // Assuming +/- 2g
    accel[1] = ay / 16384.0;
    accel[2] = az / 16384.0;
    gyro[0] = gx / 131.0; // Assuming +/- 250 degrees/sec
    gyro[1] = gy / 131.0;
    gyro[2] = gz / 131.0;
}