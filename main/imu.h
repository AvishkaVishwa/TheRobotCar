#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// Function to initialize the MPU-6050 IMU
void imu_init(void);

// Function to read acceleration data
void imu_read_acceleration(int16_t *ax, int16_t *ay, int16_t *az);

// Function to read gyroscope data
void imu_read_gyroscope(int16_t *gx, int16_t *gy, int16_t *gz);

// Function to get the orientation based on accelerometer and gyroscope data
void imu_get_orientation(float *roll, float *pitch, float *yaw);

#endif // IMU_H