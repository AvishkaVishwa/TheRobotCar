#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "line_sensor.h"
#include "ultrasonic.h"
#include "motor_control.h"
#include "imu.h"

void app_main(void) {
    // Initialize the motor control
    motor_control_init();

    // Initialize the line sensors
    line_sensor_init();

    // Initialize the ultrasonic sensor
    ultrasonic_init();

    // Initialize the IMU
    imu_init();

    while (1) {
        // Read line sensor values
        int line_status = line_sensor_read();

        // Check for obstacles using ultrasonic sensor
        float distance = ultrasonic_read_distance();

        // If an obstacle is detected within a certain range
        if (distance < 20.0) {
            // Stop and avoid the obstacle
            motor_control_stop();
            // Implement obstacle avoidance logic here
            // For example, reverse and turn
            motor_control_reverse();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            motor_control_turn_right();
            vTaskDelay(500 / portTICK_PERIOD_MS);
        } else {
            // Follow the line based on sensor readings
            motor_control_follow_line(line_status);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for stability
    }
}