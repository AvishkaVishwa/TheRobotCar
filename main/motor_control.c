#include "motor_control.h"
#include "driver/gpio.h"

#define STBY_PIN 33
#define AIN1_PIN 25
#define AIN2_PIN 26
#define PWMA_PIN 27
#define BIN1_PIN 14
#define BIN2_PIN 13
#define PWMB_PIN 12

void motor_init() {
    gpio_set_direction(STBY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(AIN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(AIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWMA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWMB_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(STBY_PIN, 1); // Enable the motor driver
}

void set_motor_speed(int motor, int speed) {
    if (motor == 1) { // Motor A
        gpio_set_level(AIN1_PIN, speed > 0 ? 1 : 0);
        gpio_set_level(AIN2_PIN, speed < 0 ? 1 : 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(speed));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else if (motor == 2) { // Motor B
        gpio_set_level(BIN1_PIN, speed > 0 ? 1 : 0);
        gpio_set_level(BIN2_PIN, speed < 0 ? 1 : 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(speed));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

void stop_motors() {
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
}