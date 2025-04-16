#include "ultrasonic.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX_DISTANCE 400 // Maximum distance to measure in cm
#define TIMEOUT 30000    // Timeout for echo in microseconds

static const char *TAG = "Ultrasonic";

void ultrasonic_init() {
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}

float ultrasonic_read_distance() {
    // Send a 10us pulse to trigger the sensor
    gpio_set_level(TRIG_PIN, 1);
    ets_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    // Wait for the echo to go high
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_time > TIMEOUT) {
            ESP_LOGE(TAG, "Timeout waiting for echo high");
            return MAX_DISTANCE; // Return max distance on timeout
        }
    }

    // Wait for the echo to go low
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - start_time > TIMEOUT) {
            ESP_LOGE(TAG, "Timeout waiting for echo low");
            return MAX_DISTANCE; // Return max distance on timeout
        }
    }

    // Calculate the distance in cm
    uint32_t duration = esp_timer_get_time() - start_time;
    float distance = (duration / 2.0) * 0.0343; // Speed of sound is 0.0343 cm/us

    return distance < MAX_DISTANCE ? distance : MAX_DISTANCE;
}