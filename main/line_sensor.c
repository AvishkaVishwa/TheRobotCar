#include "line_sensor.h"
#include "driver/adc.h"

#define LEFT_THRESHOLD 2000
#define RIGHT_THRESHOLD 2000

void init_line_sensors() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LEFT_IR_PIN, ADC_ATTEN_DB_0);
    adc1_config_channel_atten(RIGHT_IR_PIN, ADC_ATTEN_DB_0);
}

int read_left_sensor() {
    return adc1_get_raw(LEFT_IR_PIN);
}

int read_right_sensor() {
    return adc1_get_raw(RIGHT_IR_PIN);
}

int is_on_black_tape() {
    int left_value = read_left_sensor();
    int right_value = read_right_sensor();

    if (left_value < LEFT_THRESHOLD && right_value < RIGHT_THRESHOLD) {
        return 1; // On black tape
    }
    return 0; // Not on black tape
}

void adjust_movement_based_on_line() {
    if (is_on_black_tape()) {
        // Continue moving forward
    } else {
        // Adjust movement to find the black tape
        int left_value = read_left_sensor();
        int right_value = read_right_sensor();

        if (left_value >= LEFT_THRESHOLD) {
            // Turn right
        } else if (right_value >= RIGHT_THRESHOLD) {
            // Turn left
        }
    }
}