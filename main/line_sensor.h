#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

// Define the pin numbers for the line sensors
#define LEFT_IR_PIN ADC1_CHANNEL_7 // GPIO35
#define RIGHT_IR_PIN ADC1_CHANNEL_6 // GPIO34
#define MIDDLE_IR_PIN ADC1_CHANNEL_5 // GPIO33 (Assuming a middle sensor)

// Function to initialize the line sensors
void line_sensor_init(void);

// Function to read the state of the line sensors
uint8_t read_line_sensors(void);

// Function to check if the robot is on the black tape
uint8_t is_on_black_tape(void);

// Function to adjust the robot's movement based on line sensor readings
void adjust_movement_based_on_line(void);

#endif // LINE_SENSOR_H