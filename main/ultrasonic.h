#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

// Define the pins for the ultrasonic sensor
#define TRIG_PIN 18
#define ECHO_PIN 32

// Function to initialize the ultrasonic sensor
void ultrasonic_init(void);

// Function to read the distance from the ultrasonic sensor
uint32_t ultrasonic_read_distance(void);

#endif // ULTRASONIC_H