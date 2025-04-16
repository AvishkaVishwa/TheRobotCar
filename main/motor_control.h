#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Motor control pins
#define STBY_PIN 33
#define AIN1_PIN 25
#define AIN2_PIN 26
#define PWMA_PIN 27
#define BIN1_PIN 14
#define BIN2_PIN 13
#define PWMB_PIN 12

// Function declarations
void motor_init();
void set_motor_speed(int left_speed, int right_speed);
void stop_motors();
void move_forward();
void turn_left();
void turn_right();
void move_backward();

#endif // MOTOR_CONTROL_H