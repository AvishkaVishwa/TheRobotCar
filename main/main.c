#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "driver/ultrasonic.h"
#include "servo.h" // Custom servo library header

#define TAG "ROBOT"

// Motor Pins
#define STBY_PIN          GPIO_NUM_33
#define AIN1_PIN          GPIO_NUM_25
#define AIN2_PIN          GPIO_NUM_26
#define PWMA_PIN          GPIO_NUM_27  // MCPWM0A
#define BIN1_PIN          GPIO_NUM_14
#define BIN2_PIN          GPIO_NUM_13
#define PWMB_PIN          GPIO_NUM_12  // MCPWM0B

// Sensor Pins
#define LEFT_IR_PIN       ADC1_CHANNEL_7  // GPIO35
#define MIDDLE_IR_PIN     ADC1_CHANNEL_3  // GPIO39
#define RIGHT_IR_PIN      ADC1_CHANNEL_6  // GPIO34
#define TRIG_PIN          GPIO_NUM_18
#define ECHO_PIN          GPIO_NUM_32
#define SERVO_PIN         GPIO_NUM_19

// Constants
#define OBSTACLE_DISTANCE_CM 25
#define LINE_THRESHOLD       2000
#define MOTOR_BASE_SPEED     0.7f  // 0-1.0
#define TURN_FACTOR          0.3f
#define SERVO_CENTER         90
#define TASK_STACK_SIZE      4096

// PID Parameters
static const float Kp = 0.8f;
static const float Ki = 0.02f;
static const float Kd = 0.6f;
static float integral = 0;
static float previous_error = 0;

// Global States
static bool obstacle_detected = false;
static int current_servo_pos = SERVO_CENTER;

// Hardware Timer for Ultrasonic
static esp_timer_handle_t ultrasonic_timer;
static volatile uint64_t pulse_start = 0;
static volatile uint32_t distance_cm = 0;

void ultrasonic_isr(void* arg) {
    static bool rising_edge = true;
    int level = gpio_get_level(ECHO_PIN);
    
    if(rising_edge && level == 1) {
        pulse_start = esp_timer_get_time();
        rising_edge = false;
    } else if(!rising_edge && level == 0) {
        uint64_t pulse_end = esp_timer_get_time();
        distance_cm = (pulse_end - pulse_start) / 58;
    }
}

void init_ultrasonic() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_PIN, ultrasonic_isr, NULL);
}

void init_motors() {
    // Configure MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWMA_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWMB_PIN);

    mcpwm_config_t pwm_config = {
        .frequency = 20000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Direction pins
    gpio_set_direction(AIN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(AIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(STBY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY_PIN, 1);
}

void set_motor_speeds(float left, float right) {
    // Left motor
    if(left >= 0) {
        gpio_set_level(AIN1_PIN, 1);
        gpio_set_level(AIN2_PIN, 0);
    } else {
        gpio_set_level(AIN1_PIN, 0);
        gpio_set_level(AIN2_PIN, 1);
        left = -left;
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, left * 100);

    // Right motor
    if(right >= 0) {
        gpio_set_level(BIN1_PIN, 0);
        gpio_set_level(BIN2_PIN, 1);
    } else {
        gpio_set_level(BIN1_PIN, 1);
        gpio_set_level(BIN2_PIN, 0);
        right = -right;
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, right * 100);
}

void line_follow_task(void *pvParameters) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LEFT_IR_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(RIGHT_IR_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(MIDDLE_IR_PIN, ADC_ATTEN_DB_11);

    while(1) {
        if(!obstacle_detected) {
            int left = adc1_get_raw(LEFT_IR_PIN);
            int middle = adc1_get_raw(MIDDLE_IR_PIN);
            int right = adc1_get_raw(RIGHT_IR_PIN);

            float error = 0;
            if(middle > LINE_THRESHOLD) {
                if(left > LINE_THRESHOLD) error = -2.0f;
                else if(right > LINE_THRESHOLD) error = 2.0f;
            } else {
                if(left > LINE_THRESHOLD) error = -1.0f;
                else if(right > LINE_THRESHOLD) error = 1.0f;
                else error = previous_error * 0.95f;
            }

            integral += error;
            integral = (integral < -50) ? -50 : (integral > 50) ? 50 : integral;
            float derivative = error - previous_error;
            float correction = Kp*error + Ki*integral + Kd*derivative;
            
            set_motor_speeds(
                MOTOR_BASE_SPEED - correction,
                MOTOR_BASE_SPEED + correction
            );
            
            previous_error = error;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void obstacle_task(void *pvParameters) {
    init_ultrasonic();
    servo_init(SERVO_PIN);

    while(1) {
        // Trigger pulse
        gpio_set_level(TRIG_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);

        // Check distance
        if(distance_cm > 0 && distance_cm < OBSTACLE_DISTANCE_CM) {
            obstacle_detected = true;
            
            set_motor_speeds(0, 0);
            servo_set_angle(SERVO_CENTER - 60);
            vTaskDelay(pdMS_TO_TICKS(500));
            
            bool left_clear = (distance_cm >= OBSTACLE_DISTANCE_CM);
            
            servo_set_angle(SERVO_CENTER + 60);
            vTaskDelay(pdMS_TO_TICKS(500));
            
            bool right_clear = (distance_cm >= OBSTACLE_DISTANCE_CM);
            servo_set_angle(SERVO_CENTER);

            if(left_clear) {
                set_motor_speeds(-TURN_FACTOR, TURN_FACTOR);
            } else if(right_clear) {
                set_motor_speeds(TURN_FACTOR, -TURN_FACTOR);
            } else {
                set_motor_speeds(-TURN_FACTOR, -TURN_FACTOR);
                vTaskDelay(pdMS_TO_TICKS(400));
                set_motor_speeds(TURN_FACTOR, -TURN_FACTOR);
            }
            vTaskDelay(pdMS_TO_TICKS(300));
            obstacle_detected = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    init_motors();
    
    xTaskCreate(line_follow_task, "line_task", TASK_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(obstacle_task, "obstacle_task", TASK_STACK_SIZE, NULL, 4, NULL);
}
