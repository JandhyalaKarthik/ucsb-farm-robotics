#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motor.h"

int LEFT_MOTOR_PIN;
int RIGHT_MOTOR_PIN;

void motor_init(int left_pin, int right_pin) {
    LEFT_MOTOR_PIN = left_pin;
    RIGHT_MOTOR_PIN = right_pin;

    gpio_set_function(LEFT_MOTOR_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR_PIN, GPIO_FUNC_PWM);

    // PWM slice numbers
    uint slice_left = pwm_gpio_to_slice_num(LEFT_MOTOR_PIN);
    uint slice_right = pwm_gpio_to_slice_num(RIGHT_MOTOR_PIN);

    // enable PWM
    pwm_set_enabled(slice_left, true);
    pwm_set_enabled(slice_right, true);
}

// set motor speed
// linear = forward/backward
// angular = turning
void motor_set_speed(float linear, float angular) {
    float left_speed = linear - angular;
    float right_speed = linear + angular;

    // clamp values between -1 and 1
    if (left_speed > 1) left_speed = 1;
    if (left_speed < -1) left_speed = -1;
    if (right_speed > 1) right_speed = 1;
    if (right_speed < -1) right_speed = -1;

    
// check in // convert to PWM (0–65535)
    uint16_t left_pwm = (uint16_t)((left_speed + 1) * 32767);
    uint16_t right_pwm = (uint16_t)((right_speed + 1) * 32767);

    pwm_set_gpio_level(LEFT_MOTOR_PIN, left_pwm);
    pwm_set_gpio_level(RIGHT_MOTOR_PIN, right_pwm);
}