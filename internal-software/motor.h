#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// Initialize motor system
// left/right throttle pins = PWM outputs (TH pins)
// left/right dir pins      = direction control (RW pins)
// left/right en pins       = enable pins (EN pins)
void motor_init(
    int left_throttle_pin,
    int right_throttle_pin,
    int left_dir_pin,
    int right_dir_pin,
    int left_en_pin,
    int right_en_pin
);

void motor_enable(void);

void motor_disable(void);


void motor_set_speed(float linear, float angular);

#endif