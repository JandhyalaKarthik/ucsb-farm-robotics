#ifndef MOTOR_H
#define MOTOR_H

void motor_init(int left_pin, int right_pin);
void motor_set_speed(float linear, float angular);

#endif