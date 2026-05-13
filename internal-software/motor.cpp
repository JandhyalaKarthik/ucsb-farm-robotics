#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>
#include "motor.h"

/*
    MOTOR CONTROLLER PIN USAGE

    EN  -> enable controller
    RW  -> direction
    TH  -> throttle PWM

    HIGH on RW = forward
    LOW  on RW = reverse

    PWM duty:
        0%   = stop
        100% = full speed
*/

// MOTOR PIN DEFINITIONS

static int LEFT_THROTTLE_PIN;
static int RIGHT_THROTTLE_PIN;

static int LEFT_DIR_PIN;
static int RIGHT_DIR_PIN;

static int LEFT_EN_PIN;
static int RIGHT_EN_PIN;


// PWM SETTINGS

#define PWM_WRAP 65535

// INIT

void motor_init(
    int left_throttle_pin,
    int right_throttle_pin,
    int left_dir_pin,
    int right_dir_pin,
    int left_en_pin,
    int right_en_pin
) {

    LEFT_THROTTLE_PIN = left_throttle_pin;
    RIGHT_THROTTLE_PIN = right_throttle_pin;

    LEFT_DIR_PIN = left_dir_pin;
    RIGHT_DIR_PIN = right_dir_pin;

    LEFT_EN_PIN = left_en_pin;
    RIGHT_EN_PIN = right_en_pin;

    // DIRECTION PINS

    gpio_init(LEFT_DIR_PIN);
    gpio_set_dir(LEFT_DIR_PIN, GPIO_OUT);
    gpio_put(LEFT_DIR_PIN, 1);

    gpio_init(RIGHT_DIR_PIN);
    gpio_set_dir(RIGHT_DIR_PIN, GPIO_OUT);
    gpio_put(RIGHT_DIR_PIN, 1);

    
    // ENABLE PINS

    gpio_init(LEFT_EN_PIN);
    gpio_set_dir(LEFT_EN_PIN, GPIO_OUT);

    gpio_init(RIGHT_EN_PIN);
    gpio_set_dir(RIGHT_EN_PIN, GPIO_OUT);

    // keep disabled at startup
    gpio_put(LEFT_EN_PIN, 0);
    gpio_put(RIGHT_EN_PIN, 0);

    // PWM THROTTLE PINS

    gpio_set_function(LEFT_THROTTLE_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_THROTTLE_PIN, GPIO_FUNC_PWM);

    uint left_slice = pwm_gpio_to_slice_num(LEFT_THROTTLE_PIN);
    uint right_slice = pwm_gpio_to_slice_num(RIGHT_THROTTLE_PIN);

    // configure PWM frequency
    pwm_set_wrap(left_slice, PWM_WRAP);
    pwm_set_wrap(right_slice, PWM_WRAP);

    pwm_set_clkdiv(left_slice, 4.0f);
    pwm_set_clkdiv(right_slice, 4.0f);

    // start with motors stopped
    pwm_set_gpio_level(LEFT_THROTTLE_PIN, 0);
    pwm_set_gpio_level(RIGHT_THROTTLE_PIN, 0);

    pwm_set_enabled(left_slice, true);
    pwm_set_enabled(right_slice, true);
}

// ENABLE / DISABLE

void motor_enable() {
    gpio_put(LEFT_EN_PIN, 1);
    gpio_put(RIGHT_EN_PIN, 1);
}

void motor_disable() {
    gpio_put(LEFT_EN_PIN, 0);
    gpio_put(RIGHT_EN_PIN, 0);

    pwm_set_gpio_level(LEFT_THROTTLE_PIN, 0);
    pwm_set_gpio_level(RIGHT_THROTTLE_PIN, 0);
}

// SET SINGLE MOTOR
// speed range: -1.0 to +1.0


static void set_motor(
    int throttle_pin,
    int dir_pin,
    float speed
) {

    // clamp
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // set direction
    if (speed >= 0.0f) {
        gpio_put(dir_pin, 1); // forward
    } else {
        gpio_put(dir_pin, 0); // reverse
    }

    // magnitude only for throttle
    float magnitude = fabs(speed);

    // convert to PWM
    uint16_t pwm_level = (uint16_t)(magnitude * PWM_WRAP);

    pwm_set_gpio_level(throttle_pin, pwm_level);
}

// DIFFERENTIAL DRIVE
// linear  = forward/backward
// angular = turning

void motor_set_speed(float linear, float angular) {

    float left_speed = linear - angular;
    float right_speed = linear + angular;

    // clamp
    if (left_speed > 1.0f) left_speed = 1.0f;
    if (left_speed < -1.0f) left_speed = -1.0f;

    if (right_speed > 1.0f) right_speed = 1.0f;
    if (right_speed < -1.0f) right_speed = -1.0f;

    set_motor(
        LEFT_THROTTLE_PIN,
        LEFT_DIR_PIN,
        left_speed
    );

    set_motor(
        RIGHT_THROTTLE_PIN,
        RIGHT_DIR_PIN,
        right_speed
    );
}