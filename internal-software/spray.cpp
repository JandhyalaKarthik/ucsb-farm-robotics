#include "pico/stdlib.h"
#include "spray.h"

int SPRAY_PIN;
bool is_spraying = false;
absolute_time_t spray_end_time;

void spray_init(int pin) {
    SPRAY_PIN = pin;
    gpio_init(SPRAY_PIN);
    gpio_set_dir(SPRAY_PIN, GPIO_OUT);

    // DEFAULT 
    gpio_put(SPRAY_PIN, 0);
}

void spray_activate() {
    if (!is_spraying) {
        // Step 1: switch to spray tank
        gpio_put(SPRAY_PIN, 1);
        is_spraying = true;
        
        // Step 2: Set a target end time 500ms in the future (Non-blocking)
        spray_end_time = make_timeout_time_ms(500);
    }
}

void spray_update() {
    // Step 3: Check if we are spraying AND if the time has passed
    if (is_spraying && absolute_time_diff_us(get_absolute_time(), spray_end_time) <= 0) {
        // Switch back to default tank
        gpio_put(SPRAY_PIN, 0);
        is_spraying = false;
    }
}