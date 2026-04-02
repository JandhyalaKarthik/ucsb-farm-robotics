#include "pico/stdlib.h"
#include "spray.h"

int SPRAY_PIN;

void spray_init(int pin) {
    SPRAY_PIN = pin;
    gpio_init(SPRAY_PIN);
    gpio_set_dir(SPRAY_PIN, GPIO_OUT);

    // DEFAULT 
    gpio_put(SPRAY_PIN, 0);
}

void spray_activate() {
    // Step 1: switch to spray tank
    gpio_put(SPRAY_PIN, 1);

    // Step 2 fpr 500ms
    sleep_ms(500);

    // Step 3: switch back to default tank
    gpio_put(SPRAY_PIN, 0);
}