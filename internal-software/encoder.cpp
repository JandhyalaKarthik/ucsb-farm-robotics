#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "encoder.h"

// totals tickes
volatile int ticks = 0;

// store which pin we are using
int ENCODER_PIN;

void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == ENCODER_PIN) {  
        ticks++;                
    }
}

void encoder_init(int pin) {
    ENCODER_PIN = pin;

    gpio_init(ENCODER_PIN);              // turn on 
    gpio_set_dir(ENCODER_PIN, GPIO_IN);  // set as input
    gpio_pull_up(ENCODER_PIN);           // keep signal stable

    gpio_set_irq_enabled(ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_callback(&encoder_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

// get the current number of ticks
int get_ticks() {
    return ticks;
}

