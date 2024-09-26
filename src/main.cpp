#include "stm32f3xx.h"
#include "system.h"
#include "gpio.h"

int main(void) {
    System::init();
    Gpio::init_leds();

    while(true) {
        GPIOE->ODR |= (0xFF << 8);  // Set bits 8 to 15 (turn ON)
        System::delay(100);

        GPIOE->ODR &= ~(0xFF << 8);  // Reset bits 8 to 15 (turn OFF)
        System::delay(100);
    }
    return 0;
}


