#include "stm32f3xx.h"

#include <stdint.h>

#include "system.h"
#include "gpio.h"

int main(void) {
    System::init();
    Gpio::init();

    while(true) {
        Gpio::Led::toggle<Gpio::LD3>();
        System::delay(500);
    }
    return 0;
}


