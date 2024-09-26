#include "stm32f3xx.h"

namespace Gpio
{
    namespace Led {
        void init(void)
        {
            // Step 1: Enable clock for GPIO port E (RCC_AHBENR_GPIOEEN)
            RCC->AHBENR |= RCC_AHBENR_GPIOEEN;

            // Step 2: Set pins 8 to 15 as output (MODER)
            // Each pin requires 2 bits in MODER register (00: Input, 01: Output, 10: Alternate function, 11: Analog)
            // Set bits for pins 8 to 15 to 01 (output mode)
            GPIOE->MODER &= ~(0xFFFF << 16);  // Clear bits 16 to 31 (pins 8 to 15)
            GPIOE->MODER |= (0x5555 << 16);   // Set bits 16 to 31 to 01 (output mode for pins 8 to 15)

            // Optional: Configure output type to push-pull (default is push-pull, but can be set explicitly)
            GPIOE->OTYPER &= ~(0xFF << 8);    // Clear bits 8 to 15 (set as push-pull)

            // Optional: Set the speed of pins (Low, Medium, High, or Very High)
            GPIOE->OSPEEDR &= ~(0xFFFF << 16);  // Clear speed bits for pins 8 to 15
            GPIOE->OSPEEDR |= (0x5555 << 16);   // Set to medium speed (01 for medium speed)

            // Optional: Configure no pull-up, pull-down resistors (PUPDR)
            GPIOE->PUPDR &= ~(0xFFFF << 16);   // No pull-up, no pull-down for pins 8 to 15
        }

        void on_all(void) {
            GPIOE->ODR |= (0xFF << 8);  // Set bits 8 to 15 (turn ON)
        }
        void off_all(void) {
            GPIOE->ODR &= ~(0xFF << 8);  // Reset bits 8 to 15 (turn OFF)
        }
        void toggle_all(void) {
            GPIOE->ODR ^= (0xFF << 8);  // Reset bits 8 to 15 (turn OFF)
        }
    }

    void init(void) {
        Led::init();
    }
    // void toggle()
} // namespace gpio


