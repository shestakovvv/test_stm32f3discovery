#pragma once

#include <stdint.h>

namespace Gpio
{
    constexpr uint8_t LD3 = 9;
    constexpr uint8_t LD4 = 8;
    constexpr uint8_t LD5 = 10;
    constexpr uint8_t LD6 = 15;
    constexpr uint8_t LD7 = 11;
    constexpr uint8_t LD8 = 14;
    constexpr uint8_t LD9 = 12;
    constexpr uint8_t LD10 = 13;

    void init(void);

    namespace Led {
        template <uint8_t PIN> void on(void)
        {
            GPIOE->ODR |= (1 << PIN);
        }

        template <uint8_t PIN> void off(void)
        {
            GPIOE->ODR &= ~(1 << PIN);
        }

        template <uint8_t PIN> void toggle(void)
        {
            GPIOE->ODR ^= (1 << PIN);
        }

        void on_all(void);
        void off_all(void);
        void toggle_all(void);
    }
}