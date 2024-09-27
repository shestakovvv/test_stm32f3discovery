#pragma once

#include <stdint.h>

namespace System
{
    extern uint32_t CORE_CLOCK;
    void init(void);
    uint32_t get_tick(void);
    void delay(uint32_t ms);
}
