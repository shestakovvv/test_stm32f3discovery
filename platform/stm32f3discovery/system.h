#pragma once

#include <stdint.h>

namespace System
{
    void init(void);
    uint32_t get_tick(void);
    void delay(uint32_t ms);
}
