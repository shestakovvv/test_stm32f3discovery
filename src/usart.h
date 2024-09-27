#pragma once

#include <stdint.h>

namespace Usart1 {
    void init(uint32_t baud);
    void transmit(uint8_t* tx_buffer, uint32_t tx_size);
    void receive(uint8_t* rx_buffer, uint32_t rx_size);

    void on_transmit();
    void on_receive();
    void on_idle();
}