#include "usart.h"
#include "stm32f3xx.h"
#include "system.h"

#include "gpio.h"

namespace Usart1 {
    void enable_clocks(void)
    {
        // Enable clock for GPIOC (for PC4 and PC5)
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
        // Enable clock for USART1
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        // Enable clock for DMA1 (USART1 is connected to DMA1)
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    }

    void gpio_init(void)
    {
        // Set PC4 (TX) and PC5 (RX) to alternate function mode
        GPIOC->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
        GPIOC->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);  // Alternate function mode (10)

        // Set alternate function for USART1 on PC4 (AF7 for TX) and PC5 (AF7 for RX)
        GPIOC->AFR[0] |= (0x7 << GPIO_AFRL_AFRL4_Pos);  // AF7 for PC4
        GPIOC->AFR[0] |= (0x7 << GPIO_AFRL_AFRL5_Pos);  // AF7 for PC5

        // Set pins as push-pull
        GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5);  // Push-pull

        // Set high speed for both pins
        GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5);  // High speed

        // No pull-up, pull-down resistors
        GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);  // No pull-up/pull-down
    }

    void usart_init(uint32_t baud)
    {
        // Set baud rate: Assuming system clock of 72 MHz and baud rate of 9600
        USART1->BRR = (System::CORE_CLOCK / baud);

        // Enable USART, set to 8 data bits, 1 stop bit, no parity
        USART1->CR1 &= ~USART_CR1_M;       // 8 data bits
        USART1->CR1 &= ~USART_CR1_PCE;     // No parity
        USART1->CR2 &= ~USART_CR2_STOP;    // 1 stop bit

        // Enable DMA mode for reception and transmission
        USART1->CR3 |= USART_CR3_DMAT;     // Enable DMA for transmission
        USART1->CR3 |= USART_CR3_DMAR;     // Enable DMA for reception

        USART1->CR1 |= USART_CR1_IDLEIE; // Enable IDLE

        // Enable USART1
        USART1->CR1 |= USART_CR1_UE;
    }

    void transmit(uint8_t* tx_buffer, uint32_t tx_size) {
        USART1->CR1 |= USART_CR1_TE;

        DMA1_Channel4->CCR &= ~DMA_CCR_EN;          // Disable DMA before configuration
        DMA1_Channel4->CPAR = (uint32_t)&USART1->TDR;   // Peripheral address (USART1 TDR)
        DMA1_Channel4->CMAR = (uint32_t)tx_buffer;      // Memory address (TX buffer)
        DMA1_Channel4->CNDTR = tx_size;                 // Number of data to transfer
        DMA1_Channel4->CCR = DMA_CCR_MINC               // Memory increment mode
                            | DMA_CCR_DIR              // Memory-to-peripheral
                            | DMA_CCR_TCIE             // Transfer complete interrupt
                            | DMA_CCR_PL_1;            // Priority level (high)

        NVIC_SetPriority(DMA1_Channel4_IRQn, 5);
        NVIC_EnableIRQ(DMA1_Channel4_IRQn);             // Enable interrupt for TX
        DMA1_Channel4->CCR |= DMA_CCR_EN;               // Enable DMA1 Channel 4 (TX)
    }

    void receive(uint8_t* rx_buffer, uint32_t rx_size)
    {
        USART1->CR1 |= USART_CR1_RE;

        DMA1_Channel5->CCR &= ~DMA_CCR_EN;          // Disable DMA before configuration
        DMA1_Channel5->CPAR = (uint32_t)&USART1->RDR;   // Peripheral address (USART1 RDR)
        DMA1_Channel5->CMAR = (uint32_t)rx_buffer;      // Memory address (RX buffer)
        DMA1_Channel5->CNDTR = rx_size;                 // Number of data to transfer
        DMA1_Channel5->CCR = DMA_CCR_MINC               // Memory increment mode
                            | DMA_CCR_TCIE             // Transfer complete interrupt
                            | DMA_CCR_PL_1;            // Priority level (high)

        NVIC_SetPriority(DMA1_Channel5_IRQn, 5);
        NVIC_EnableIRQ(DMA1_Channel5_IRQn);             // Enable interrupt for RX
        DMA1_Channel5->CCR |= DMA_CCR_EN;               // Enable DMA1 Channel 5 (RX)
    }

    extern "C" void DMA1_Channel4_IRQHandler(void) // TX
    {
        if (DMA1->ISR & DMA_ISR_TCIF4)
        {
            DMA1->IFCR |= DMA_IFCR_CTCIF4;
            USART1->CR1 &= ~USART_CR1_TE;
            on_transmit();
        }
    }

    extern "C" void DMA1_Channel5_IRQHandler(void) // RX
    {
        if (DMA1->ISR & DMA_ISR_TCIF5)
        {
            DMA1->IFCR |= DMA_IFCR_CTCIF5;
            USART1->CR1 &= ~USART_CR1_RE;
            on_receive();
        }
    }

    extern "C" void USART1_IRQHandler(void) // RX
    {
        if (USART1->ISR & USART_ISR_IDLE) {
            USART1->ICR &= ~USART_ICR_IDLECF;
            on_idle();
        }
    }

    void __attribute__((weak)) on_transmit() {
    }
    void __attribute__((weak)) on_receive() {
    }
    void __attribute__((weak)) on_idle() {
    }


    void init(uint32_t baud) {
        enable_clocks();
        gpio_init();
        usart_init(baud);
    }
}