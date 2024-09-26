#include "stm32f3xx.h"

namespace System {
  #if !defined  (HSE_VALUE) 
    #define HSE_VALUE    ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz.
                                                  This value can be provided and adapted by the user application. */
  #endif /* HSE_VALUE */

  #if !defined  (HSI_VALUE)
    #define HSI_VALUE    ((uint32_t)8000000) /*!< Default value of the Internal oscillator in Hz.
                                                  This value can be provided and adapted by the user application. */
  #endif /* HSI_VALUE */


  #if defined(USER_VECT_TAB_ADDRESS)
  /*!< Uncomment the following line if you need to relocate your vector Table
      in Sram else user remap will be done in Flash. */
  /* #define VECT_TAB_SRAM */
  #if defined(VECT_TAB_SRAM)
  #define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                      This value must be a multiple of 0x200. */
  #define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                      This value must be a multiple of 0x200. */
  #else
  #define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                      This value must be a multiple of 0x200. */
  #define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                      This value must be a multiple of 0x200. */
  #endif /* VECT_TAB_SRAM */
  #endif /* USER_VECT_TAB_ADDRESS */

  uint32_t CORE_CLOCK = 72000000;

  const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
  const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

  extern "C" void SystemInit(void)
  {
  /* FPU settings --------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

    /* Configure the Vector Table location -------------------------------------*/
  #if defined(USER_VECT_TAB_ADDRESS)
    SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
  #endif /* USER_VECT_TAB_ADDRESS */
  }

void core_clock_update(void)
  {
    // 1. Enable the HSE (High-Speed External) clock
      RCC->CR |= RCC_CR_HSEON;  // Turn on HSE

      // 2. Wait until HSE is ready
      while (!(RCC->CR & RCC_CR_HSERDY));  // Wait until HSE is stable and ready

      // 3. Configure the PLL:
      //    PLL source is HSE
      //    PLL multiplier is 9 (8 MHz * 9 = 72 MHz)
      RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);  // Clear PLL source and multiplier settings
      RCC->CFGR |= (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL9);  // Set HSE as PLL source and set multiplier to 9

      // 4. Enable the PLL
      RCC->CR |= RCC_CR_PLLON;  // Turn on PLL

      // 5. Wait until the PLL is ready
      while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait until PLL is locked and ready

      // 6. Configure Flash memory latency:
      //    Set 2 wait states (Latency = 2) for 72 MHz operation
      FLASH->ACR |= FLASH_ACR_LATENCY_2;

      // 7. Select PLL as the system clock source
      RCC->CFGR &= ~(RCC_CFGR_SW);  // Clear system clock switch bits
      RCC->CFGR |= RCC_CFGR_SW_PLL;  // Set PLL as system clock source

      // 8. Wait until the system clock has switched to PLL
      while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait until PLL is used as system clock

      // 9. Configure AHB, APB1, and APB2 clock dividers:
      //    AHB = SYSCLK (72 MHz), APB1 = SYSCLK/2 (36 MHz), APB2 = SYSCLK (72 MHz)
      RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear AHB, APB1, and APB2 prescaler bits
      RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // Set APB1 prescaler to divide by 2 (36 MHz for APB1)
      // APB2 and AHB are already at SYSCLK, so no need to modify the defaults
  }

  volatile uint32_t TICK = 0;  // Global tick count (incremented by SysTick interrupt)

  extern "C" void SysTick_Handler(void)
  {
    TICK++;
  }

  // Function to initialize SysTick timer to generate interrupts every 1 ms
  void init_tick(void)
  {
      // Configure SysTick to interrupt every 1 ms (assuming system clock is 72 MHz)
      SysTick->LOAD = (CORE_CLOCK / 1000) - 1;  // Set reload register for 1 ms (72 MHz / 1000 = 72,000)
      SysTick->VAL = 0;                              // Reset the SysTick counter value
      SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |    // Use processor clock (AHB)
                      SysTick_CTRL_TICKINT_Msk |      // Enable SysTick interrupt
                      SysTick_CTRL_ENABLE_Msk;        // Enable SysTick timer
  }

  uint32_t get_tick(void) {
    return TICK;
  }

  void delay(uint32_t ms) {
    uint32_t start = get_tick();
    while(TICK - start < ms) {}
  }

  void init(void) {
    core_clock_update();
    init_tick();
  }
}