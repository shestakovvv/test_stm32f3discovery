#include "stm32f3xx.h"

#include <stdint.h>

#include "system.h"
#include "gpio.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

osThreadId_t OS_MAIN_TASK_HANDLE;
const osThreadAttr_t OS_MAIN_TASK_ATTRS = {"os_main", 0, nullptr, 0, nullptr, 128 * 4, osPriorityNormal, 0, 0};


void os_main(void *argument)
{
    (void)(argument);
    Gpio::init();
    Usart1::init(115200);

    uint8_t buf[] = "hello mir!\n";

    for (;;)
    {
        Usart1::transmit(buf, sizeof(buf));
        Gpio::Led::toggle<Gpio::LD5>();
        osDelay(1000);
    }
}

void Usart1::on_transmit(void) {
    Gpio::Led::toggle<Gpio::LD6>();
}


int main(void)
{
    System::init();
    
    osKernelInitialize();
    OS_MAIN_TASK_HANDLE = osThreadNew(os_main, NULL, &OS_MAIN_TASK_ATTRS);
    osKernelStart();

    while (true)
    {
    }
    return 0;
}


