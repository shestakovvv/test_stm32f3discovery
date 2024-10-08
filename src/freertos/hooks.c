#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  Run time stack overflow checking is performed if
 * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
 * called if a stack overflow is detected.
 *
 * @param xTask
 * @param pcTaskName
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    for (;;)
    {
    }
}
/* USER CODE END 4 */

/**
 * @brief
 * vApplicationMallocFailedHook() will only be called if
 * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
 * function that will get called if a call to pvPortMalloc() fails.
 * pvPortMalloc() is called internally by the kernel whenever a task, queue,
 * timer or semaphore is created. It is also called by various parts of the
 * demo application. If heap_1.c or heap_2.c are used, then the size of the
 * heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
 * FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
 * to query the size of free heap space that remains (although it does not
 * provide information on how the remaining heap might be fragmented).
 */
void vApplicationMallocFailedHook(void)
{
    for (;;)
    {
    }
}

