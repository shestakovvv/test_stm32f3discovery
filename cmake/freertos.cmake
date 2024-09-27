set(FREERTOS_PATH ${PROJECT_SOURCE_DIR}/lib/FreeRTOS/FreeRTOS)

# Include the necessary headers for FreeRTOS
set(FREERTOS_INCLUDES
    ${FREERTOS_PATH}/Source/include
    ${FREERTOS_PATH}/Source/portable/GCC/ARM_CM4F  # Change for your specific STM32 core (e.g., Cortex-M4)
    ${PROJECT_SOURCE_DIR}/lib/CMSIS/CMSIS/RTOS2/Include  # Change for your specific STM32 core (e.g., Cortex-M4)
)

# FreeRTOS core files
set(FREERTOS_SOURCES
    ${FREERTOS_PATH}/Source/tasks.c
    ${FREERTOS_PATH}/Source/queue.c
    ${FREERTOS_PATH}/Source/list.c
    ${FREERTOS_PATH}/Source/timers.c
    ${FREERTOS_PATH}/Source/portable/MemMang/heap_4.c  # Memory management scheme (heap_1.c, heap_2.c, heap_4.c, etc.)

    ${FREERTOS_PATH}/Source/portable/GCC/ARM_CM4F/port.c
    ${FREERTOS_PATH}/Source/portable/GCC/ARM_CM4F/portmacro.h
)
