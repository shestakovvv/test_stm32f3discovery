# MARK: print_list
function(print_list)
        set(prefix PRINT)
        set(flags "")
        set(singleValues NAME)
        set(multiValues LIST)

        include(CMakeParseArguments)
        cmake_parse_arguments(
                ${prefix}
                "${flags}"
                "${singleValues}"
                "${multiValues}"
                ${ARGN}
        )
        foreach(LINE ${PRINT_LIST})
                message("${PRINT_NAME}: ${LINE}")
        endforeach()
endfunction()

# MARK: print_h1
function(print_h1 TEXT)
        message("\n########## ${TEXT} ##########")
endfunction()

# MARK: print_table
function(print_table)
        set(prefix TABLE)
        set(flags "")
        set(singleValues HEADER)
        set(multiValues ROWS)

        include(CMakeParseArguments)
        cmake_parse_arguments(
                ${prefix}
                "${flags}"
                "${singleValues}"
                "${multiValues}"
                ${ARGN}
        )

        message("|__________ ${TABLE_HEADER} ___________")
        foreach(ROW ${TABLE_ROWS})
                message("| ${ROW}")
        endforeach()
        # message("_________ END ${TABLE_HEADER} _________")
endfunction()

# MARK: helper_set_mcu_info
macro(helper_set_mcu_info _platform)
    if(${_platform} STREQUAL "stm32f3discovery")
        set(MCU_COMPANY STM32)
        set(MCU_SERIES stm32f3xx)
        set(MCU STM32F303xC)
        set(MCU_SHORT stm32f303)
    endif()

    if(${MCU} STREQUAL "STM32F303xC")
        set(MCU_FLAGS
                -mcpu=cortex-m4
                -mfloat-abi=hard
                -mfpu=fpv4-sp-d16
        )
    else()
        message(WARNING "No mcu flags")
    endif()

    set(MCU_COMPILE_OPTIONS
        ${MCU_FLAGS}
        -DSTM32F303xC
        -mthumb 
        -specs=nano.specs

        -ffunction-sections
        -fdata-sections
    )

    set(MCU_LINK_OPTIONS
        ${MCU_FLAGS}
        -mthumb
        -specs=nano.specs
        -u _printf_float
        
        -static
        -Wl,-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${_platform}_${CMAKE_BUILD_TYPE}.map,--cref
        -Wl,--gc-sections
        -Wl,-V
        -Wl,--print-memory-usage
    )
endmacro()

# MARK: helper_set_linker_file
macro(helper_set_linker_file _platform)
    file(GLOB LINKER_FILES ${PROJECT_SOURCE_DIR}/platform/${_platform}/*.ld) 
    list(GET LINKER_FILES 0 LINKER_FILE)
endmacro()

# MARK: helper_generate_cmsis_libs
function(helper_generate_cmsis_libs)
    if (NOT CMSIS_IS_GENERATED EQUAL true)
        set(CMSIS_IS_GENERATED true)

        set(prefix CMSIS)
        set(flags "")
        set(multiValues MCU_SERIESES)

        include(CMakeParseArguments)
        cmake_parse_arguments(${prefix} "${flags}" "${singleValues}" "${multiValues}" ${ARGN})

        foreach(MCU_SERIES ${CMSIS_MCU_SERIESES})
        
            add_library(${MCU_SERIES}_cmsis INTERFACE)
            message("add INTERFACE:\t${MCU_SERIES}_cmsis")

            target_include_directories(${MCU_SERIES}_cmsis INTERFACE
                ${PROJECT_SOURCE_DIR}/lib/CMSIS/CMSIS/Core/Include
                ${PROJECT_SOURCE_DIR}/lib/${MCU_SERIES}_cmsis/Include
            )
        endforeach()
    endif()
endfunction()

# MARK: helper_post_build
function(helper_post_build TARGET)
        # Print executable size
        add_custom_command(TARGET ${TARGET}
        POST_BUILD
        COMMAND arm-none-eabi-size ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET})

        # Create hex file
        add_custom_command(TARGET ${TARGET}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.bin)

        set_property(
                TARGET ${TARGET}
                APPEND
                PROPERTY ADDITIONAL_CLEAN_FILES 
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.bin
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.hex
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.map
        )
endfunction()