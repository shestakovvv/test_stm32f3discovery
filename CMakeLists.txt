cmake_minimum_required(VERSION 3.26)
project(test_stm32f3discovery)

enable_language(CXX C ASM)

set(CMAKE_C_FLAGS "-std=gnu17")
set(CMAKE_C_FLAGS_DEBUG "-O3 -g3 -gdwarf-2")
set(CMAKE_C_FLAGS_RELEASE "-O3 -g0")

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -gdwarf-2")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g0")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../bin)

include(${PROJECT_SOURCE_DIR}/cmake/helper.cmake)

helper_generate_cmsis_libs(MCU_SERIESES stm32f3xx)

set(PLATFORM stm32f3discovery)

helper_set_mcu_info(${PLATFORM})
helper_set_linker_file(${PLATFORM})

set(TARGET_EXECUTABLE ${PLATFORM}_${CMAKE_BUILD_TYPE})

print_h1("Build app: ${TARGET_EXECUTABLE}")
print_table(HEADER "MCU INFO" ROWS
        "mcu company:\t${MCU_COMPANY}"
        "mcu series:\t${MCU_SERIES}"
        "mcu:\t\t\t${MCU}"
        "mcu short:\t${MCU_SHORT}"
)
message("| mcu flags:\t${MCU_FLAGS}")

file(GLOB_RECURSE SOURCES 
    ${PROJECT_SOURCE_DIR}/platform/${PLATFORM}/*(.s|.c|.cpp|.h)    
    ${PROJECT_SOURCE_DIR}/src/*(.s|.c|.cpp|.h)    
)

add_executable(${TARGET_EXECUTABLE}
    ${SOURCES}
)

# MARK: includes
target_include_directories(${TARGET_EXECUTABLE} PRIVATE
    ${PROJECT_SOURCE_DIR}/platform/${PLATFORM}
    ${PROJECT_SOURCE_DIR}/src
)

# MARK: libs
target_link_libraries(${TARGET_EXECUTABLE} PRIVATE
    stm32f3xx_cmsis
)

# MARK: compile definitions
target_compile_definitions(${TARGET_EXECUTABLE} PUBLIC
    ${TARGET_COMPILE_DEFINITIONS}
    -D${MCU}
)

# MARK: compile options
target_compile_options(${TARGET_EXECUTABLE} PRIVATE
    ${MCU_COMPILE_OPTIONS}
    -Wall
    -Wextra
    -Wpedantic
    -Werror
    -Wconversion
    -fstack-usage
)

# MARK: link options
target_link_options(${TARGET_EXECUTABLE} PRIVATE
    -T${LINKER_FILE}
    ${MCU_LINK_OPTIONS}
)

helper_post_build(${TARGET_EXECUTABLE})