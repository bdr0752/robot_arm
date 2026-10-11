cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME robot_arm)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    bsp/Src/bsp_board.c
    bsp/Src/bsp_dwt.c
    bsp/Src/bsp_ws2812.c
    bsp/Src/bsp_rs485.c
    driver/Src/ft_servo_bus.c
    driver/Src/ft_Servo.c
    driver/Src/ft_servo_protocol.c
    driver/Src/ws2812.c
    bsp/Src/bsp_uart.c
    Controller/Src/PID.c
    Controller/Src/servo_speed_loop.c
    app/Src/vofa.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    bsp/Inc
    driver/Inc
    Controller/Inc
    app/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    # FireWater 的 snprintf 浮点格式需要 newlib-nano 浮点打印支持。
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-u" "_printf_float")
endif()

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
