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
set(CMAKE_PROJECT_NAME dreamfly2_flight)

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
        Core/Src/softi2c.c
        Components/LED/led.c
        Components/Battery/battery.c
        Components/Motor/motor.c
        Components/WS2812B/ws2812b.c

        Components/ICM20948/icm20948.c
        Components/AK09916/ak09916.c
        Components/BMP280/bmp280.c
        Components/PMW3901/
        Components/VL53L1/

        Components/Si24R1/si24r1.c

        Common/filter.c
        Common/anoprotocol.c
        Common/rcprotocol.c
        Common/attitude.c
        Common/pid.c

        Application/debug.c
        Application/flight.c
        Application/rtos.c
        Application/sensors.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Components/LED
        Components/Battery
        Components/WS2812B
        Components/Motor
        Components/ICM20948
        Components/BMP280
        Components/AK09916
        Common
        Application
        Components/Si24R1

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Enable floating-point formatting for printf/vsnprintf with nano.specs.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
