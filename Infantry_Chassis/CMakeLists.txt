cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING;__FPU_USED=1U;__FPU_PRESENT=1U)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16 )
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
link_directories(Middlewares/ST/ARM/DSP/Lib)
link_libraries(libarm_cortexM4lf_math.a)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Rudder_Infantry_Chassis)

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
    User/algorithm/src/crc.cpp
    User/algorithm/src/filter.cpp
    User/algorithm/src/Fusion_AHRS.cpp
    User/algorithm/src/kalman_filter.cpp
    User/algorithm/src/PID.cpp
    User/algorithm/src/ADRC.cpp
    User/algorithm/src/user_lib.cpp
    User/algorithm/src/slope.cpp
    User/Application/src/chassis.cpp
    User/Application/src/detect.cpp
    User/Application/src/UI_task.cpp
    User/Application/src/infantry.cpp
    User/Application/src/ins.cpp
    User/bsp/src/bsp_can.c
    User/bsp/src/bsp_dwt.c
    User/bsp/src/bsp_pwm.c
    User/bsp/src/bsp_uart.c
    User/modules/src/bmi088_driver.cpp
    User/modules/src/bmi088_middleware.cpp
    User/modules/src/board_comm.cpp
    User/modules/src/cap.cpp
    User/modules/src/DJI_Motor.cpp
    User/modules/src/dr16.cpp
    User/modules/src/Motor.cpp
    User/modules/src/referee.cpp
    User/modules/src/dm_motor.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/algorithm/inc
    User/Application/inc
    User/bsp/inc
    User/modules/inc
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


if(EXISTS "${CMAKE_SOURCE_DIR}/User/algorithm/src/AHRS.lib")
    add_library(AHRS_lib STATIC IMPORTED)
    set_target_properties(AHRS_lib PROPERTIES
        IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/User/algorithm/src/AHRS.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/User/algorithm/src"
    )
    target_link_libraries(${CMAKE_PROJECT_NAME} AHRS_lib)
else()
    message(STATUS "AHRS.lib not found at Components/Algorithm/AHRS.lib; skipping AHRS_lib import")
endif()
