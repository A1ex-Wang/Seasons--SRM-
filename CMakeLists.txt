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
set(CMAKE_PROJECT_NAME Robo)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        user/safety.c
        user/safety.h
        user/chassiscontrol.c
        user/chassiscontrol.h
        user/actuator.c
        user/actuator.h
        user/remote.c
        user/remote.h
        user/sensorimu.c
        user/sensorimu.h
        user/pose.c
        user/pose.h
        user/grab.c
        user/grab.h
        user/stepper.c
        user/stepper.h
        user/OLEDpower.c
        user/OLEDpower.h
        user/strategy.h
        user/telemetry.c
        user/telemetry.h
        user/app_config.h
        user/app_types.h
        user/debug_uart.c
        user/debug_uart.h
        Middlewares/Third_Party/SRM/srm_uart.c
        Middlewares/Third_Party/SRM/srm_remote.h
        Middlewares/Third_Party/SRM/srm_protocol.c
        Middlewares/Third_Party/SRM/srm_remote.c
        user/control/control_time.c
        user/control/control_time.h
        user/control/controller.c
        user/control/controller.h
        user/control/encode.c
        user/control/encode.h
        user/motor_driver.c
        user/motor_driver.h
        user/velocity_control.c
        user/velocity_control.h
        user/ssd1306/ssd1306.c
        user/ssd1306/ssd1306_conf.h
        user/ssd1306/ssd1306_fonts.c
        user/ssd1306/ssd1306_fonts.h
        Core/Src/MPU6050.c
        Core/Inc/MPU6050.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 应用层头文件（app_config.h / app_types.h / remote.h / ...）
    ${CMAKE_CURRENT_SOURCE_DIR}/user
    # SSD1306 OLED 驱动头文件（ssd1306.h / ssd1306_conf.h / ssd1306_fonts.h）。
    # 缺这一行时 user/OLEDpower.c 的 #include "ssd1306.h" 会“找不到文件”。
    # 注意 ssd1306_conf.h 内部还会 #include "i2c.h"，那个由 Core/Inc 提供。
    ${CMAKE_CURRENT_SOURCE_DIR}/user/ssd1306
    # 上游 SRM v4 协议层头文件（srm_remote.h / srm_protocol.h / srm_uart.h）
    # ⚠ 注意目录名是 SRM（前面没有空格）。这里原来写成 " Middlewares/..."（带一个
    #   前导空格）指向不存在的路径，会导致找不到 srm_remote.h 而编译失败。
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/SRM
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 开环标定 / 闭环控制 二选一：
    #   MOTOR_TEST_MODE = 1 → 四个轮子吃固定占空比（P1 标定用，默认）
    #   MOTOR_TEST_MODE = 0 → 速度环闭环工作
    # 不在这里定义时，以 user/app_config.h 里的值为准；
    # 想不改文件就切换，取消下一行注释即可。
    # MOTOR_TEST_MODE=0
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
