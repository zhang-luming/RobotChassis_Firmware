cmake_minimum_required(VERSION 3.22)

#
# 本文件仅生成一次
# 如果多次调用转换器，不会重新生成
#
# 用户可以根据需要自由修改此文件
#

# 配置编译器设置
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# 定义构建类型
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# 设置项目名称
set(CMAKE_PROJECT_NAME RobotChassis_Firmware)

# 启用编译命令以支持使用 clangd 等工具进行索引
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 核心项目设置
project(${CMAKE_PROJECT_NAME})
message("构建类型: " ${CMAKE_BUILD_TYPE})

# 启用 CMake 对 ASM 和 C 语言的支持
enable_language(C ASM)

# 创建可执行文件类型
add_executable(${CMAKE_PROJECT_NAME})

# 添加 STM32CubeMX 生成的源文件
add_subdirectory(cmake/stm32cubemx)

# 链接目录设置
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # 在此添加用户定义的库搜索路径
)

# 添加源文件到可执行文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # ==================== MPU6050 驱动 ====================
        USER/IMU/MPU6050/mpu6050.c
        USER/IMU/MPU6050/mpuiic.c
        USER/IMU/MPU6050/eMPL/inv_mpu.c
        USER/IMU/MPU6050/eMPL/inv_mpu_dmp_motion_driver.c

        # ==================== 系统支持 ====================
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/EXIT/EXIT.c
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/System/retarget.c
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/user_config.c

        # ==================== 功能模块 ====================
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/Communication/comm_protocol.c
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/MotorControl/motor_control.c
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/PowerManagement/power_management.c
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/ServoControl/servo_control.c
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/IMU/imu_data.c
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/LedControl/led_control.c
)

# 添加头文件包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # ==================== 根目录 ====================
        ${CMAKE_CURRENT_SOURCE_DIR}/USER

        # ==================== MPU6050 ====================
        USER/IMU/MPU6050
        USER/IMU/MPU6050/eMPL

        # ==================== 系统支持 ====================
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/EXIT
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/System

        # ==================== 功能模块 ====================
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/Communication
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/MotorControl
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/PowerManagement
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/ServoControl
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/IMU
        ${CMAKE_CURRENT_SOURCE_DIR}/USER/LedControl
)

# 添加项目符号（宏定义）
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # 在此添加用户定义的符号
)

# 使用 cpp 文件时移除错误的 libob.a 库依赖
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 添加链接库
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        # 在此添加用户定义的库
)

# ==================== 生成固件输出文件 ====================
# 设置输出目录
set(OUTPUT_DIR ${CMAKE_BINARY_DIR}/output)
file(MAKE_DIRECTORY ${OUTPUT_DIR})

# 生成 .hex 文件 (Intel HEX 格式)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Building ${CMAKE_PROJECT_NAME}.hex"
)

# 生成 .bin 文件 (纯二进制格式)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Building ${CMAKE_PROJECT_NAME}.bin"
)

# 显示输出文件信息
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "─────────────────────────────────────────"
    COMMAND ${CMAKE_COMMAND} -E echo "固件输出文件:"
    COMMAND ${CMAKE_COMMAND} -E echo "  ELF: $<TARGET_FILE:${CMAKE_PROJECT_NAME}>"
    COMMAND ${CMAKE_COMMAND} -E echo "  HEX: ${OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.hex"
    COMMAND ${CMAKE_COMMAND} -E echo "  BIN: ${OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.bin"
    COMMAND ${CMAKE_COMMAND} -E echo "─────────────────────────────────────────"
)
