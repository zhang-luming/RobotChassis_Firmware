#!/usr/bin/env python3
"""
RobotChassis Communication Protocol Debug Tool

STM32F103 机器人底盘通信协议调试工具
生成符合协议格式的串口数据帧，方便MCU调试

协议帧格式（12字节固定）：
  [0xFC][FuncCode][Data8Bytes][XOR][0xDF]
   帧头    功能码     数据区    校验  帧尾
    1B      1B        8B        1B    1B
"""

import sys


class ProtocolConfig:
    """协议配置常量"""
    PROTOCOL_HEADER = 0xFC  # 帧头
    PROTOCOL_TAIL = 0xDF    # 帧尾

    # 数据区实际长度是8字节（索引2-9）
    # 但XOR校验的是前10字节（帧头+功能码+数据区）
    XOR_CHECK_LEN = 10

    # 功能码定义
    FUNC_BATTERY_VOLTAGE = 0x01
    FUNC_ENCODER = 0x02
    FUNC_GYRO = 0x03
    FUNC_ACCEL = 0x04
    FUNC_EULER_ANGLE = 0x05
    FUNC_MOTOR_SPEED = 0x06
    FUNC_PID_PARAM = 0x07
    FUNC_SERVO_CONTROL = 0x08

    FUNC_NAMES = {
        0x01: "电池电压",
        0x02: "编码器",
        0x03: "陀螺仪",
        0x04: "加速度",
        0x05: "欧拉角",
        0x06: "电机目标速度",
        0x07: "PID参数设置",
        0x08: "舵机控制",
    }


def xor_checksum(data: bytes) -> int:
    """
    计算XOR校验和

    Args:
        data: 需要校验的数据字节（前10字节）

    Returns:
        XOR校验值
    """
    result = 0
    for byte in data:
        result ^= byte
    return result


def build_frame(func_code: int, data: bytes) -> bytes:
    """
    构建协议帧（12字节）

    Args:
        func_code: 功能码 (1字节)
        data: 8字节数据区

    Returns:
        完整的12字节协议帧
    """
    if len(data) != 8:
        raise ValueError(f"数据区长度必须为8字节，当前为{len(data)}字节")

    # 构建帧：帧头(1) + 功能码(1) + 数据区(8) = 10字节用于校验
    frame_for_xor = bytes([ProtocolConfig.PROTOCOL_HEADER, func_code]) + data
    checksum = xor_checksum(frame_for_xor)

    # 完整帧：帧头(1) + 功能码(1) + 数据区(8) + 校验(1) + 帧尾(1) = 12字节
    frame = frame_for_xor + bytes([checksum, ProtocolConfig.PROTOCOL_TAIL])

    return frame


def format_hex(data: bytes, sep: str = " ") -> str:
    """格式化字节为十六进制字符串"""
    return sep.join(f"0x{b:02X}" for b in data)


def format_hex_array(data: bytes) -> str:
    """格式化为C数组格式"""
    return "{" + ", ".join(f"0x{b:02X}" for b in data) + "}"


def print_frame_info(func_code: int, data: bytes, frame: bytes):
    """打印协议帧详细信息"""
    print("\n" + "="*60)
    print("协议帧生成完成")
    print("="*60)
    print(f"功能类型: {ProtocolConfig.FUNC_NAMES.get(func_code, '未知')}")
    print(f"功能码: 0x{func_code:02X}")
    print(f"\n数据区内容 (8字节):")
    print(f"  {format_hex(data)}")
    print(f"\n完整帧 (12字节):")
    print(f"  十六进制: {format_hex(frame)}")
    print(f"  C数组:    {format_hex_array(frame)}")
    print(f"\n帧结构解析:")
    print(f"  [0]  帧头:   0x{frame[0]:02X}")
    print(f"  [1]  功能码: 0x{frame[1]:02X}")
    print(f"  [2-9] 数据:  {format_hex(frame[2:10], ' ')}")
    print(f"  [10] 校验码: 0x{frame[10]:02X}  (XOR校验前10字节)")
    print(f"  [11] 帧尾:   0x{frame[11]:02X}")
    print("="*60)


def input_int16(prompt: str, min_val: int = -32768, max_val: int = 32767) -> int:
    """输入int16整数"""
    while True:
        try:
            val = int(input(prompt))
            if min_val <= val <= max_val:
                return val
            print(f"  输入超出范围 [{min_val}, {max_val}]，请重新输入")
        except ValueError:
            print("  请输入有效的整数")


def input_int8(prompt: str, min_val: int = -128, max_val: int = 127) -> int:
    """输入int8整数"""
    while True:
        try:
            val = int(input(prompt))
            if min_val <= val <= max_val:
                return val
            print(f"  输入超出范围 [{min_val}, {max_val}]，请重新输入")
        except ValueError:
            print("  请输入有效的整数")


def input_uint8(prompt: str, min_val: int = 0, max_val: int = 255) -> int:
    """输入uint8整数"""
    while True:
        try:
            val = int(input(prompt))
            if min_val <= val <= max_val:
                return val
            print(f"  输入超出范围 [{min_val}, {max_val}]，请重新输入")
        except ValueError:
            print("  请输入有效的整数")


def motor_speed_menu():
    """电机速度控制 (0x06)"""
    print("\n--- 电机速度控制 (0x06) ---")
    print("说明: 输入4个电机的目标速度")
    print("单位: encoder ticks/40ms (centi-CPS)")
    print("范围: -32768 ~ 32767\n")

    speeds = []
    for i in range(4):
        motor_name = ['A', 'B', 'C', 'D'][i]
        speed = input_int16(f"  电机{motor_name} 速度: ", -32768, 32767)
        speeds.append(speed)

    # 构建8字节数据：4个int16
    data = bytearray()
    for speed in speeds:
        data.extend(speed.to_bytes(2, byteorder='big', signed=True))

    return ProtocolConfig.FUNC_MOTOR_SPEED, bytes(data)


def pid_param_menu():
    """PID参数设置 (0x07)"""
    print("\n--- PID参数设置 (0x07) ---")
    print("说明: 设置电机PID参数（应用到所有4个电机）")
    print("范围: -32768 ~ 32767")
    print("注意: 参数值需要 ×100 (如 Kp=1.50，则输入150)")
    print("未使用字节自动填充0x00\n")

    kp = input_int16("  Kp (比例系数 ×100): ", -32768, 32767)
    ki = input_int16("  Ki (积分系数 ×100): ", -32768, 32767)
    kd = input_int16("  Kd (微分系数 ×100): ", -32768, 32767)

    # 构建8字节数据：3个int16 = 6字节，补2字节0x00
    data = bytearray()
    for param in [kp, ki, kd]:
        data.extend(param.to_bytes(2, byteorder='big', signed=True))
    data.extend(bytes(2))  # 填充

    return ProtocolConfig.FUNC_PID_PARAM, bytes(data)


def servo_control_menu():
    """舵机控制 (0x08)"""
    print("\n--- 舵机控制 (0x08) ---")
    print("说明: 设置2个舵机的角度")
    print("范围: 0 ~ 180 度")
    print("未使用字节自动填充0x00\n")

    angle1 = input_uint8("  舵机1 角度 (0-180): ", 0, 180)
    angle2 = input_uint8("  舵机2 角度 (0-180): ", 0, 180)

    # 构建8字节数据：2个uint8 = 2字节，补6字节0x00
    data = bytearray([angle1, angle2])
    data.extend(bytes(6))  # 填充

    return ProtocolConfig.FUNC_SERVO_CONTROL, bytes(data)


def custom_menu():
    """自定义帧输入"""
    print("\n--- 自定义帧输入 ---")

    func_code = input_uint8("  功能码 (0x01-0xFF): ", 0x01, 0xFF)

    print("  输入8字节数据区 (以空格分隔的8个十六进制数)")
    print("  例如: 01 02 03 04 05 06 07 08")
    while True:
        try:
            hex_str = input("  数据: ")
            hex_values = [int(x, 16) for x in hex_str.split()]
            if len(hex_values) != 8:
                print(f"  需要输入8个字节，当前为{len(hex_values)}个，请重新输入")
                continue
            data = bytes(hex_values)
            break
        except ValueError:
            print("  格式错误，请输入有效的十六进制数")

    return func_code, data


def main_menu():
    """主菜单"""
    print("\n" + "="*60)
    print("RobotChassis 通信协议调试工具")
    print("="*60)
    print("\n请选择功能类型:")
    print("  1. 电机速度控制     (0x06) - 8字节数据")
    print("  2. PID参数设置      (0x07) - 6字节数据+2字节填充")
    print("  3. 舵机控制         (0x08) - 2字节数据+6字节填充")
    print("  4. 自定义帧输入")
    print("  0. 退出")
    print("-"*60)

    choice = input("\n请输入选项 (0-4): ").strip()

    menus = {
        "1": motor_speed_menu,
        "2": pid_param_menu,
        "3": servo_control_menu,
        "4": custom_menu,
    }

    if choice == "0":
        print("\n退出程序")
        sys.exit(0)
    elif choice in menus:
        return menus[choice]()
    else:
        print("\n无效选项，请重新选择")
        return None, None


def main():
    """主函数"""
    print("\n" + "="*60)
    print("RobotChassis Communication Protocol Debug Tool")
    print("="*60)
    print("\n协议帧格式: [0xFC][FuncCode][Data8Bytes][XOR][0xDF]")
    print("完整帧长度: 12字节 (1+1+8+1+1)")

    while True:
        func_code, data = main_menu()

        if func_code is None or data is None:
            continue

        try:
            # 构建协议帧
            frame = build_frame(func_code, data)

            # 打印详细信息
            print_frame_info(func_code, data, frame)

            # 询问是否继续
            print("\n按 Enter 继续生成下一帧，输入 q 退出...")
            if input().strip().lower() == 'q':
                print("\n退出程序")
                break

        except Exception as e:
            print(f"\n错误: {e}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被中断，退出")
        sys.exit(0)
