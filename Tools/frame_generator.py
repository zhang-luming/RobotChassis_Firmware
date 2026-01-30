#!/usr/bin/env python3
"""
机器人底盘通信帧生成工具（交互式）

功能：
- 生成电机速度控制帧
- 生成PID参数设置帧
- 自动计算XOR校验和
- 输出可直接使用的十六进制帧

用法:
    python3 frame_generator.py
    或者
    python3 frame_generator.py <type> <params...>

示例:
    交互模式:
        python3 frame_generator.py

    命令行模式:
        python3 frame_generator.py motor 10 10 10 10
        python3 frame_generator.py pid 1.5 0.1 0.05
"""

import sys
import os

def calculate_checksum(data):
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def generate_motor_frame(speed_a, speed_b, speed_c, speed_d):
    """
    生成电机速度控制帧

    Args:
        speed_a, speed_b, speed_c, speed_d: 4个电机的速度（centi-CPS，已×100）

    Returns:
        str: 十六进制格式的帧字符串
    """
    # 转换为整数
    speeds = [int(speed_a), int(speed_b), int(speed_c), int(speed_d)]

    # 构建帧数据（不包括校验和和帧尾）
    frame_data = [0xFC, 0x06]  # 帧头 + 功能码

    # 添加4个电机的速度（小端序int16_t）
    for speed in speeds:
        frame_data.append(speed & 0xFF)          # 低字节在前
        frame_data.append((speed >> 8) & 0xFF)   # 高字节在后

    # 计算校验和
    checksum = calculate_checksum(frame_data)

    # 添加校验和和帧尾
    frame_data.append(checksum)
    frame_data.append(0xDF)

    # 格式化为十六进制字符串
    hex_str = ' '.join([f'{byte:02X}' for byte in frame_data])

    return hex_str, checksum

def generate_pid_frame(kp, ki, kd):
    """
    生成PID参数设置帧

    Args:
        kp, ki, kd: PID参数（可以是小数，会自动×100）

    Returns:
        str: 十六进制格式的帧字符串
    """
    # 转换为协议值（×100）
    kp_val = int(kp * 100)
    ki_val = int(ki * 100)
    kd_val = int(kd * 100)

    # 构建帧数据（不包括校验和和帧尾）
    frame_data = [
        0xFC,              # 帧头
        0x07,              # 功能码：PID参数设置
        kp_val & 0xFF,         # Kp低字节（小端序）
        (kp_val >> 8) & 0xFF,  # Kp高字节
        ki_val & 0xFF,         # Ki低字节
        (ki_val >> 8) & 0xFF,  # Ki高字节
        kd_val & 0xFF,         # Kd低字节
        (kd_val >> 8) & 0xFF,  # Kd高字节
    ]

    # 计算校验和
    checksum = calculate_checksum(frame_data)

    # 添加校验和和帧尾
    frame_data.append(checksum)
    frame_data.append(0xDF)

    # 格式化为十六进制字符串
    hex_str = ' '.join([f'{byte:02X}' for byte in frame_data])

    return hex_str, kp_val, ki_val, kd_val, checksum

def print_banner():
    """打印工具横幅"""
    print("=" * 70)
    print(" " * 15 + "机器人底盘通信帧生成工具")
    print("=" * 70)

def interactive_mode():
    """交互模式"""
    print_banner()
    print()
    print("请选择功能:")
    print("  1. 电机速度控制帧")
    print("  2. PID参数设置帧")
    print("  0. 退出")
    print()

    while True:
        try:
            choice = input("请输入选项 (0-2): ").strip()

            if choice == '0':
                print("退出工具")
                break

            elif choice == '1':
                print("\n--- 电机速度控制帧生成 ---")
                print("请输入4个电机的速度（单位：centi-CPS，已×100）")
                print("示例: 10 10 10 10")
                print()

                input_str = input("请输入4个速度值（用空格分隔）: ").strip()
                speeds = input_str.split()

                if len(speeds) != 4:
                    print("错误: 必须输入4个值")
                    continue

                try:
                    # 转换为整数（电机速度通常是整数）
                    speed_a = int(speeds[0])
                    speed_b = int(speeds[1])
                    speed_c = int(speeds[2])
                    speed_d = int(speeds[3])
                    hex_str, checksum = generate_motor_frame(speed_a, speed_b, speed_c, speed_d)

                    print()
                    print("=" * 70)
                    print("电机速度控制帧生成成功")
                    print("=" * 70)
                    print(f"速度设置: A={speed_a}, B={speed_b}, C={speed_c}, D={speed_d}")
                    print(f"校验和:   0x{checksum:02X}")
                    print()
                    print("原始帧（十六进制）:")
                    print(hex_str)
                    print()
                    print("复制到串口工具:")
                    print(hex_str)
                    print("=" * 70)

                except ValueError as e:
                    print(f"错误: 输入必须是整数")
                    print(f"  {e}")

                print()

            elif choice == '2':
                print("\n--- PID参数设置帧生成 ---")
                print("请输入PID参数（支持小数，会自动×100）")
                print("示例: 1.5 0.1 0.05")
                print()

                input_str = input("请输入Kp Ki Kd（用空格分隔）: ").strip()
                params = input_str.split()

                if len(params) != 3:
                    print("错误: 必须输入3个值")
                    continue

                try:
                    # 转换为浮点数以支持小数输入
                    kp = float(params[0])
                    ki = float(params[1])
                    kd = float(params[2])
                    hex_str, kp_val, ki_val, kd_val, checksum = generate_pid_frame(kp, ki, kd)

                    print()
                    print("=" * 70)
                    print("PID参数设置帧生成成功")
                    print("=" * 70)
                    print(f"输入参数: Kp={kp}, Ki={ki}, Kd={kd}")
                    print(f"协议值:   Kp={kp_val}, Ki={ki_val}, Kd={kd_val}")
                    print(f"校验和:   0x{checksum:02X}")
                    print()
                    print("原始帧（十六进制）:")
                    print(hex_str)
                    print()
                    print("复制到串口工具:")
                    print(hex_str)
                    print("=" * 70)

                except ValueError as e:
                    print(f"错误: 输入必须是数字")
                    print(f"  {e}")

                print()

            else:
                print("错误: 无效的选项，请输入0-2")
                print()

        except KeyboardInterrupt:
            print("\n\n退出工具")
            break
        except Exception as e:
            print(f"错误: {e}")
            print()

def command_line_mode():
    """命令行模式"""
    if len(sys.argv) < 2:
        print("错误: 缺少参数")
        print("用法:")
        print("  交互模式: python3 frame_generator.py")
        print("  命令行模式:")
        print("    python3 frame_generator.py motor <A> <B> <C> <D>")
        print("    python3 frame_generator.py pid <Kp> <Ki> <Kd>")
        sys.exit(1)

    frame_type = sys.argv[1].lower()

    if frame_type == 'motor':
        if len(sys.argv) != 6:
            print("错误: 电机控制需要4个速度参数")
            print("用法: python3 frame_generator.py motor <A> <B> <C> <D>")
            sys.exit(1)

        try:
            speed_a, speed_b, speed_c, speed_d = sys.argv[2:6]
            hex_str, checksum = generate_motor_frame(speed_a, speed_b, speed_c, speed_d)

            print("电机速度控制帧:")
            print(hex_str)

        except ValueError:
            print("错误: 速度值必须是数字")
            sys.exit(1)

    elif frame_type == 'pid':
        if len(sys.argv) != 5:
            print("错误: PID设置需要3个参数")
            print("用法: python3 frame_generator.py pid <Kp> <Ki> <Kd>")
            sys.exit(1)

        try:
            kp = float(sys.argv[2])
            ki = float(sys.argv[3])
            kd = float(sys.argv[4])
            hex_str, kp_val, ki_val, kd_val, checksum = generate_pid_frame(kp, ki, kd)

            print("PID参数设置帧:")
            print(hex_str)

        except ValueError:
            print("错误: PID参数必须是数字")
            sys.exit(1)

    else:
        print(f"错误: 未知的帧类型 '{frame_type}'")
        print("支持的类型: motor, pid")
        sys.exit(1)

def main():
    # 检查运行模式
    if len(sys.argv) == 1:
        # 无参数，进入交互模式
        interactive_mode()
    else:
        # 有参数，命令行模式
        command_line_mode()

if __name__ == "__main__":
    main()
