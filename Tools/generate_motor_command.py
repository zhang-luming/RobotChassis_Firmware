#!/usr/bin/env python3
"""
电机控制协议帧生成工具
用法：python generate_motor_command.py <A电机centi-CPS> <B电机centi-CPS> <C电机centi-CPS> <D电机centi-CPS>
"""

import sys

def calculate_checksum(data):
    """计算XOR校验码（只校验功能码+数据部分）"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def build_motor_frame(speed_a, speed_b, speed_c, speed_d):
    """
    构建电机控制协议帧

    参数：
        speed_a, speed_b, speed_c, speed_d: 四个电机的速度（单位：centi-CPS，即CPS/100）

    返回：
        hex字符串：协议帧的十六进制表示
        bytes：协议帧的字节流
    """
    # 构建数据帧（不含帧头、校验、帧尾）
    # 格式：[功能码][电机A高][电机A低][电机B高][电机B低][电机C高][电机C低][电机D高][电机D低]
    data = bytearray([
        0x06,                           # 功能码：电机速度控制
    ])

    # 添加四个电机速度（大端序，高字节在前）
    data.append((speed_a >> 8) & 0xFF)
    data.append(speed_a & 0xFF)
    data.append((speed_b >> 8) & 0xFF)
    data.append(speed_b & 0xFF)
    data.append((speed_c >> 8) & 0xFF)
    data.append(speed_c & 0xFF)
    data.append((speed_d >> 8) & 0xFF)
    data.append(speed_d & 0xFF)

    # 计算校验码（XOR校验）
    # 注意：校验范围包含帧头FC（MCU代码中校验buffer[0:10]）
    checksum_data = bytearray([0xFC]) + data
    checksum = calculate_checksum(checksum_data)
    data.append(checksum)

    # 添加帧头和帧尾
    frame = bytearray([0xFC]) + data + bytearray([0xDF])

    # 返回十六进制字符串和字节流
    hex_str = ' '.join([f'{b:02X}' for b in frame])
    return hex_str, frame

def main():
    if len(sys.argv) != 5:
        print("用法: python generate_motor_command.py <A电机centi-CPS> <B电机centi-CPS> <C电机centi-CPS> <D电机centi-CPS>")
        print()
        print("示例：")
        print("  python generate_motor_command.py 100 100 100 100")
        print("  输出: FC 06 00 64 00 64 00 64 00 64 FA DF")
        print()
        print("说明：")
        print("  - 输入单位是 centi-CPS（CPS/100）")
        print("  - 例如：输入100 表示实际10000 CPS")
        print("  - 根据实际硬件配置（PPR、减速比）换算RPM")
        sys.exit(1)

    try:
        # 读取四个电机的速度值
        speed_a = int(sys.argv[1])
        speed_b = int(sys.argv[2])
        speed_c = int(sys.argv[3])
        speed_d = int(sys.argv[4])

        # 构建协议帧
        hex_str, frame = build_motor_frame(speed_a, speed_b, speed_c, speed_d)

        # 输出结果
        print("协议帧（十六进制）：")
        print(hex_str)
        print()
        print("详细说明：")
        print(f"  帧头: FC")
        print(f"  功能码: 06")
        print(f"  A电机: {speed_a:5d} centi-CPS = {speed_a*100:5d} CPS (0x{speed_a:04X})")
        print(f"  B电机: {speed_b:5d} centi-CPS = {speed_b*100:5d} CPS (0x{speed_b:04X})")
        print(f"  C电机: {speed_c:5d} centi-CPS = {speed_c*100:5d} CPS (0x{speed_c:04X})")
        print(f"  D电机: {speed_d:5d} centi-CPS = {speed_d*100:5d} CPS (0x{speed_d:04X})")

        # 提取校验码
        checksum = frame[-2]
        print(f"  校验码: 0x{checksum:02X}")
        print(f"  帧尾: DF")

    except ValueError:
        print("错误：参数必须是整数")
        sys.exit(1)

if __name__ == '__main__':
    main()
