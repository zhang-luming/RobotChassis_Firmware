#!/usr/bin/env python3
"""
电机控制协议帧生成工具（支持RPM输入）
用法：python generate_motor_command_with_rpm.py <A电机RPM> <B电机RPM> <C电机RPM> <D电机RPM> <PPR> <减速比>
"""

import sys

def rpm_to_centi_cps(rpm, ppr, gear_ratio, encoder_4x=4):
    """
    RPM转换为centi-CPS

    参数：
        rpm: 目标转速（RPM）
        ppr: 编码器线数（每转脉冲数）
        gear_ratio: 减速比（如30表示30:1）
        encoder_4x: 4倍频计数（默认4）

    返回：
        centi-CPS值（CPS/100）
    """
    # 计算实际CPS
    cps = rpm * ppr * encoder_4x * gear_ratio / 60
    # 转换为centi-CPS
    centi_cps = int(cps / 100)
    return centi_cps

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
    if len(sys.argv) != 7:
        print("用法: python generate_motor_command_with_rpm.py <A电机RPM> <B电机RPM> <C电机RPM> <D电机RPM> <PPR> <减速比>")
        print()
        print("示例：")
        print("  python generate_motor_command_with_rpm.py 50 50 50 50 500 30")
        print()
        print("参数说明：")
        print("  - A/B/C/D电机RPM: 目标转速（转/分钟）")
        print("  - PPR: 编码器线数（如500表示500线编码器）")
        print("  - 减速比: 减速器减速比（如30表示30:1）")
        print()
        print("常用配置参考：")
        print("  - 小型电机: PPR=13, 减速比=1")
        print("  - 中型电机: PPR=500, 减速比=30")
        print("  - 高精度电机: PPR=1000, 减速比=20")
        sys.exit(1)

    try:
        # 读取参数
        rpm_a = float(sys.argv[1])
        rpm_b = float(sys.argv[2])
        rpm_c = float(sys.argv[3])
        rpm_d = float(sys.argv[4])
        ppr = int(sys.argv[5])
        gear_ratio = int(sys.argv[6])

        # 转换RPM到centi-CPS
        speed_a = rpm_to_centi_cps(rpm_a, ppr, gear_ratio)
        speed_b = rpm_to_centi_cps(rpm_b, ppr, gear_ratio)
        speed_c = rpm_to_centi_cps(rpm_c, ppr, gear_ratio)
        speed_d = rpm_to_centi_cps(rpm_d, ppr, gear_ratio)

        # 构建协议帧
        hex_str, frame = build_motor_frame(speed_a, speed_b, speed_c, speed_d)

        # 输出结果
        print("=" * 70)
        print("输入参数：")
        print(f"  目标转速: A={rpm_a} RPM, B={rpm_b} RPM, C={rpm_c} RPM, D={rpm_d} RPM")
        print(f"  硬件配置: PPR={ppr}, 减速比={gear_ratio}:1, 4倍频")
        print()
        print("转换结果：")
        print(f"  A电机: {rpm_a} RPM -> {speed_a} centi-CPS ({speed_a*100} CPS)")
        print(f"  B电机: {rpm_b} RPM -> {speed_b} centi-CPS ({speed_b*100} CPS)")
        print(f"  C电机: {rpm_c} RPM -> {speed_c} centi-CPS ({speed_c*100} CPS)")
        print(f"  D电机: {rpm_d} RPM -> {speed_d} centi-CPS ({speed_d*100} CPS)")
        print()
        print("协议帧（十六进制）：")
        print(hex_str)
        print("=" * 70)

    except ValueError as e:
        print(f"错误：参数格式不正确 - {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
