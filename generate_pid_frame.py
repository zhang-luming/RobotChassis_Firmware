#!/usr/bin/env python3
"""
机器人底盘PID参数帧生成工具

用法:
    python3 generate_pid_frame.py <Kp> <Ki> <Kd>

示例:
    python3 generate_pid_frame.py 1.5 0.1 0.05
    python3 generate_pid_frame.py 150 10 5

说明:
    - 参数支持小数，会自动×100转换为协议值
    - 整数直接作为协议值（已×100）
"""

import sys

def calculate_checksum(data):
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def generate_pid_frame(kp, ki, kd):
    """
    生成PID参数设置帧

    Args:
        kp: P参数（可以是小数，会自动×100）
        ki: I参数（可以是小数，会自动×100）
        kd: D参数（可以是小数，会自动×100）

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
        (kp_val >> 8) & 0xFF,  # Kp高字节
        kp_val & 0xFF,         # Kp低字节
        (ki_val >> 8) & 0xFF,  # Ki高字节
        ki_val & 0xFF,         # Ki低字节
        (kd_val >> 8) & 0xFF,  # Kd高字节
        kd_val & 0xFF,         # Kd低字节
    ]

    # 计算校验和
    checksum = calculate_checksum(frame_data)

    # 添加校验和和帧尾
    frame_data.append(checksum)
    frame_data.append(0xDF)  # 帧尾

    # 格式化为十六进制字符串
    hex_str = ' '.join([f'{byte:02X}' for byte in frame_data])

    return hex_str, kp_val, ki_val, kd_val, checksum

def main():
    if len(sys.argv) != 4:
        print("用法: python3 generate_pid_frame.py <Kp> <Ki> <Kd>")
        print()
        print("示例:")
        print("  python3 generate_pid_frame.py 1.5 0.1 0.05")
        print("  python3 generate_pid_frame.py 150 10 5")
        print()
        print("说明:")
        print("  - 参数支持小数，会自动×100转换为协议值")
        print("  - 整数直接作为协议值（已×100）")
        sys.exit(1)

    try:
        # 解析参数
        kp = float(sys.argv[1])
        ki = float(sys.argv[2])
        kd = float(sys.argv[3])

        # 生成帧
        hex_str, kp_val, ki_val, kd_val, checksum = generate_pid_frame(kp, ki, kd)

        # 输出结果
        print("=" * 60)
        print("PID参数帧生成成功")
        print("=" * 60)
        print(f"输入参数: Kp={kp}, Ki={ki}, Kd={kd}")
        print(f"协议值:   Kp={kp_val}, Ki={ki_val}, Kd={kd_val}")
        print(f"校验和:   0x{checksum:02X}")
        print()
        print("原始帧（十六进制）:")
        print(hex_str)
        print("=" * 60)

    except ValueError as e:
        print(f"错误: 参数必须是数字")
        print(f"  {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
