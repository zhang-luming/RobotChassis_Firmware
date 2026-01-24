#!/usr/bin/env python3
"""
串口调试工具 - 显示发送的原始字节

用于验证发送的数据格式是否正确
"""

import struct
import sys


# ==================== 协议定义 ====================

PROTOCOL_HEADER = 0xFC
PROTOCOL_TAIL = 0xDF

# 功能码定义
FUNC_MOTOR_SPEED = 0x06
FUNC_PID_PARAM = 0x07
FUNC_SERVO_CONTROL = 0x08


def calculate_xor_checksum(data: bytes) -> int:
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_frame(func_code: int, data: bytes = b'') -> bytes:
    """构建协议帧"""
    frame = bytearray()
    frame.append(PROTOCOL_HEADER)
    frame.append(func_code)
    frame.extend(data)

    # 计算并添加校验和（包括帧头、功能码和数据段，不包括帧尾）
    checksum = calculate_xor_checksum(bytes(frame))
    frame.append(checksum)

    # 添加帧尾
    frame.append(PROTOCOL_TAIL)

    return bytes(frame)


def print_frame_hex(frame: bytes, label: str):
    """打印帧的十六进制表示"""
    print(f"\n{label}:")
    print(f"  长度: {len(frame)} 字节")
    print(f"  十六进制: {' '.join(f'{b:02X}' for b in frame)}")
    print(f"  详细分解:")
    print(f"    [0] 帧头:       0x{frame[0]:02X}")
    print(f"    [1] 功能码:    0x{frame[1]:02X}")

    # 打印数据部分
    data_len = len(frame) - 4  # 减去帧头、功能码、校验、帧尾
    if data_len > 0:
        print(f"    [2-{1+data_len}] 数据:  ", end='')
        for i in range(data_len):
            print(f"0x{frame[2+i]:02X} ", end='')
        print()

    print(f"    [{-2}] 校验和:   0x{frame[-2]:02X}")
    print(f"    [{-1}] 帧尾:       0x{frame[-1]:02X}")

    # 验证校验和
    calc_checksum = calculate_xor_checksum(frame[:-2])
    recv_checksum = frame[-2]
    if calc_checksum == recv_checksum:
        print(f"  ✓ 校验和正确 (0x{calc_checksum:02X})")
    else:
        print(f"  ✗ 校验和错误! 计算=0x{calc_checksum:02X}, 接收=0x{recv_checksum:02X}")


def test_motor_speed():
    """测试电机速度帧"""
    print("\n" + "=" * 60)
    print("测试1: 电机速度控制")
    print("=" * 60)

    speed = 100
    print(f"\n速度值: {speed}")

    # 构建数据：4个int16_t，大端序
    data = struct.pack('>hhhh', speed, speed, speed, speed)
    print(f"数据 (struct.pack): {data.hex()}")

    frame = build_frame(FUNC_MOTOR_SPEED, data)
    print_frame_hex(frame, "电机速度帧 (速度=100)")

    # 手动验证
    print("\n手动验证:")
    print(f"  speed = {speed} = 0x{speed:04X}")
    print(f"  高字节 = 0x{speed >> 8:02X}, 低字节 = 0x{speed & 0xFF:02X}")


def test_pid():
    """测试PID参数帧"""
    print("\n" + "=" * 60)
    print("测试2: PID参数设置")
    print("=" * 60)

    kp, ki, kd = 1.5, 0.2, 0.8
    print(f"\nPID参数: Kp={kp}, Ki={ki}, Kd={kd}")

    # 转换为协议格式（×100）
    kp_scaled = int(kp * 100)
    ki_scaled = int(ki * 100)
    kd_scaled = int(kd * 100)

    print(f"缩放后: Kp={kp_scaled}, Ki={ki_scaled}, Kd={kd_scaled}")

    # 构建数据：3个int16_t，大端序
    data = struct.pack('>hhh', kp_scaled, ki_scaled, kd_scaled)
    frame = build_frame(FUNC_PID_PARAM, data)
    print_frame_hex(frame, "PID参数帧")


def test_servo():
    """测试舵机控制帧"""
    print("\n" + "=" * 60)
    print("测试3: 舵机控制")
    print("=" * 60)

    servo1, servo2 = 90, 45
    print(f"\n舵机角度: 舵机1={servo1}°, 舵机2={servo2}°")

    # 构建数据：2个uint8_t
    data = struct.pack('BB', servo1, servo2)
    frame = build_frame(FUNC_SERVO_CONTROL, data)
    print_frame_hex(frame, "舵机控制帧")


def main():
    print("=" * 60)
    print("串口协议调试工具")
    print("=" * 60)

    test_motor_speed()
    test_pid()
    test_servo()

    print("\n" + "=" * 60)
    print("总结")
    print("=" * 60)
    print("\n期望的接收帧格式（电机速度=100）:")
    print("  FC 06 00 64 00 64 00 64 00 64 CS DF")
    print("  ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^ ^^")
    print("  |  |  |  |  |  |  |  |  |  |  |  |")
    print("  |  |  |  |  |  |  |  |  |  |  |  帧尾 (0xDF)")
    print("  |  |  |  |  |  |  |  |  |  |  校验和 (CS)")
    print("  |  |  |  |  |  |  |  |  |  速度D低字节 (0x64 = 100)")
    print("  |  |  |  |  |  |  |  |  速度D高字节 (0x00)")
    print("  |  |  |  |  |  |  |  速度C低字节 (0x64)")
    print("  |  |  |  |  |  |  速度C高字节 (0x00)")
    print("  |  |  |  |  |  速度B低字节 (0x64)")
    print("  |  |  |  |  速度B高字节 (0x00)")
    print("  |  |  |  速度A低字节 (0x64)")
    print("  |  |  速度A高字节 (0x00)")
    print("  |  功能码 (0x06)")
    print("  帧头 (0xFC)")

    # 计算校验和
    test_data = bytes([0xFC, 0x06, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64])
    cs = calculate_xor_checksum(test_data)
    print(f"\n校验和计算:")
    print(f"  0xFC ^ 0x06 ^ 0x00 ^ 0x64 ^ 0x00 ^ 0x64 ^ 0x00 ^ 0x64 ^ 0x00 ^ 0x64 = 0x{cs:02X}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
