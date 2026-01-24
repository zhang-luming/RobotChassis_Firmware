#!/usr/bin/env python3
"""
原始串口测试工具

直接控制串口，诊断发送问题
"""

import serial
import sys
import time


def test_serial_write_bytes():
    """测试直接写字节"""

    port = "/dev/ttyUSB0"
    baudrate = 115200

    print(f"打开串口: {port} @ {baudrate}")
    print("发送测试数据...\n")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )

        print(f"✓ 串口已打开: {ser.name}")

        # 测试1: 发送单个字节
        print("\n[测试1] 发送单个字节序列")
        test_bytes = [0xFC, 0x06, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0xFA, 0xDF]
        for i, b in enumerate(test_bytes):
            ser.write(bytes([b]))
            print(f"  发送: 0x{b:02X}")
            time.sleep(0.01)

        time.sleep(0.5)

        # 测试2: 一次性发送完整帧
        print("\n[测试2] 一次性发送完整帧")
        frame = bytes([0xFC, 0x06, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0xFA, 0xDF])
        print(f"  帧数据: {' '.join(f'{b:02X}' for b in frame)}")
        ser.write(frame)
        print(f"  已发送 {len(frame)} 字节")

        time.sleep(0.5)

        # 测试3: 发送多个帧
        print("\n[测试3] 发送3个完整帧")
        for i in range(3):
            ser.write(frame)
            print(f"  帧#{i+1} 已发送")
            time.sleep(0.2)

        ser.close()
        print("\n✓ 测试完成")
        return 0

    except serial.SerialException as e:
        print(f"\n✗ 串口错误: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n✗ 用户中断")
        return 1


def test_serial_with_read():
    """测试串口读写（回环测试）"""

    port = "/dev/ttyUSB0"
    baudrate = 115200

    print(f"\n[回环测试] 打开串口: {port} @ {baudrate}")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.5
        )

        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 发送测试数据
        test_data = bytes([0xFC, 0x06, 0x01, 0x02, 0x03, 0x04])
        print(f"  发送: {' '.join(f'{b:02X}' for b in test_data)}")
        ser.write(test_data)

        # 尝试读取回显（如果有）
        time.sleep(0.1)
        if ser.in_waiting > 0:
            received = ser.read(ser.in_waiting)
            print(f"  接收: {' '.join(f'{b:02X}' for b in received)}")
        else:
            print("  (无回显数据 - 正常，因为这不是回环连接)")

        ser.close()
        return 0

    except Exception as e:
        print(f"✗ 错误: {e}")
        return 1


def main():
    print("=" * 60)
    print("原始串口测试工具")
    print("=" * 60)

    ret = test_serial_write_bytes()
    if ret == 0:
        test_serial_with_read()

    return ret


if __name__ == '__main__':
    sys.exit(main())
