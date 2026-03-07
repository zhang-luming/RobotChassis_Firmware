#!/usr/bin/env python3
"""
简易串口监控工具 - 用于调试通信协议

功能：
- 实时显示接收到的原始字节（十六进制）
- 自动检测协议帧
- 显示帧解析结果

用法：
    python3 serial_monitor.py /dev/ttyUSB0
    python3 serial_monitor.py /dev/ttyUSB0 -b 921600
"""

import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 未找到 pyserial 模块")
    print("请运行: pip install pyserial")
    sys.exit(1)

PROTOCOL_HEADER = 0xFC
PROTOCOL_TAIL = 0xDF


def list_ports():
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用的串口设备")
        return []

    print("可用串口设备:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    return ports


def calculate_checksum(data: bytes) -> int:
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def format_hex(data: bytes, max_len=64) -> str:
    """格式化为十六进制字符串"""
    if len(data) > max_len:
        return ' '.join(f'{b:02X}' for b in data[:max_len]) + f"... ({len(data)} 字节)"
    return ' '.join(f'{b:02X}' for b in data)


def main():
    import argparse

    parser = argparse.ArgumentParser(description='串口监控工具')
    parser.add_argument('port', nargs='?', help='串口设备')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('-l', '--list', action='store_true', help='列出可用串口')
    parser.add_argument('--no-parse', action='store_true', help='不解析帧，只显示原始数据')

    args = parser.parse_args()

    if args.list or not args.port:
        list_ports()
        if not args.port:
            print("\n请指定串口设备，例如: python3 serial_monitor.py /dev/ttyUSB0")
            return 1
        return 0

    try:
        ser = serial.Serial(
            args.port,
            args.baudrate,
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print(f"✓ 已连接到 {args.port} ({args.baudrate} baud)")
        print("\n开始监控 (按Ctrl+C退出)...\n")
        print("="*70)

    except serial.SerialException as e:
        print(f"✗ 连接失败: {e}")
        return 1

    try:
        buffer = bytearray()
        frame_count = 0
        start_time = time.time()

        while True:
            # 读取数据
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                if not data:
                    continue

                # 显示原始数据
                timestamp = time.time() - start_time
                print(f"\n[{timestamp:8.3f}s] 接收 {len(data)} 字节")
                print(f"  原始: {format_hex(data)}")

                buffer.extend(data)

                # 尝试解析帧
                if not args.no_parse:
                    while True:
                        # 查找帧头
                        header_idx = buffer.find(PROTOCOL_HEADER)
                        if header_idx == -1:
                            buffer.clear()
                            break

                        # 丢弃帧头之前的数据
                        if header_idx > 0:
                            print(f"  [丢弃] {header_idx} 字节无效数据")
                            del buffer[:header_idx]

                        # 检查最小长度
                        if len(buffer) < 5:
                            break

                        func_code = buffer[1]
                        print(f"  [帧头] FC=0x{func_code:02X}", end="")

                        # 尝试查找帧尾
                        tail_idx = buffer.find(PROTOCOL_TAIL, 2)
                        if tail_idx == -1:
                            print(" (等待帧尾...)")
                            break

                        # 提取完整帧
                        frame_len = tail_idx + 1
                        frame = bytes(buffer[:frame_len])
                        del buffer[:frame_len]

                        # 计算校验和
                        calc_checksum = calculate_checksum(frame[:-2])
                        recv_checksum = frame[-2]

                        frame_count += 1
                        print(f" 长度={frame_len}B 校验={'✓' if calc_checksum == recv_checksum else '✗'}")

                        print(f"  完整帧: {format_hex(frame)}")

                        if calc_checksum != recv_checksum:
                            print(f"  [错误] 校验和: 计算=0x{calc_checksum:02X}, 接收=0x{recv_checksum:02X}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        duration = time.time() - start_time
        print("\n" + "="*70)
        print(f"\n监控结束")
        print(f"  运行时长: {duration:.1f} 秒")
        print(f"  接收帧数: {frame_count}")
        if frame_count > 0:
            print(f"  平均帧率: {frame_count/duration:.1f} fps")

    finally:
        ser.close()
        print("\n✓ 已断开连接")

    return 0


if __name__ == '__main__':
    sys.exit(main())
