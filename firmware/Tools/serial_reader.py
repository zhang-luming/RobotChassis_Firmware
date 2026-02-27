#!/usr/bin/env python3
"""
机器人底盘上位机数据读取脚本

功能：
- 从串口读取数据
- 解析通信协议帧
- 打印解析后的数据

协议格式：[0xFC][FuncCode][Data...][Checksum][0xDF]
- 帧头：0xFC
- 功能码：1字节
- 数据：可变长度int16_t数组（小端序）
- 校验和：XOR校验（包括帧头、功能码和数据段）
- 帧尾：0xDF
"""

import sys
import struct
from typing import Optional, List
from datetime import datetime

# 确保导入的是 pyserial 而非标准库的 serial
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 未找到 pyserial 模块")
    print("请运行: pip install pyserial")
    sys.exit(1)

# 验证是 pyserial 而非其他 serial 模块
if not hasattr(serial, 'Serial'):
    print("错误: 导入的 serial 模块不正确")
    print("请确保安装了 pyserial: pip install pyserial")
    sys.exit(1)


# ==================== 协议定义 ====================

PROTOCOL_HEADER = 0xFC
PROTOCOL_TAIL = 0xDF

# MCU上报数据的功能码定义
FUNC_NAMES = {
    0x01: "电池电压",
    0x02: "编码器",
    0x03: "IMU数据",
}

# 数据长度定义（字节数）- 不包括8字节时间戳
FUNC_DATA_LENGTHS = {
    0x01: 2,   # 电池电压: 1个int16_t
    0x02: 16,  # 编码器: 4个int32_t = 8个int16_t
    0x03: 18,  # IMU数据: 9个int16_t (欧拉角3 + 陀螺仪3 + 加速度3)
}

# 帧格式：[FC][Func][Data...][TxTimestamp(8 bytes)][Checksum][DF]
TX_TIMESTAMP_LEN = 8  # 发送时间戳固定8字节


# ==================== 解析函数 ====================

def calculate_xor_checksum(data: bytes) -> int:
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def parse_int16_array(data: bytes, num_values: int) -> List[int]:
    """解析int16_t数组（小端序）"""
    values = []
    for i in range(num_values):
        value = struct.unpack('<h', data[i*2:(i+1)*2])[0]
        values.append(value)
    return values


def parse_frame(frame: bytes) -> Optional[dict]:
    """
    解析协议帧

    新帧格式：[FC][Func][Data...][TxTimestamp(8B)][Checksum][DF]

    返回：
        {
            'func_code': int,
            'func_name': str,
            'data': list,
            'timestamp': int (微秒时间戳),
            'raw': bytes
        }
        或 None（解析失败）
    """
    if len(frame) < 5:  # 最小帧长度
        return None

    if frame[0] != PROTOCOL_HEADER:
        return None

    if frame[-1] != PROTOCOL_TAIL:
        return None

    func_code = frame[1]
    if func_code not in FUNC_NAMES:
        print(f"  [警告] 未知功能码: 0x{func_code:02X}")
        return None

    # 数据长度 = 总帧长 - 帧头(1) - 功能码(1) - 时间戳(8) - 校验(1) - 帧尾(1)
    data_len = len(frame) - 12

    # 验证数据长度
    expected_len = FUNC_DATA_LENGTHS.get(func_code)
    if expected_len is not None and data_len != expected_len:
        print(f"  [警告] 功能码0x{func_code:02X}的数据长度不匹配: 预期{expected_len}, 实际{data_len}")

    # 计算并验证校验和（包括时间戳）
    checksum_data = frame[:-2]  # 排除校验位和帧尾
    calculated_checksum = calculate_xor_checksum(checksum_data)
    received_checksum = frame[-2]

    if calculated_checksum != received_checksum:
        print(f"  [警告] 校验和错误: 计算值0x{calculated_checksum:02X}, 接收值0x{received_checksum:02X}")
        return None

    # 解析8字节时间戳（小端序）
    timestamp_bytes = frame[2+data_len:2+data_len+8]
    timestamp = int.from_bytes(timestamp_bytes, byteorder='little')

    # 解析数据（所有功能码都使用int16_t）
    num_values = data_len // 2
    data = parse_int16_array(frame[2:2+data_len], num_values)

    return {
        'func_code': func_code,
        'func_name': FUNC_NAMES[func_code],
        'data': data,
        'timestamp': timestamp,
        'raw': frame
    }


def format_data(func_code: int, data: list) -> str:
    """格式化数据为可读字符串"""
    if func_code == 0x01:  # 电池电压
        voltage = data[0]  # mV
        return f"{voltage} mV ({voltage/1000:.2f} V)"

    elif func_code == 0x02:  # 编码器：8个int16_t组成4个int32_t
        # 重构int32值 (小端序: 低16位在前，高16位在后)
        def to_int32(low, high):
            val = (high << 16) | (low & 0xFFFF)
            if val & 0x80000000:  # 处理负数
                val = val - 0x100000000
            return val
        enc_a = to_int32(data[0], data[1])
        enc_b = to_int32(data[2], data[3])
        enc_c = to_int32(data[4], data[5])
        enc_d = to_int32(data[6], data[7])
        return f"A={enc_a:6d} B={enc_b:6d} C={enc_c:6d} D={enc_d:6d}"

    elif func_code == 0x03:  # IMU数据：欧拉角(3) + 陀螺仪(3) + 加速度(3)
        # 数据顺序：俯仰、横滚、航向、gyro(x,y,z)、accel(x,y,z)
        lines = []
        lines.append(f"Euler   : Pitch={data[0]/100:7.2f}°, Roll={data[1]/100:7.2f}°, Yaw={data[2]/100:7.2f}°")
        lines.append(f"Gyro    : X={data[3]/100:7.2f},   Y={data[4]/100:7.2f},   Z={data[5]/100:7.2f} rad/s")
        lines.append(f"Accel   : X={data[6]/100:7.2f},   Y={data[7]/100:7.2f},   Z={data[8]/100:7.2f} G")
        return "\n         ".join(lines)

    else:
        return str(data)


def format_timestamp(tx_timestamp: int, first_timestamp: int) -> str:
    """格式化时间戳为可读字符串"""
    # 将微秒转换为毫秒浮点数，保留微秒精度
    tx_ms = tx_timestamp / 1000.0
    return f"TX:{tx_ms:.3f}ms"


def print_frame(frame_info: dict, show_raw: bool = False, first_timestamp: int = 0):
    """打印帧信息"""
    func_name = frame_info['func_name']
    data_str = format_data(frame_info['func_code'], frame_info['data'])
    tx_timestamp = frame_info.get('timestamp', 0)

    ts_str = format_timestamp(tx_timestamp, first_timestamp)

    # 判断是否为多行输出（IMU数据）
    if '\n' in data_str:
        # 多行输出：时间戳只在第一行
        lines = data_str.split('\n')
        print(f"[{ts_str}] {func_name}:")
        for i, line in enumerate(lines):
            if i == 0:
                print(f"  {line}")
            else:
                print(f"  {line}")
    else:
        # 单行输出
        print(f"[{ts_str}] {func_name}: {data_str}")

    if show_raw:
        raw_hex = ' '.join(f'{b:02X}' for b in frame_info['raw'])
        print(f"  原始: {raw_hex}")


# ==================== 串口处理 ====================

class SerialReader:
    """串口读取器"""

    def __init__(self, port: str, baudrate: int = 921600):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.buffer = bytearray()

    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,  # 100ms超时
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✓ 已连接到 {self.port} ({self.baudrate} baud)")
            return True
        except serial.SerialException as e:
            print(f"✗ 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("✓ 已断开连接")

    def read_frames(self) -> list:
        """读取并解析所有完整帧"""
        if not self.serial or not self.serial.is_open:
            return []

        # 读取所有可用数据
        data = self.serial.read(self.serial.in_waiting)
        if not data:
            return []

        self.buffer.extend(data)
        frames = []

        # 查找完整帧
        while True:
            # 查找帧头
            header_idx = self.buffer.find(PROTOCOL_HEADER)
            if header_idx == -1:
                # 没有找到帧头，清空缓冲区
                self.buffer.clear()
                break

            # 丢弃帧头之前的数据
            if header_idx > 0:
                print(f"  [警告] 丢弃 {header_idx} 字节无效数据")
                del self.buffer[:header_idx]

            # 检查是否有足够的数据来判断帧长度
            if len(self.buffer) < 5:
                break

            func_code = self.buffer[1]
            expected_data_len = FUNC_DATA_LENGTHS.get(func_code, 0)
            # 帧格式：[FC][Func][Data...][TxTimestamp(8B)][Checksum][DF]
            expected_frame_len = 4 + expected_data_len + TX_TIMESTAMP_LEN  # 包含时间戳

            # 检查是否收到完整帧
            if len(self.buffer) < expected_frame_len:
                break

            # 提取一帧
            frame = bytes(self.buffer[:expected_frame_len])
            del self.buffer[:expected_frame_len]

            # 解析帧
            frame_info = parse_frame(frame)
            if frame_info:
                frames.append(frame_info)

        return frames


def list_serial_ports():
    """列出所有可用的串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用的串口设备")
        return []

    print("可用串口设备:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
        if port.manufacturer:
            print(f"     制造商: {port.manufacturer}")

    return ports


# ==================== 主程序 ====================

def main():
    import argparse

    parser = argparse.ArgumentParser(description='机器人底盘串口数据读取器')
    parser.add_argument('-p', '--port', type=str, help='串口设备 (例如: /dev/ttyUSB0 或 COM3)')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率 (默认: 921600)')
    parser.add_argument('-l', '--list', action='store_true', help='列出可用串口')
    parser.add_argument('-r', '--raw', action='store_true', help='显示原始数据')
    parser.add_argument('-f', '--filter', type=str, nargs='+',
                       choices=['battery', 'encoder', 'imu'],
                       help='只显示指定类型的数据')

    args = parser.parse_args()

    # 列出串口
    if args.list:
        list_serial_ports()
        return 0

    # 自动选择串口
    port = args.port
    if not port:
        ports = list_serial_ports()
        if len(ports) == 0:
            print("错误: 未找到可用串口，请手动指定串口设备")
            return 1
        elif len(ports) == 1:
            port = ports[0].device
            print(f"自动选择串口: {port}")
        else:
            print("错误: 找到多个串口，请手动指定")
            return 1

    # 功能码过滤器
    filter_codes = None
    if args.filter:
        filter_map = {
            'battery': 0x01,
            'encoder': 0x02,
            'imu': 0x03,
        }
        filter_codes = set(filter_map[f] for f in args.filter)

    # 连接串口
    reader = SerialReader(port, args.baudrate)
    if not reader.connect():
        return 1

    print("\n开始读取数据 (按Ctrl+C退出)...\n")

    try:
        frame_count = 0
        first_timestamp = 0  # 记录第一个时间戳
        while True:
            frames = reader.read_frames()

            for frame_info in frames:
                frame_count += 1

                # 记录第一个时间戳
                if first_timestamp == 0:
                    first_timestamp = frame_info.get('timestamp', 0)

                # 应用过滤器
                if filter_codes and frame_info['func_code'] not in filter_codes:
                    continue

                # 打印帧信息
                print_frame(frame_info, show_raw=args.raw, first_timestamp=first_timestamp)

    except KeyboardInterrupt:
        print(f"\n\n总共接收 {frame_count} 帧数据")

    finally:
        reader.disconnect()

    return 0


if __name__ == '__main__':
    sys.exit(main())
