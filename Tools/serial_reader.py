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
- 数据：可变长度int16_t数组（大端序）
- 校验和：XOR校验（包括帧头、功能码和数据段）
- 帧尾：0xDF
"""

import sys
import struct
from typing import Optional, Tuple, List
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

# 功能码定义
FUNC_NAMES = {
    0x01: "电池电压",
    0x02: "编码器",
    0x03: "陀螺仪",
    0x04: "加速度",
    0x05: "欧拉角",
    0x06: "电机目标速度",
    0x07: "PID参数",
    0x08: "舵机控制",
}

# 数据长度定义（字节数）
FUNC_DATA_LENGTHS = {
    0x01: 2,  # 电池电压: 1个int16_t
    0x02: 8,  # 编码器: 4个int16_t
    0x03: 6,  # 陀螺仪: 3个int16_t
    0x04: 6,  # 加速度: 3个int16_t
    0x05: 6,  # 欧拉角: 3个int16_t
    0x06: 8,  # 电机速度: 4个int16_t
    0x07: 6,  # PID参数: 3个int16_t
    0x08: 2,  # 舵机控制: 2个int8_t
}


# ==================== 解析函数 ====================

def calculate_xor_checksum(data: bytes) -> int:
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def parse_int16_array(data: bytes, num_values: int) -> List[int]:
    """解析int16_t数组（大端序）"""
    values = []
    for i in range(num_values):
        value = struct.unpack('>h', data[i*2:(i+1)*2])[0]
        values.append(value)
    return values


def parse_frame(frame: bytes) -> Optional[dict]:
    """
    解析协议帧

    返回：
        {
            'func_code': int,
            'func_name': str,
            'data': list,
            'raw': bytes
        }
        或 None（解析失败）
    """
    if len(frame) < 5:  # 最小帧长度：帧头(1) + 功能码(1) + 数据(2) + 校验(1) + 帧尾(1)
        return None

    if frame[0] != PROTOCOL_HEADER:
        return None

    if frame[-1] != PROTOCOL_TAIL:
        return None

    func_code = frame[1]
    if func_code not in FUNC_NAMES:
        print(f"  [警告] 未知功能码: 0x{func_code:02X}")
        return None

    # 数据长度 = 总帧长 - 帧头(1) - 功能码(1) - 校验(1) - 帧尾(1)
    data_len = len(frame) - 4

    # 验证数据长度
    expected_len = FUNC_DATA_LENGTHS.get(func_code)
    if expected_len is not None and data_len != expected_len:
        print(f"  [警告] 功能码0x{func_code:02X}的数据长度不匹配: 预期{expected_len}, 实际{data_len}")

    # 计算并验证校验和（包括帧头、功能码和数据段）
    checksum_data = frame[:-2]  # 排除校验位和帧尾
    calculated_checksum = calculate_xor_checksum(checksum_data)
    received_checksum = frame[-2]

    if calculated_checksum != received_checksum:
        print(f"  [警告] 校验和错误: 计算值0x{calculated_checksum:02X}, 接收值0x{received_checksum:02X}")
        return None

    # 解析数据
    if func_code == 0x08:  # 舵机控制使用int8_t
        data = list(frame[2:2+data_len])
    else:  # 其他功能码使用int16_t
        num_values = data_len // 2
        data = parse_int16_array(frame[2:2+data_len], num_values)

    return {
        'func_code': func_code,
        'func_name': FUNC_NAMES[func_code],
        'data': data,
        'raw': frame
    }


def format_data(func_code: int, data: list) -> str:
    """格式化数据为可读字符串"""
    if func_code == 0x01:  # 电池电压
        voltage = data[0]  # mV
        return f"{voltage} mV ({voltage/1000:.2f} V)"

    elif func_code == 0x02:  # 编码器
        return f"Motor A={data[0]}, B={data[1]}, C={data[2]}, D={data[3]}"

    elif func_code == 0x03:  # 陀螺仪 (rad/s ×100)
        return f"X={data[0]/100:.2f}, Y={data[1]/100:.2f}, Z={data[2]/100:.2f} rad/s"

    elif func_code == 0x04:  # 加速度 (G ×100)
        return f"X={data[0]/100:.2f}, Y={data[1]/100:.2f}, Z={data[2]/100:.2f} G"

    elif func_code == 0x05:  # 欧拉角 (度 ×100)
        return f"Roll={data[0]/100:.2f}, Pitch={data[1]/100:.2f}, Yaw={data[2]/100:.2f}°"

    elif func_code == 0x06:  # 电机目标速度
        return f"Motor A={data[0]}, B={data[1]}, C={data[2]}, D={data[3]}"

    elif func_code == 0x07:  # PID参数 (×100)
        return f"Kp={data[0]/100:.2f}, Ki={data[1]/100:.2f}, Kd={data[2]/100:.2f}"

    elif func_code == 0x08:  # 舵机控制
        return f"Servo1={data[0]}, Servo2={data[1]}"

    else:
        return str(data)


def print_frame(frame_info: dict, show_raw: bool = False):
    """打印帧信息"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    func_name = frame_info['func_name']
    data_str = format_data(frame_info['func_code'], frame_info['data'])

    print(f"[{timestamp}] {func_name}: {data_str}")

    if show_raw:
        raw_hex = ' '.join(f'{b:02X}' for b in frame_info['raw'])
        print(f"  原始数据: {raw_hex}")


# ==================== 串口处理 ====================

class SerialReader:
    """串口读取器"""

    def __init__(self, port: str, baudrate: int = 115200):
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
            expected_frame_len = 4 + expected_data_len  # 帧头 + 功能码 + 数据 + 校验 + 帧尾

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
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='波特率 (默认: 115200)')
    parser.add_argument('-l', '--list', action='store_true', help='列出可用串口')
    parser.add_argument('-r', '--raw', action='store_true', help='显示原始数据')
    parser.add_argument('-f', '--filter', type=str, nargs='+',
                       choices=['battery', 'encoder', 'gyro', 'accel', 'euler', 'motor', 'pid', 'servo'],
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
            'gyro': 0x03,
            'accel': 0x04,
            'euler': 0x05,
            'motor': 0x06,
            'pid': 0x07,
            'servo': 0x08,
        }
        filter_codes = set(filter_map[f] for f in args.filter)

    # 连接串口
    reader = SerialReader(port, args.baudrate)
    if not reader.connect():
        return 1

    print("\n开始读取数据 (按Ctrl+C退出)...\n")

    try:
        frame_count = 0
        while True:
            frames = reader.read_frames()

            for frame_info in frames:
                frame_count += 1

                # 应用过滤器
                if filter_codes and frame_info['func_code'] not in filter_codes:
                    continue

                # 打印帧信息
                print_frame(frame_info, show_raw=args.raw)

    except KeyboardInterrupt:
        print(f"\n\n总共接收 {frame_count} 帧数据")

    finally:
        reader.disconnect()

    return 0


if __name__ == '__main__':
    sys.exit(main())
