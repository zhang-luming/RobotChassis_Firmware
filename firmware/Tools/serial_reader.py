#!/usr/bin/env python3
"""
机器人底盘上位机数据读取工具

功能：
- 从串口读取并解析通信协议帧
- 支持所有MCU上报数据类型（电池/编码器/IMU/PTP）
- 实时统计和频率监控
- 数据记录到文件
- 友好的显示格式

协议帧格式：
  MCU→PC（新格式，带时间戳）：
    [0xFC][FuncCode][Data...][TxTimestamp(8B)][Checksum][0xDF]

  PC→MCU（控制指令，不带时间戳）：
    [0xFC][FuncCode][Data...][Checksum][0xDF]

字节序：Little-Endian（小端序）
校验和：XOR校验所有数据字节（MCU上报包括时间戳）
"""

import sys
import struct
import csv
from typing import Optional, List, Dict, Any
from collections import defaultdict
import time

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
class FuncCode:
    SENSOR_MERGED = 0x20    # 合并传感器数据（编码器4 + IMU9）(MCU→PC)
    MOTOR_SPEED = 0x31      # 电机目标速度 (PC→MCU)
    PID_PARAM = 0x32        # PID参数设置 (PC→MCU)
    PTP_SYNC = 0x10         # PTP时间同步 (双向)

# 功能码名称映射
FUNC_NAMES = {
    FuncCode.SENSOR_MERGED: "传感器数据",
    FuncCode.MOTOR_SPEED: "电机速度",  # PC→MCU
    FuncCode.PID_PARAM: "PID参数",    # PC→MCU
    FuncCode.PTP_SYNC: "PTP同步",
}

# MCU上报数据的数据长度定义（字节数，不包括8字节时间戳）
MCU_DATA_LENGTHS = {
    FuncCode.SENSOR_MERGED: 26,      # 13×int16（编码器4 + IMU9）
    FuncCode.PTP_SYNC: 8,            # 4×int16 → t2时间戳
}

TX_TIMESTAMP_LEN = 8  # 发送时间戳固定8字节


# ==================== 协议帧类 ====================

class ProtocolFrame:
    """协议帧解析结果"""

    def __init__(self, func_code: int, data: List[int],
                 tx_timestamp: int, raw: bytes, is_valid: bool = True):
        self.func_code = func_code
        self.func_name = FUNC_NAMES.get(func_code, f"未知(0x{func_code:02X})")
        self.data = data
        self.tx_timestamp = tx_timestamp
        self.raw = raw
        self.is_valid = is_valid
        self.error_msg = ""

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            'func_code': f"0x{self.func_code:02X}",
            'func_name': self.func_name,
            'data': self.data,
            'tx_timestamp_us': self.tx_timestamp,
            'is_valid': self.is_valid,
            'error': self.error_msg
        }


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


def parse_int64_timestamp(data: bytes) -> int:
    """解析64位时间戳（小端序）"""
    return int.from_bytes(data, byteorder='little')


def reconstruct_int32(low: int, high: int) -> int:
    """将两个int16_t重构为int32_t（小端序：低16位在前，高16位在后）"""
    val = (high << 16) | (low & 0xFFFF)
    if val & 0x80000000:  # 处理负数
        val = val - 0x100000000
    return val


def parse_frame(frame: bytes) -> Optional[ProtocolFrame]:
    """
    解析协议帧

    新帧格式（MCU上报）：[FC][Func][Data...][TxTimestamp(8B)][Checksum][DF]
    旧帧格式（PC下发）：[FC][Func][Data...][Checksum][DF]

    返回：ProtocolFrame对象或None（解析失败）
    """
    if len(frame) < 5:  # 最小帧长度
        return None

    if frame[0] != PROTOCOL_HEADER:
        return None

    if frame[-1] != PROTOCOL_TAIL:
        return None

    func_code = frame[1]

    # 判断是否为MCU上报帧（带时间戳）
    # MCU上报帧通常较长（最少14字节），且功能码在MCU_DATA_LENGTHS中
    has_timestamp = func_code in MCU_DATA_LENGTHS

    if has_timestamp:
        # MCU上报帧：[FC][Func][Data...][TxTimestamp(8B)][Checksum][DF]
        expected_data_len = MCU_DATA_LENGTHS.get(func_code, 0)
        expected_frame_len = 4 + expected_data_len + TX_TIMESTAMP_LEN

        if len(frame) != expected_frame_len:
            return None

        data_len = expected_data_len
        checksum_data = frame[:-2]  # 包括时间戳
    else:
        # PC下发帧或其他帧：[FC][Func][Data...][Checksum][DF]
        data_len = len(frame) - 4  # 减去FC、Func、Checksum、DF
        checksum_data = frame[:-2]

    # 计算并验证校验和
    calculated_checksum = calculate_xor_checksum(checksum_data)
    received_checksum = frame[-2]

    if calculated_checksum != received_checksum:
        frame_obj = ProtocolFrame(func_code, [], 0, frame, is_valid=False)
        frame_obj.error_msg = f"校验和错误: 计算值0x{calculated_checksum:02X}, 接收值0x{received_checksum:02X}"
        return frame_obj

    # 解析数据
    num_values = data_len // 2
    data = parse_int16_array(frame[2:2+data_len], num_values)

    # 解析时间戳（如果有）
    tx_timestamp = 0
    if has_timestamp:
        tx_timestamp = parse_int64_timestamp(frame[2+data_len:2+data_len+8])

    return ProtocolFrame(func_code, data, tx_timestamp, frame)


# ==================== 格式化输出 ====================

def format_battery(data: List[int]) -> str:
    """格式化电池电压数据"""
    voltage = data[0]  # mV
    return f"{voltage} mV ({voltage/1000:.2f} V)"


def format_encoder(data: List[int]) -> str:
    """格式化编码器数据"""
    # 4个int16_t直接表示编码器计数值（0-65535）
    # 上位机自行处理溢出（65535→0）
    return f"A={data[0]:6d} B={data[1]:6d} C={data[2]:6d} D={data[3]:6d}"


def format_imu(data: List[int]) -> str:
    """格式化IMU数据"""
    lines = []
    lines.append(f"欧拉角: Pitch={data[0]/100:7.2f}°, Roll={data[1]/100:7.2f}°, Yaw={data[2]/100:7.2f}°")
    lines.append(f"陀螺仪: X={data[3]/100:7.2f}, Y={data[4]/100:7.2f}, Z={data[5]/100:7.2f} rad/s")
    lines.append(f"加速度: X={data[6]/100:7.2f}, Y={data[7]/100:7.2f}, Z={data[8]/100:7.2f} G")
    return "\n       ".join(lines)


def format_ptp_sync(data: List[int]) -> str:
    """格式化PTP同步数据"""
    # 4个int16_t组成64位t2时间戳
    t2 = (data[3] << 48) | (data[2] << 32) | (data[1] << 16) | data[0]
    return f"t2={t2} us ({t2/1e6:.6f} s)"


def format_encoder(data: List[int]) -> str:
    """格式化编码器数据"""
    # data包含4个int16_t：电机A/B/C/D的编码器位置
    # 上位机自行处理溢出（65535→0）
    return f"A={data[0]:6d} B={data[1]:6d} C={data[2]:6d} D={data[3]:6d}"


def format_imu(data: List[int]) -> str:
    """格式化IMU数据"""
    lines = []
    lines.append(f"欧拉角: Pitch={data[0]/100:7.2f}°, Roll={data[1]/100:7.2f}°, Yaw={data[2]/100:7.2f}°")
    lines.append(f"陀螺仪: X={data[3]/100:7.2f}, Y={data[4]/100:7.2f}, Z={data[5]/100:7.2f} rad/s")
    lines.append(f"加速度: X={data[6]/100:7.2f}, Y={data[7]/100:7.2f}, Z={data[8]/100:7.2f} G")
    return "\n       ".join(lines)


def format_sensor_merged(data: List[int]) -> str:
    """格式化合并传感器数据（编码器4 + IMU9）"""
    lines = []

    # 编码器数据（前4个int16）
    enc_str = format_encoder(data[:4])
    lines.append(f"编码器: {enc_str}")

    # IMU数据（后9个int16）
    imu_str = format_imu(data[4:13])
    lines.append(f"IMU:    {imu_str}")

    return "\n       ".join(lines)


def format_ptp_sync(data: List[int]) -> str:
    """格式化PTP同步数据"""
    # 4个int16_t组成64位t2时间戳
    t2 = (data[3] << 48) | (data[2] << 32) | (data[1] << 16) | data[0]
    return f"t2={t2} us ({t2/1e6:.6f} s)"


def format_data(frame: ProtocolFrame) -> str:
    """根据功能码格式化数据"""
    if not frame.is_valid:
        return f"[错误] {frame.error_msg}"

    if frame.func_code == FuncCode.SENSOR_MERGED:
        return format_sensor_merged(frame.data)
    elif frame.func_code == FuncCode.PTP_SYNC:
        return format_ptp_sync(frame.data)
    else:
        return str(frame.data)


def format_timestamp(tx_timestamp: int, first_timestamp: int) -> str:
    """格式化时间戳"""
    if tx_timestamp == 0:
        return "N/A"
    tx_ms = tx_timestamp / 1000.0
    return f"TX:{tx_ms:10.3f}ms"


def print_frame(frame: ProtocolFrame, show_raw: bool = False, first_timestamp: int = 0):
    """打印帧信息"""
    data_str = format_data(frame)
    ts_str = format_timestamp(frame.tx_timestamp, first_timestamp)

    # 判断是否为多行输出
    if '\n' in data_str:
        print(f"[{ts_str}] {frame.func_name}:")
        for line in data_str.split('\n'):
            print(f"  {line}")
    else:
        print(f"[{ts_str}] {frame.func_name}: {data_str}")

    if show_raw:
        raw_hex = ' '.join(f'{b:02X}' for b in frame.raw)
        print(f"  原始: {raw_hex}")


# ==================== 统计模块 ====================

class FrameStats:
    """帧统计信息"""

    def __init__(self):
        self.counts = defaultdict(int)
        self.errors = defaultdict(int)
        self.last_timestamp = defaultdict(int)
        self.intervals = defaultdict(list)
        self.first_time = None

    def update(self, frame: ProtocolFrame):
        """更新统计"""
        self.counts[frame.func_code] += 1

        if not frame.is_valid:
            self.errors[frame.func_code] += 1

        # 计算帧间隔
        if frame.tx_timestamp > 0:
            last_ts = self.last_timestamp[frame.func_code]
            if last_ts > 0:
                interval = frame.tx_timestamp - last_ts
                if 0 < interval < 1000000:  # 合理间隔（<1秒）
                    self.intervals[frame.func_code].append(interval)
                    if len(self.intervals[frame.func_code]) > 100:
                        self.intervals[frame.func_code].pop(0)
            self.last_timestamp[frame.func_code] = frame.tx_timestamp

        if self.first_time is None:
            self.first_time = time.time()

    def get_frequency(self, func_code: int) -> float:
        """获取指定功能码的帧频率"""
        intervals = self.intervals.get(func_code, [])
        if len(intervals) < 2:
            return 0.0
        avg_interval = sum(intervals) / len(intervals)
        return 1000000.0 / avg_interval if avg_interval > 0 else 0.0

    def print_summary(self):
        """打印统计摘要"""
        duration = time.time() - self.first_time if self.first_time else 0

        print("\n" + "="*60)
        print("统计摘要")
        print("="*60)
        print(f"运行时长: {duration:.1f} 秒")
        print()

        total = sum(self.counts.values())
        print(f"总帧数: {total}")
        print()

        for func_code in sorted(self.counts.keys()):
            count = self.counts[func_code]
            errors = self.errors.get(func_code, 0)
            freq = self.get_frequency(func_code)
            name = FUNC_NAMES.get(func_code, f"未知(0x{func_code:02X})")

            print(f"[{name}]")
            print(f"  帧数: {count} ({count/duration*100:.1f} fps)" if duration > 0 else f"  帧数: {count}")
            if freq > 0:
                print(f"  频率: {freq:.1f} Hz")
            if errors > 0:
                print(f"  错误: {errors}")

        print("="*60)


# ==================== 串口处理 ====================

class SerialReader:
    """串口读取器"""

    def __init__(self, port: str, baudrate: int = 921600, debug: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.buffer = bytearray()
        self.stats = FrameStats()
        self.debug = debug
        self.raw_bytes_count = 0

    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
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

    def read_frames(self) -> List[ProtocolFrame]:
        """读取并解析所有完整帧"""
        if not self.serial or not self.serial.is_open:
            return []

        # 读取所有可用数据
        data = self.serial.read(self.serial.in_waiting)
        if not data:
            return []

        # 调试：显示接收到的原始数据
        if self.debug:
            self.raw_bytes_count += len(data)
            print(f"\n[调试] 接收 {len(data)} 字节 (总计: {self.raw_bytes_count})")
            print(f"  原始: {' '.join(f'{b:02X}' for b in data)}")

        self.buffer.extend(data)
        frames = []

        # 查找完整帧
        while True:
            # 查找帧头
            header_idx = self.buffer.find(PROTOCOL_HEADER)
            if header_idx == -1:
                self.buffer.clear()
                break

            # 丢弃帧头之前的数据
            if header_idx > 0:
                del self.buffer[:header_idx]

            # 检查是否有足够的数据来判断帧长度
            if len(self.buffer) < 5:
                break

            func_code = self.buffer[1]

            # 判断帧类型
            if func_code in MCU_DATA_LENGTHS:
                # MCU上报帧（带时间戳）
                expected_data_len = MCU_DATA_LENGTHS[func_code]
                expected_frame_len = 4 + expected_data_len + TX_TIMESTAMP_LEN
            else:
                # PC下发帧或其他帧（不带时间戳）
                # 需要查找帧尾来确定长度
                tail_idx = self.buffer.find(PROTOCOL_TAIL, 2)
                if tail_idx == -1:
                    break
                expected_frame_len = tail_idx + 1

            # 检查是否收到完整帧
            if len(self.buffer) < expected_frame_len:
                break

            # 提取一帧
            frame = bytes(self.buffer[:expected_frame_len])
            del self.buffer[:expected_frame_len]

            # 调试：显示找到的帧
            if self.debug:
                print(f"[调试] 找到帧 (长度: {len(frame)})")
                print(f"  内容: {' '.join(f'{b:02X}' for b in frame)}")

            # 解析帧
            frame_obj = parse_frame(frame)
            if frame_obj:
                if self.debug and not frame_obj.is_valid:
                    print(f"[调试] 解析失败: {frame_obj.error_msg}")
                frames.append(frame_obj)

        return frames

    def get_stats(self) -> FrameStats:
        """获取统计信息"""
        return self.stats


# ==================== 数据记录模块 ====================

class DataLogger:
    """数据记录器"""

    def __init__(self, log_file: Optional[str] = None):
        self.log_file = log_file
        self.file_handle = None
        self.csv_writer = None
        self.start_time = None

        if log_file:
            try:
                self.file_handle = open(log_file, 'w', newline='')
                self.csv_writer = csv.writer(self.file_handle)
                # 写入表头
                self.csv_writer.writerow([
                    'timestamp', 'func_code', 'func_name',
                    'data_str', 'tx_timestamp', 'is_valid'
                ])
                self.start_time = time.time()
                print(f"✓ 数据日志: {log_file}")
            except Exception as e:
                print(f"✗ 创建日志失败: {e}")
                self.file_handle = None

    def log(self, frame: ProtocolFrame):
        """记录一帧数据"""
        if not self.csv_writer:
            return

        elapsed = time.time() - self.start_time if self.start_time else 0
        data_str = ','.join(map(str, frame.data))

        self.csv_writer.writerow([
            f'{elapsed:.6f}',
            f'0x{frame.func_code:02X}',
            frame.func_name,
            data_str,
            frame.tx_timestamp,
            frame.is_valid
        ])

    def close(self):
        """关闭日志文件"""
        if self.file_handle:
            self.file_handle.close()
            print(f"✓ 日志已保存")


# ==================== 辅助函数 ====================

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

    parser = argparse.ArgumentParser(
        description='机器人底盘串口数据读取工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s -l                          # 列出可用串口
  %(prog)s -p /dev/ttyUSB0             # 读取所有数据
  %(prog)s -p /dev/ttyUSB0 -f sensor   # 只显示传感器数据
  %(prog)s -p /dev/ttyUSB0 -r          # 显示原始数据
  %(prog)s -p /dev/ttyUSB0 -b 921600   # 高波特率
  %(prog)s -p /dev/ttyUSB0 -s data.csv # 记录到文件

支持的数据类型:
  sensor   - 传感器数据（编码器+IMU）(0x20)
  ptp      - PTP同步 (0x10)
        """
    )

    parser.add_argument('-p', '--port', type=str, help='串口设备 (例如: /dev/ttyUSB0 或 COM3)')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率 (默认: 921600)')
    parser.add_argument('-l', '--list', action='store_true', help='列出可用串口')
    parser.add_argument('-r', '--raw', action='store_true', help='显示原始数据')
    parser.add_argument('-d', '--debug', action='store_true', help='调试模式（显示接收的原始字节）')
    parser.add_argument('-f', '--filter', type=str, nargs='+',
                       choices=['battery', 'encoder', 'imu', 'ptp'],
                       help='只显示指定类型的数据')
    parser.add_argument('-s', '--save', type=str, help='保存数据到CSV文件')
    parser.add_argument('--stats', action='store_true', help='显示统计信息')

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
            'sensor': FuncCode.SENSOR_MERGED,
            'ptp': FuncCode.PTP_SYNC,
        }
        filter_codes = set(filter_map[f] for f in args.filter)

    # 连接串口
    reader = SerialReader(port, args.baudrate, debug=args.debug)
    if not reader.connect():
        return 1

    # 创建数据记录器
    logger = DataLogger(args.save)

    print("\n开始读取数据 (按Ctrl+C退出)...\n")

    try:
        frame_count = 0
        error_count = 0
        first_timestamp = 0

        # 定期显示统计
        last_stats_time = time.time()

        while True:
            frames = reader.read_frames()

            for frame in frames:
                frame_count += 1
                reader.stats.update(frame)

                # 记录第一个时间戳
                if first_timestamp == 0 and frame.tx_timestamp > 0:
                    first_timestamp = frame.tx_timestamp

                # 统计错误
                if not frame.is_valid:
                    error_count += 1

                # 记录到文件
                logger.log(frame)

                # 应用过滤器
                if filter_codes and frame.func_code not in filter_codes:
                    continue

                # 打印帧信息
                print_frame(frame, show_raw=args.raw, first_timestamp=first_timestamp)

            # 定期显示统计信息
            if args.stats and time.time() - last_stats_time >= 5.0:
                reader.stats.print_summary()
                last_stats_time = time.time()

    except KeyboardInterrupt:
        print(f"\n\n总共接收 {frame_count} 帧数据")
        if error_count > 0:
            print(f"错误帧数: {error_count}")

        if args.stats:
            reader.stats.print_summary()

    finally:
        logger.close()
        reader.disconnect()

    return 0


if __name__ == '__main__':
    sys.exit(main())
