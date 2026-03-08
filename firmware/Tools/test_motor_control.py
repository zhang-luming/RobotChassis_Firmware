#!/usr/bin/env python3
"""
电机控制测试脚本

功能：
1. 向底盘发送电机速度控制指令
2. 实时监控编码器位置和计算速度
3. 验证电机响应是否符合预期

用法：
    python3 test_motor_control.py /dev/ttyUSB0 <speed_A> <speed_B> <speed_C> <speed_D>

示例：
    python3 test_motor_control.py /dev/ttyUSB0 1000 1000 1000 1000  # 前进
    python3 test_motor_control.py /dev/ttyUSB0 0 0 0 0           # 停止
    python3 test_motor_control.py /dev/ttyUSB0 -500 500 -500 500 # 原地旋转
"""

import sys
import time
import struct
from typing import Optional, List

try:
    import serial
except ImportError:
    print("错误: 未找到 pyserial 模块")
    print("请运行: pip install pyserial")
    sys.exit(1)

# ==================== 协议定义 ====================

PROTOCOL_HEADER = 0xFC
PROTOCOL_TAIL = 0xDF

FUNC_MOTOR_SPEED = 0x31  # 电机目标速度
FUNC_SENSOR_MERGED = 0x20  # 合并传感器数据（编码器4 + IMU9）

BAUD_RATE = 921600


def calc_checksum(data: bytes) -> int:
    """计算XOR校验和"""
    checksum = 0
    for b in data:
        checksum ^= b
    return checksum


def build_motor_speed_frame(speeds: List[int]) -> bytes:
    """
    构建电机速度控制帧

    帧格式：[FC][0x31][速度A低][速度A高][速度B低][速度B高]...[校验][DF]
    字节序：小端序

    Args:
        speeds: 4个电机的目标速度 (CPS - Counts Per Second)

    Returns:
        完整的协议帧字节流
    """
    if len(speeds) != 4:
        raise ValueError("需要4个电机速度值")

    frame = bytearray()
    frame.append(PROTOCOL_HEADER)
    frame.append(FUNC_MOTOR_SPEED)

    # 4个int16_t速度值（小端序）
    for speed in speeds:
        frame.extend(struct.pack('<h', int(speed)))  # 小端序int16

    # 校验和
    checksum = calc_checksum(frame)
    frame.append(checksum)
    frame.append(PROTOCOL_TAIL)

    return bytes(frame)


def parse_sensor_merged_frame(frame: bytes) -> Optional[dict]:
    """
    解析合并传感器数据帧（编码器 + IMU）

    帧格式：[FC][0x02][编码器4×int16][IMU 9×int16][时间戳8B][校验][DF]
           0    1     2-10字节        10-28字节

    Returns:
        包含编码器位置和IMU数据的字典 或 None
    """
    if len(frame) < 38:  # 最小帧长度：1+1+26+8+1+1=38
        return None

    if frame[0] != PROTOCOL_HEADER or frame[1] != FUNC_SENSOR_MERGED:
        return None

    # 数据部分：13个int16（编码器4 + IMU9）
    data = frame[2:28]
    values = struct.unpack('<13h', data)  # 13个小端序int16

    # 提取编码器数据（前4个int16）
    positions = [v & 0xFFFF for v in values[0:4]]

    # 提取IMU数据（后9个int16）
    imu_values = values[4:13]

    return {
        'encoder': positions,
        'imu': {
            'euler': {'pitch': imu_values[0] / 100.0, 'roll': imu_values[1] / 100.0, 'yaw': imu_values[2] / 100.0},
            'gyro': {'x': imu_values[3] / 100.0, 'y': imu_values[4] / 100.0, 'z': imu_values[5] / 100.0},
            'accel': {'x': imu_values[6] / 100.0, 'y': imu_values[7] / 100.0, 'z': imu_values[8] / 100.0}
        }
    }


class MotorTester:
    """电机控制测试器"""

    def __init__(self, port: str, baudrate: int = BAUD_RATE):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.buffer = bytearray()

        # 编码器历史（用于计算速度）
        self.last_positions = None
        self.last_time = None
        self.actual_speeds = [0.0, 0.0, 0.0, 0.0]
        self.first_sample = True  # 标记第一次采样
        self.print_count = 0      # 打印计数器（用于控制IMU显示频率）
        self.print_count = 0      # 打印计数器（用于控制IMU显示频率）

    def connect(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[OK] 已连接到 {self.port} ({self.baudrate} baud)")
            return True
        except serial.SerialException as e:
            print(f"[错误] 无法连接串口: {e}")
            return False

    def send_speed_command(self, speeds: List[int]):
        """发送速度控制指令"""
        frame = build_motor_speed_frame(speeds)
        self.serial.write(frame)
        self.serial.flush()

        speed_str = ', '.join(f'{s:>6}' for s in speeds)
        print(f"\n[TX] 发送速度指令: [{speed_str}] CPS")

    def read_frames(self) -> dict:
        """读取并解析所有完整帧"""
        if not self.serial or not self.serial.is_open:
            return {}

        # 读取所有可用数据
        data = self.serial.read(self.serial.in_waiting)
        if not data:
            return {}

        self.buffer.extend(data)
        frames = {}

        # 查找完整帧
        while True:
            # 查找帧头
            fc_idx = self.buffer.find(PROTOCOL_HEADER)
            if fc_idx == -1:
                self.buffer.clear()
                break

            # 删除帧头之前的数据
            if fc_idx > 0:
                del self.buffer[:fc_idx]

            # 检查是否有足够数据判断帧长度
            if len(self.buffer) < 5:
                break

            func_code = self.buffer[1]

            # 根据功能码确定数据长度
            if func_code == FUNC_SENSOR_MERGED:
                data_len = 26  # 13个int16 = 26字节（编码器4 + IMU9）
                total_len = 2 + data_len + 8 + 2  # FC+Func+Data+Timestamp+Checksum+DF = 38
            else:
                # 未知功能码，跳过
                del self.buffer[0:1]
                continue

            # 检查帧是否完整
            if len(self.buffer) < total_len:
                break

            # 提取完整帧
            frame = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]

            # 验证帧尾
            if frame[-1] != PROTOCOL_TAIL:
                continue

            # 解析帧
            if func_code == FUNC_SENSOR_MERGED:
                sensor_data = parse_sensor_merged_frame(frame)
                if sensor_data:
                    frames['encoder'] = sensor_data['encoder']
                    frames['imu'] = sensor_data['imu']

        return frames

    def calculate_speeds(self, positions: List[int], current_time: float) -> List[float]:
        """计算实际速度（CPS）"""
        # 第一次采样：只记录位置，不计算速度
        if self.first_sample:
            self.last_positions = positions
            self.last_time = current_time
            self.first_sample = False
            return [0.0, 0.0, 0.0, 0.0]

        # 正常计算
        if self.last_positions is None or self.last_time is None:
            return [0.0, 0.0, 0.0, 0.0]

        dt = current_time - self.last_time
        if dt < 0.001:  # 最小1ms间隔
            return self.actual_speeds

        speeds = []
        for i in range(4):
            # 处理编码器溢出（16位回环）
            delta = positions[i] - self.last_positions[i]
            if delta < -32767:  # 溢出：65535→0，delta约为-65535
                delta += 65536
            elif delta > 32767:  # 溢出：0→65535，delta约为+65535
                delta -= 65536

            speed_cps = delta / dt
            speeds.append(speed_cps)

        self.last_positions = positions
        self.last_time = current_time
        self.actual_speeds = speeds

        return speeds

    def print_status(self, target_speeds: List[int], positions: List[int],
                     actual_speeds: List[float], imu_data: Optional[dict] = None):
        """打印状态信息"""
        self.print_count += 1

        # 使用ANSI转义码清屏并移动到顶部
        if self.print_count > 1:
            # 向上移动4行到电机输出区域
            print("\033[4A", end="")

        # 打印电机状态
        for i in range(4):
            pos = positions[i]
            target = target_speeds[i]
            actual = actual_speeds[i]

            # 计算误差百分比
            error = actual - target
            error_pct = (error / target * 100) if target != 0 else 0

            print(f"[{i}] 位置:{pos:>6}  目标:{target:>6}  实际:{actual:>7.1f}  误差:{error_pct:>+7.1f}%")

        sys.stdout.flush()

    def run_test(self, target_speeds: List[int], duration: float = 5.0):
        """
        运行电机控制测试

        Args:
            target_speeds: 4个电机的目标速度 (CPS)
            duration: 测试持续时间（秒）
        """
        print("\n" + "="*80)
        print("电机控制测试")
        print("="*80)
        print(f"目标速度: {target_speeds} CPS")
        print(f"测试时长: {duration} 秒")
        print("按 Ctrl+C 提前停止")
        print("="*80)

        # 发送速度指令
        self.send_speed_command(target_speeds)

        # 监控循环
        start_time = time.time()
        last_print_time = 0

        try:
            while True:
                current_time = time.time()
                elapsed = current_time - start_time

                # 检查是否超时
                if elapsed >= duration:
                    break

                # 读取帧数据
                frames = self.read_frames()

                # 每100ms打印一次状态
                if current_time - last_print_time >= 0.1:
                    if 'encoder' in frames:
                        positions = frames['encoder']
                        actual_speeds = self.calculate_speeds(positions, current_time)
                        self.print_status(target_speeds, positions, actual_speeds,
                                        frames.get('imu'))
                        last_print_time = current_time

                time.sleep(0.01)  # 10ms轮询

        except KeyboardInterrupt:
            print("\n\n[中断] 用户停止测试")

        # 停止电机
        print("\n\n停止电机...")
        self.send_speed_command([0, 0, 0, 0])
        time.sleep(0.5)

        print("\n测试完成！")


def main():
    if len(sys.argv) != 6:
        print(__doc__)
        print("\n参数说明:")
        print("  speed_A/B/C/D: 电机目标速度 (CPS - Counts Per Second)")
        print("  正值=正转，负值=反转，0=停止")
        print("\n典型速度范围:")
        print("  慢速: 1000-3000 CPS")
        print("  中速: 3000-10000 CPS")
        print("  高速: 10000-30000 CPS")
        sys.exit(1)

    port = sys.argv[1]
    speeds = [int(arg) for arg in sys.argv[2:6]]

    # 创建测试器
    tester = MotorTester(port)

    # 连接串口
    if not tester.connect():
        sys.exit(1)

    # 等待用户确认
    input("\n按 Enter 开始测试...")

    # 运行测试
    try:
        tester.run_test(speeds, duration=10.0)
    finally:
        tester.serial.close()
        print("\n串口已关闭")


if __name__ == '__main__':
    main()
