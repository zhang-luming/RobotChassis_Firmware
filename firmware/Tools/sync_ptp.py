#!/usr/bin/env python3
"""
PTP时间同步测试脚本

基于MCU简化后的PTP实现：
- 请求帧（PC→MCU）：[0xFC][0x10][0x01][checksum][0xDF] (5B)
- 响应帧（MCU→PC）：[0xFC][0x10][t2(8B)][t3(8B)][checksum][0xDF] (20B)

时间戳说明：
- t1: PC发送PTP请求的时刻
- t2: MCU接收PTP请求完成的时刻
- t3: MCU发送PTP响应开始的时刻
- t4: PC接收PTP响应完成的时刻

计算公式：
- offset = ((t2_unix - t1) - (t4 - t3_unix)) / 2
- delay = ((t4 - t1) + (t3_unix - t2_unix)) / 2
- Δt_MCU = t3 - t2 (MCU内部处理时间)
- Δt_Linux = t4 - t1 (整体往返时间)
"""

import serial
import serial.tools.list_ports
import time
import sys
import argparse
import statistics
from typing import Optional, Tuple


class PTPSync:
    """PTP时间同步类"""

    # 协议常量
    PROTOCOL_HEADER = 0xFC
    PROTOCOL_TAIL = 0xDF
    FUNC_CODE = 0x10
    MSG_REQUEST = 0x01

    # PTP帧大小
    REQUEST_FRAME_SIZE = 5
    RESPONSE_FRAME_SIZE = 20

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None

        # 时域映射参数
        self.g_offset: Optional[int] = None  # MCU时间 → Linux时间的偏移（微秒）
        self.filtered_offset: Optional[float] = None  # EMA滤波后的offset

        # 时间戳记录（用于异常检测）
        self.last_t1 = 0
        self.last_t2 = 0

        # 精确频率控制
        self.next_sync_time: Optional[int] = None  # 下次同步的时间点（微秒）

        # PI控制器参数
        self.kp = 0.5  # 比例系数
        self.ki = 0.01  # 积分系数
        self.integral = 0  # 积分累积

        # 滤波参数
        self.filter_alpha = 0.3  # EMA滤波系数
        self.convergence_threshold = 1000  # 收敛阈值（微秒）
        self.valid_threshold = 500  # 有效样本阈值（微秒）

        # 统计数据
        self.offset_history: list[float] = []  # 收敛后的offset历史
        self.sync_count = 0  # 同步次数
        self.consecutive_valid = 0  # 连续有效同步次数
        self.is_converged = False  # 是否已收敛

    def connect(self) -> bool:
        """连接串口"""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.5,
                write_timeout=0.5
            )
            print(f"[连接] 已连接到 {self.port} ({self.baudrate} baud)")
            return True
        except serial.SerialException as e:
            print(f"[错误] 连接失败: {e}")
            return False

    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[信息] 已断开连接")

    @staticmethod
    def checksum(data: bytes) -> int:
        """计算XOR校验和"""
        c = 0
        for b in data:
            c ^= b
        return c

    @staticmethod
    def now_us() -> int:
        """获取当前时间（微秒）"""
        return time.time_ns() // 1000

    def send_request_and_get_t1(self) -> Tuple[bool, int]:
        """
        发送PTP同步请求并记录t1（发送完成时刻）
        返回: (成功标志, t1时间戳)
        """
        frame = bytes([
            self.PROTOCOL_HEADER,
            self.FUNC_CODE,
            self.MSG_REQUEST
        ])
        # MCU端校验和包括帧头，从frame[0]开始
        cs = self.checksum(frame)
        full_frame = frame + bytes([cs, self.PROTOCOL_TAIL])

        try:
            self.ser.write(full_frame)
            self.ser.flush()
            # 发送完成后立即记录t1（最接近实际发送时刻）
            t1 = self.now_us()
            return True, t1
        except serial.SerialException as e:
            print(f"[错误] 发送失败: {e}")
            return False, 0

    def recv_response(self, timeout: float = 0.1) -> Optional[Tuple[int, int, int]]:
        """
        接收PTP响应帧
        返回: (t2, t3, t4) 或 None
              t4是接收完成时刻（检测到帧尾后立即记录）
        """
        start = time.time()
        buf = bytearray()
        t4 = None

        while time.time() - start < timeout:
            # 读取可用数据
            if self.ser.in_waiting:
                new_data = self.ser.read(self.ser.in_waiting)
                # 检查新数据中是否包含帧尾，如果包含则立即记录t4
                if self.PROTOCOL_TAIL in new_data:
                    t4 = self.now_us()
                buf.extend(new_data)

            # 尝试解析完整帧
            while len(buf) >= self.RESPONSE_FRAME_SIZE:
                # 查找帧头
                if buf[0] != self.PROTOCOL_HEADER:
                    buf.pop(0)
                    continue

                # 检查帧尾
                if buf[self.RESPONSE_FRAME_SIZE - 1] != self.PROTOCOL_TAIL:
                    buf.pop(0)
                    continue

                # 提取完整帧
                frame = bytes(buf[:self.RESPONSE_FRAME_SIZE])
                buf = buf[self.RESPONSE_FRAME_SIZE:]

                # 如果还没记录t4（应该已经记录了），使用当前时间
                if t4 is None:
                    t4 = self.now_us()

                # 校验验证（MCU端校验和包括帧头，从frame[0]开始到frame[17]）
                cs_recv = frame[18]
                cs_calc = self.checksum(frame[0:18])  # 包括帧头
                if cs_recv != cs_calc:
                    print(f"[警告] 校验失败: 计算值=0x{cs_calc:02X}, 接收值=0x{cs_recv:02X}")
                    t4 = None  # 重置t4
                    continue

                # 检查功能码
                if frame[1] != self.FUNC_CODE:
                    t4 = None  # 重置t4
                    continue

                # 解析t2（data[0-3]，4个int16_t小端序组成64位）
                t2_parts = []
                for i in range(4):
                    idx = 2 + i * 2
                    part = int.from_bytes(frame[idx:idx+2], byteorder='little', signed=False)
                    t2_parts.append(part)
                t2 = (t2_parts[3] << 48) | (t2_parts[2] << 32) | (t2_parts[1] << 16) | t2_parts[0]

                # 解析t3（tx_timestamp，8字节小端序）
                t3 = int.from_bytes(frame[10:18], byteorder='little', signed=False)

                return t2, t3, t4

        return None

    def sync_once(self) -> bool:
        """执行一次PTP同步"""
        # 发送请求并记录t1（发送完成时刻）
        success, t1 = self.send_request_and_get_t1()
        if not success:
            return False

        # 接收响应（返回t2, t3, t4，其中t4是接收完成时刻）
        resp = self.recv_response()

        if not resp:
            print(f"[警告] 同步#{self.sync_count+1} 超时无响应")
            # 超时不更新状态，避免积分累积误差
            return False

        t2_mcu, t3_mcu, t4 = resp

        # ===== 异常检测 =====
        if self.sync_count > 0:
            delta_t1 = t1 - self.last_t1
            delta_t2 = t2_mcu - self.last_t2

            # 检测时间间隔异常（可能是丢包导致的长时间间隔）
            # 预期间隔约500ms (500000us)，允许±50ms误差
            if abs(delta_t1 - 500000) > 50000 or abs(delta_t2 - 500000) > 50000:
                print(f"[警告] 异常时间间隔: Δt1={delta_t1/1000:.1f}ms, Δt2={delta_t2/1000:.1f}ms")
                # 重置PI控制器，防止累积误差
                self.integral = 0
                self.last_t1 = t1
                self.last_t2 = t2_mcu
                self.sync_count += 1
                return False

        # ===== 首次同步：建立时域映射 =====
        if self.g_offset is None:
            self.g_offset = t1 - t2_mcu
            self.integral = 0
            self.filtered_offset = 0.0

            # 转换为可读时间
            linux_time = time.gmtime(t1 / 1e6)
            time_str = time.strftime('%Y-%m-%d %H:%M:%S', linux_time)
            print(f"[初始化] 时域映射建立")
            print(f"         g_offset = {self.g_offset} us")
            print(f"         Linux时间: {time_str}.{t1 % 1000000:06d} UTC")
            print(f"         MCU运行时间: {t2_mcu/1e6:.3f} 秒")
            print()

        # ===== 将MCU时间转换到Linux时间域 =====
        t2_unix = t2_mcu + self.g_offset
        t3_unix = t3_mcu + self.g_offset

        # ===== 计算PTP offset和delay =====
        # offset: 时间偏移，正值=MCU快，负值=MCU慢
        offset = ((t2_unix - t1) - (t4 - t3_unix)) / 2.0
        # delay: 网络往返延迟
        delay = ((t4 - t1) + (t3_unix - t2_unix)) / 2.0

        # 检测异常offset
        if abs(offset) > 5000:
            print(f"[警告] 异常偏差值: {offset:+.1f} us，跳过本次")
            self.last_t1 = t1
            self.last_t2 = t2_mcu
            self.sync_count += 1
            return False

        # ===== PI控制器 =====
        p_term = offset * self.kp
        self.integral += offset
        # 限幅
        self.integral = max(min(self.integral, 10000), -10000)
        i_term = self.integral * self.ki

        correction = p_term + i_term
        # 限幅
        correction = max(min(correction, 500), -500)

        # 更新g_offset
        self.g_offset -= int(correction)

        # ===== EMA滤波 =====
        if self.filtered_offset is None:
            self.filtered_offset = offset
        else:
            self.filtered_offset = (self.filter_alpha * offset +
                                   (1 - self.filter_alpha) * self.filtered_offset)

        # ===== 计算时钟偏差 =====
        clock_ratio = None
        ppm_error = None
        if self.sync_count > 0:
            delta_t2_calc = t2_mcu - self.last_t2
            delta_t1_calc = t1 - self.last_t1
            if delta_t1_calc > 0:
                clock_ratio = delta_t2_calc / delta_t1_calc
                ppm_error = (clock_ratio - 1.0) * 1e6

        # ===== 计算时间间隔（用于日志） =====
        dt_mcu = t3_mcu - t2_mcu  # MCU处理时间 (t3 - t2)
        dt_linux = t4 - t1  # 整体往返时间 (t4 - t1)

        # ===== 统计 =====
        is_valid = abs(offset) < self.valid_threshold

        if is_valid:
            if not self.is_converged:
                self.is_converged = True
                print("[收敛] 已达到收敛状态，开始统计数据\n")

            self.offset_history.append(offset)
            if len(self.offset_history) > 1000:
                self.offset_history.pop(0)

        # 更新收敛状态
        if is_valid:
            self.consecutive_valid += 1
        elif abs(offset) > self.convergence_threshold:
            self.consecutive_valid = 0

        # 更新历史记录
        self.last_t1 = t1
        self.last_t2 = t2_mcu
        self.sync_count += 1

        # ===== 打印结果 =====
        status_icon = "✓" if is_valid else "!"
        print(f"#{self.sync_count:3d} {status_icon} "
              f"往返延迟={delay:+7.1f} us | "
              f"偏差={offset:+8.1f} us | "
              f"修正={correction:+8.1f} us")
        print(f"      Δt_MCU={dt_mcu:7d} us (t3-t2) | "
              f"Δt_Linux={dt_linux:7d} us (t4-t1)")

        if clock_ratio is not None:
            print(f"      时钟比={clock_ratio:+.6f} ({ppm_error:+.0f} ppm)")

        if self.consecutive_valid > 10:
            print(f"      时域偏移={self.g_offset} us | 连续收敛 {self.consecutive_valid} 次")
        else:
            print(f"      时域偏移={self.g_offset} us")

        print("-" * 80)

        return True

    def busy_wait_until(self, target_time_us: int):
        """
        忙等待直到达到目标时间点（微秒精度）
        使用单调时钟确保精度
        """
        while self.now_us() < target_time_us:
            # 短暂让出CPU（避免100%占用，但保持高精度）
            pass

    def run(self, interval: float = 0.5):
        """
        持续运行PTP同步（精确频率控制）
        使用单调时钟和忙等待确保精确的时间间隔
        """
        interval_us = int(interval * 1_000_000)  # 转换为微秒
        print(f"\n开始PTP时间同步（间隔={interval}s，精确频率控制）\n")
        print("=" * 80)

        # 设置首次同步时间（立即开始）
        self.next_sync_time = self.now_us()

        try:
            while True:
                # 执行同步
                self.sync_once()

                # 计算下次同步时间点
                self.next_sync_time += interval_us

                # 精确等待到下次同步时间点
                current_time = self.now_us()
                if current_time < self.next_sync_time:
                    self.busy_wait_until(self.next_sync_time)
                else:
                    # 如果同步耗时超过了间隔，立即开始下一次
                    # 并重新计算时间基准，避免累积误差
                    self.next_sync_time = self.now_us()

        except KeyboardInterrupt:
            self.print_statistics()

    def print_statistics(self):
        """打印统计报告"""
        print("\n")
        print("=" * 80)
        print("PTP时间同步统计报告")
        print("=" * 80)
        print(f"总同步次数: {self.sync_count}")
        print(f"连续收敛次数: {self.consecutive_valid}")
        print(f"最终时域偏移(g_offset): {self.g_offset} us")
        print()

        if len(self.offset_history) > 10:
            offsets = self.offset_history

            print("偏差(offset)统计:")
            print(f"  样本数: {len(offsets)} (仅收敛后的有效数据)")
            print(f"  最大值: {max(offsets):+.1f} us")
            print(f"  最小值: {min(offsets):+.1f} us")
            print(f"  平均值: {statistics.mean(offsets):+.1f} us")
            print(f"  中位数: {statistics.median(offsets):+.1f} us")
            if len(offsets) > 1:
                print(f"  标准差: {statistics.stdev(offsets):+.1f} us")

            # 收敛情况
            within_100 = sum(1 for o in offsets if abs(o) <= 100) / len(offsets) * 100
            within_500 = sum(1 for o in offsets if abs(o) <= 500) / len(offsets) * 100

            print()
            print("收敛情况:")
            print(f"  偏差 ≤ ±100 us: {within_100:.1f}%")
            print(f"  偏差 ≤ ±500 us: {within_500:.1f}%")

            # 评估同步质量
            if within_500 > 95:
                print()
                print("✓ 同步状态: 优秀 (95%以上偏差在±500us内)")
            elif within_500 > 80:
                print()
                print("△ 同步状态: 良好 (80%以上偏差在±500us内)")
            else:
                print()
                print("✗ 同步状态: 需要改进")
        else:
            print("[提示] 收敛数据不足，无法进行统计（需要至少10次有效同步）")

        print("=" * 80)


def list_serial_ports():
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用的串口设备")
        return []

    print("可用串口设备:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    return ports


def main():
    parser = argparse.ArgumentParser(
        description='PTP时间同步测试脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s -l                          # 列出可用串口
  %(prog)s /dev/ttyUSB0                # 使用默认参数同步（0.5s间隔）
  %(prog)s /dev/ttyUSB0 -i 0.2         # 200ms精确间隔同步
  %(prog)s /dev/ttyUSB0 -b 921600      # 使用高波特率

时间戳说明:
  t1: PC发送PTP请求的时刻
  t2: MCU接收PTP请求完成的时刻
  t3: MCU发送PTP响应开始的时刻
  t4: PC接收PTP响应完成的时刻

计算公式:
  offset = ((t2+g_offset - t1) - (t4 - (t3+g_offset))) / 2
  delay = ((t4 - t1) + ((t3+g_offset) - (t2+g_offset))) / 2
  Δt_MCU = t3 - t2 (MCU内部处理时间)
  Δt_Linux = t4 - t1 (整体往返时间)
        """
    )

    parser.add_argument('port', nargs='?', help='串口设备')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='波特率 (默认: 115200)')
    parser.add_argument('-i', '--interval', type=float, default=0.5,
                       help='同步间隔（秒，默认: 0.5，使用精确频率控制）')
    parser.add_argument('-l', '--list', action='store_true',
                       help='列出可用串口')

    args = parser.parse_args()

    # 列出串口
    if args.list or not args.port:
        list_serial_ports()
        if not args.port:
            print("\n请指定串口设备，例如: python3 sync_ptp.py /dev/ttyUSB0")
            return 1
        return 0

    # 创建PTP同步器
    ptp = PTPSync(args.port, args.baudrate)

    if not ptp.connect():
        return 1

    try:
        ptp.run(interval=args.interval)
    finally:
        ptp.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
