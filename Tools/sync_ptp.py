#!/usr/bin/env python3
"""
PTP-like Time Sync Debug Script
- 打印 t1 / t2 / t3 / t4
- 仅用于观测原始时间戳，不做控制
"""

import serial
import serial.tools.list_ports
import time
import sys
from typing import Optional, Tuple


class PTPSync:
    PROTOCOL_HEADER = 0xFC
    PROTOCOL_TAIL = 0xDF
    FUNC_CODE = 0x10
    MSG_REQ = 0x01
    MSG_RESP = 0x02

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.seq = 0

        # 时域映射参数
        self.g_offset = None  # MCU_time → Linux_time 的转换偏移
        self.filtered_offset = None  # 滤波后的PTP offset
        self.last_t2 = 0
        self.last_t1 = 0

        # 滤波参数
        self.filter_alpha = 0.3  # EMA滤波系数（提高到0.3加快收敛）
        self.convergence_threshold = 1000  # 偏差超过此值视为失去收敛（us）

        # PI控制器参数（用于快速收敛）
        self.kp = 0.5  # 比例系数
        self.ki = 0.01  # 积分系数
        self.integral = 0  # 积分累积

        # offset统计（用于收敛分析）
        self.offset_history = []  # 保存最近的offset值（仅收敛后）
        self.consecutive_valid = 0  # 连续有效的同步次数
        self.sync_count = 0  # 同步计数器
        self.is_converged = False  # 是否已经达到收敛状态

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
        print(f"[INFO] Connected to {self.port}")

    def close(self):
        if self.ser:
            self.ser.close()

    @staticmethod
    def checksum(data: bytes) -> int:
        c = 0
        for b in data:
            c ^= b
        return c

    @staticmethod
    def now_us() -> int:
        """获取微秒级时间戳（使用纳秒级精度）"""
        return time.time_ns() // 1000

    def send_request(self):
        frame = bytes([
            self.PROTOCOL_HEADER,
            self.FUNC_CODE,
            self.MSG_REQ,
            self.seq & 0xFF
        ])
        cs = self.checksum(frame)
        self.ser.write(frame + bytes([cs, self.PROTOCOL_TAIL]))

    def recv_response(self, timeout=1.0) -> Optional[Tuple[int, int, int]]:
        start = time.time()
        buf = bytearray()

        # 帧格式：[FC][Func][Data(10 bytes)][TxTimestamp(8 bytes)][Checksum][DF]
        # 总共20字节（不含帧头帧尾），所有数据小端序
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                buf.extend(self.ser.read(self.ser.in_waiting))

                while len(buf) >= 22:
                    if buf[0] != self.PROTOCOL_HEADER:
                        buf.pop(0)
                        continue

                    if self.PROTOCOL_TAIL not in buf:
                        break

                    tail = buf.index(self.PROTOCOL_TAIL)
                    frame = bytes(buf[1:tail])
                    buf = buf[tail + 1:]

                    if len(frame) < 20:
                        continue

                    cs_recv = frame[-1]
                    cs_calc = self.checksum(bytes([self.PROTOCOL_HEADER]) + frame[:-1])
                    if cs_recv != cs_calc:
                        continue

                    if frame[0] != self.FUNC_CODE:
                        continue

                    # 解析 data0 (int16_t小端序)
                    data0 = frame[1] | (frame[2] << 8)
                    msg = data0 & 0xFF
                    if msg != self.MSG_RESP:
                        continue

                    def parse_u64_from_i16_le(idx):
                        """从4个int16_t（小端序）解析64位值"""
                        part0 = frame[idx] | (frame[idx + 1] << 8)      # [15:0]
                        part1 = frame[idx + 2] | (frame[idx + 3] << 8)  # [31:16]
                        part2 = frame[idx + 4] | (frame[idx + 5] << 8)  # [47:32]
                        part3 = frame[idx + 6] | (frame[idx + 7] << 8)  # [63:48]
                        return (part3 << 48) | (part2 << 32) | (part1 << 16) | part0

                    def parse_u64_le(idx):
                        """解析64位小端序值"""
                        result = 0
                        for i in range(8):
                            result |= frame[idx + i] << (i * 8)
                        return result

                    # frame[3-10]: t2 (8字节，4个int16_t小端序)
                    # frame[11-18]: tx_timestamp (t3, 8字节，小端序)
                    t2 = parse_u64_from_i16_le(3)
                    t3 = parse_u64_le(11)
                    return data0 >> 8, t2, t3

        return None

    def sync_once(self):
        self.seq += 1

        # 发送请求并记录t1（发送完成时刻）
        self.send_request()
        t1 = self.now_us()

        # 接收响应并记录t4（接收完成时刻）
        resp = self.recv_response()
        t4 = self.now_us()

        if not resp:
            print("[警告] 超时无响应")
            # Timeout时不要更新任何状态，直接返回
            return

        _, t2_mcu, t3_mcu = resp

        # 检测时间戳异常（MCU时间戳突然大幅减小，可能是回滚）
        if self.sync_count > 0 and t2_mcu < self.last_t2 and (self.last_t2 - t2_mcu) > 100000:
            print(f"[警告] MCU时间戳回滚: {self.last_t2} -> {t2_mcu}")
            self.last_t2 = t2_mcu
            return

        # 检测异常的时间跳跃（比如timeout导致的）
        if self.sync_count > 0:
            delta_t2 = t2_mcu - self.last_t2
            delta_t1 = t1 - self.last_t1
            # 如果时间间隔异常（>1秒），说明中间有timeout或丢包
            if delta_t1 > 1000000 or delta_t2 > 1000000:
                print(f"[警告] 异常时间间隔: Δt1={delta_t1/1000:.1f}ms, Δt2={delta_t2/1000:.1f}ms，跳过本次")
                # 跳过本次，但更新last_t1和last_t2以便下次正常计算
                self.last_t1 = t1
                self.last_t2 = t2_mcu
                # 重置积分项，防止累积误差
                self.integral = 0
                # 注意：不重置consecutive_valid，因为超时不代表失去收敛
                self.sync_count += 1
                return

        # ===== 首次同步：建立时域映射 =====
        if self.g_offset is None:
            # 初始化时域偏移（将MCU时间映射到Linux时间域）
            self.g_offset = t1 - t2_mcu
            self.filtered_offset = 0
            self.integral = 0

            # 将时间戳转换为可读格式
            linux_time_struct = time.gmtime(t1 / 1e6)
            print(f"[INIT] 时域映射建立")
            print(f"       g_offset = {self.g_offset} us")
            print(f"       Linux时间: {time.strftime('%Y-%m-%d %H:%M:%S', linux_time_struct)}.{t1 % 1000000:06d} UTC")
            print(f"       MCU运行时间: {t2_mcu/1e6:.3f} 秒")
            print()

        # ===== 将MCU时间转换到Linux时间域 =====
        t2_unix = t2_mcu + self.g_offset
        t3_unix = t3_mcu + self.g_offset

        # ===== 计算标准PTP offset和delay =====
        # offset_ptp: 正值=MCU快，负值=MCU慢（反映网络延迟不对称性）
        # delay: 网络往返延迟
        offset_ptp = ((t2_unix - t1) - (t4 - t3_unix)) / 2
        delay = ((t4 - t1) + (t3_unix - t2_unix)) / 2

        # 检测异常的offset值（可能由timeout导致的）
        if abs(offset_ptp) > 5000:
            print(f"[警告] 异常偏差值: {offset_ptp:.1f} us，跳过本次")
            # 更新last_t1和last_t2
            self.last_t1 = t1
            self.last_t2 = t2_mcu
            return

        # ===== 使用PI控制器让offset快速收敛 =====
        # P项：比例控制（直接响应当前误差）
        p_term = offset_ptp * self.kp

        # I项：积分控制（消除累积误差）
        self.integral += offset_ptp
        # 限制积分项，防止饱和
        self.integral = max(min(self.integral, 10000), -10000)
        i_term = self.integral * self.ki

        # 总控制输出
        correction = p_term + i_term

        # 限制单次调整幅度（防止过度补偿）
        correction = max(min(correction, 500), -500)

        # 更新 g_offset（如果offset为正，说明MCU快，需要减小g_offset）
        self.g_offset -= correction

        # ===== 同时使用EMA滤波显示offset趋势 =====
        if self.filtered_offset is None:
            self.filtered_offset = offset_ptp
        else:
            self.filtered_offset = (self.filter_alpha * offset_ptp +
                                   (1 - self.filter_alpha) * self.filtered_offset)

        # ===== 计算时钟速度比 =====
        clock_ratio = None
        ppm_error = None
        delta_t2_out = 0
        delta_t1_out = 0
        if self.sync_count > 0:
            delta_t2_calc = t2_mcu - self.last_t2
            delta_t1_calc = t1 - self.last_t1
            if delta_t1_calc > 0:
                clock_ratio = delta_t2_calc / delta_t1_calc
                ppm_error = (clock_ratio - 1.0) * 1e6
                delta_t2_out = delta_t2_calc
                delta_t1_out = delta_t1_calc

        # ===== 记录offset用于统计分析 =====
        # 只在收敛后才记录offset，避免初始大偏差影响统计
        is_valid = abs(offset_ptp) < 500

        if is_valid:
            if not self.is_converged:
                # 首次达到收敛
                self.is_converged = True
                print(f"[信息] 已达到收敛状态，开始统计数据")

            self.offset_history.append(offset_ptp)
            if len(self.offset_history) > 1000:  # 只保留最近1000次
                self.offset_history.pop(0)

        # ===== 判断是否收敛 =====
        # 连续收敛计数只受实际偏差影响，不受超时影响
        if is_valid:
            self.consecutive_valid += 1
        else:
            # 只有偏差超过阈值才重置连续收敛计数
            # 这样可以容忍偶尔的异常值，保持收敛状态的连续性
            if abs(offset_ptp) > self.convergence_threshold:
                self.consecutive_valid = 0

        # ===== 更新历史记录和计数 =====
        self.last_t1 = t1
        self.last_t2 = t2_mcu
        self.sync_count += 1

        # ===== 打印结果（使用中文）=====
        status_icon = "✓" if abs(offset_ptp) < 500 else "!"
        print(f"#{self.sync_count:3d} {status_icon} 往返延迟={delay:+7.1f} us | "
              f"偏差={offset_ptp:+8.1f} us | 修正值={correction:+8.1f} us")
        if clock_ratio is not None:
            print(f"      Δt_MCU={delta_t2_out:7d} us | Δt_Linux={delta_t1_out:7d} us | "
                  f"时钟比={clock_ratio:+.6f} ({ppm_error:+.0f} ppm)")
        if self.consecutive_valid > 10:
            print(f"      时域偏移={self.g_offset} us | 已连续收敛 {self.consecutive_valid} 次")
        else:
            print(f"      时域偏移={self.g_offset} us")
        print("-" * 80)

    def run(self, interval=0.2):
        """持续运行PTP同步，直到用户按Ctrl+C停止"""
        try:
            while True:
                self.sync_once()
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n")
            print("=" * 80)
            print("PTP时间同步统计报告")
            print("=" * 80)
            print(f"总同步次数: {self.sync_count}")
            print(f"连续收敛次数: {self.consecutive_valid}")
            print(f"最终时域偏移(g_offset): {self.g_offset} us")
            print()

            # 计算offset统计
            if len(self.offset_history) > 10:
                offsets = self.offset_history
                import statistics

                print("偏差(offset)统计:")
                print(f"  样本数: {len(offsets)} (仅统计收敛后的数据)")
                print(f"  最大值: {max(offsets):+.1f} us")
                print(f"  最小值: {min(offsets):+.1f} us")
                print(f"  平均值: {statistics.mean(offsets):+.1f} us")
                print(f"  中位数: {statistics.median(offsets):+.1f} us")
                if len(offsets) > 1:
                    print(f"  标准差: {statistics.stdev(offsets):+.1f} us")

                # 计算在±100us、±500us内的比例
                within_100 = sum(1 for o in offsets if abs(o) <= 100) / len(offsets) * 100
                within_500 = sum(1 for o in offsets if abs(o) <= 500) / len(offsets) * 100
                print()
                print("收敛情况:")
                print(f"  偏差 ≤ ±100 us: {within_100:.1f}%")
                print(f"  偏差 ≤ ±500 us: {within_500:.1f}%")

                # 判断是否达到良好同步
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


def list_ports():
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}: {p.description}")


def main():
    print("Available ports:")
    list_ports()

    if len(sys.argv) < 2:
        print("Usage: python3 sync_ptp.py /dev/ttyUSB0")
        return

    ptp = PTPSync(sys.argv[1])
    ptp.connect()

    try:
        ptp.run()
    finally:
        ptp.close()


if __name__ == "__main__":
    main()
