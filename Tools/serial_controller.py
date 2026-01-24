#!/usr/bin/env python3
"""
机器人底盘控制指令发送脚本

功能：
- 发送电机速度控制指令
- 发送PID参数设置指令
- 发送舵机控制指令

协议格式：[0xFC][FuncCode][Data...][Checksum][0xDF]
"""

import serial
import serial.tools.list_ports
import struct
import sys
import time
from typing import List


# ==================== 协议定义 ====================

PROTOCOL_HEADER = 0xFC
PROTOCOL_TAIL = 0xDF

# 功能码定义
FUNC_MOTOR_SPEED = 0x06
FUNC_PID_PARAM = 0x07
FUNC_SERVO_CONTROL = 0x08


# ==================== 协议函数 ====================

def calculate_xor_checksum(data: bytes) -> int:
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_frame(func_code: int, data: List[int]) -> bytes:
    """
    构建协议帧

    参数：
        func_code: 功能码
        data: int16_t数据列表

    返回：
        完整的协议帧（bytes）
    """
    frame = bytearray()
    frame.append(PROTOCOL_HEADER)
    frame.append(func_code)

    # 添加数据（int16_t大端序）
    for value in data:
        frame.extend(struct.pack('>h', value))

    # 计算并添加校验和（包括帧头、功能码和数据段）
    checksum = calculate_xor_checksum(bytes(frame))
    frame.append(checksum)

    # 添加帧尾
    frame.append(PROTOCOL_TAIL)

    return bytes(frame)


# ==================== 控制指令 ====================

class RobotController:
    """机器人控制器"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: serial.Serial = None

    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
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

    def send_frame(self, frame: bytes):
        """发送数据帧"""
        if not self.serial or not self.serial.is_open:
            print("✗ 串口未连接")
            return False

        try:
            self.serial.write(frame)
            return True
        except serial.SerialException as e:
            print(f"✗ 发送失败: {e}")
            return False

    def set_motor_speed(self, motor_a: int = 0, motor_b: int = 0,
                        motor_c: int = 0, motor_d: int = 0):
        """
        设置电机目标速度

        参数单位：centi-CPS (CPS/100)
        例如：1000 表示 100000 CPS = 100000 编码器计数/秒

        示例：
            controller.set_motor_speed(100, 100, 100, 100)  # 4个电机正向
            controller.set_motor_speed(-100, -100, -100, -100)  # 4个电机反向
            controller.set_motor_speed(0, 0, 0, 0)  # 停止
        """
        frame = build_frame(FUNC_MOTOR_SPEED, [motor_a, motor_b, motor_c, motor_d])
        self.send_frame(frame)
        print(f"→ 发送电机速度: A={motor_a}, B={motor_b}, C={motor_c}, D={motor_d}")

    def set_pid(self, kp: float, ki: float, kd: float):
        """
        设置PID参数

        参数：实际的PID值（会自动×100）
        所有电机使用相同的PID参数

        示例：
            controller.set_pid(1.5, 0.2, 0.8)
        """
        # 转换为协议格式（×100）
        kp_scaled = int(kp * 100)
        ki_scaled = int(ki * 100)
        kd_scaled = int(kd * 100)

        frame = build_frame(FUNC_PID_PARAM, [kp_scaled, ki_scaled, kd_scaled])
        self.send_frame(frame)
        print(f"→ 发送PID参数: Kp={kp}, Ki={ki}, Kd={kd}")

    def set_servo(self, servo1: int, servo2: int):
        """
        设置舵机角度

        参数：舵机角度值（0-180）

        示例：
            controller.set_servo(90, 45)  # 舵机1到90度，舵机2到45度
        """
        # 限制范围
        servo1 = max(0, min(180, servo1))
        servo2 = max(0, min(180, servo2))

        frame = build_frame(FUNC_SERVO_CONTROL, [servo1, servo2])
        self.send_frame(frame)
        print(f"→ 发送舵机角度: 舵机1={servo1}°, 舵机2={servo2}°")

    def stop_all_motors(self):
        """停止所有电机"""
        self.set_motor_speed(0, 0, 0, 0)


# ==================== 示例程序 ====================

def example_basic_control():
    """基础控制示例"""
    controller = RobotController('/dev/ttyUSB0')

    if not controller.connect():
        return 1

    print("\n开始控制示例...\n")

    try:
        # 设置PID参数
        controller.set_pid(1.5, 0.2, 0.8)
        time.sleep(0.1)

        # 设置舵机
        controller.set_servo(90, 90)
        time.sleep(0.1)

        # 电机前进
        print("\n电机前进...")
        controller.set_motor_speed(100, 100, 100, 100)
        time.sleep(2)

        # 电机后退
        print("\n电机后退...")
        controller.set_motor_speed(-100, -100, -100, -100)
        time.sleep(2)

        # 停止
        print("\n停止电机...")
        controller.stop_all_motors()

    except KeyboardInterrupt:
        print("\n用户中断")
        controller.stop_all_motors()

    finally:
        controller.disconnect()

    return 0


def example_interactive():
    """交互式控制"""
    controller = RobotController('/dev/ttyUSB0')

    if not controller.connect():
        return 1

    print("\n交互式控制模式")
    print("命令:")
    print("  w/s/a/d - 前进/后退/左转/右转")
    print("  space - 停止")
    print("  q - 退出")
    print("  p <kp> <ki> <kd> - 设置PID参数")
    print("  s <servo1> <servo2> - 设置舵机")

    try:
        speed = 100

        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == 'q':
                break

            elif cmd == ' ':
                controller.stop_all_motors()

            elif cmd == 'w':
                print("前进")
                controller.set_motor_speed(speed, speed, speed, speed)

            elif cmd == 's':
                print("后退")
                controller.set_motor_speed(-speed, -speed, -speed, -speed)

            elif cmd == 'a':
                print("左转")
                controller.set_motor_speed(-speed, speed, -speed, speed)

            elif cmd == 'd':
                print("右转")
                controller.set_motor_speed(speed, -speed, speed, -speed)

            elif cmd.startswith('p '):
                parts = cmd.split()
                if len(parts) == 4:
                    kp, ki, kd = map(float, parts[1:])
                    controller.set_pid(kp, ki, kd)
                else:
                    print("用法: p <kp> <ki> <kd>")

            elif cmd.startswith('s '):
                parts = cmd.split()
                if len(parts) == 3:
                    servo1, servo2 = map(int, parts[1:])
                    controller.set_servo(servo1, servo2)
                else:
                    print("用法: s <servo1> <servo2>")

            else:
                print("未知命令")

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        controller.stop_all_motors()
        controller.disconnect()

    return 0


# ==================== 主程序 ====================

def main():
    import argparse

    parser = argparse.ArgumentParser(description='机器人底盘控制指令发送器')
    parser.add_argument('-p', '--port', type=str, help='串口设备 (默认: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='波特率 (默认: 115200)')
    parser.add_argument('-i', '--interactive', action='store_true', help='交互式控制模式')
    parser.add_argument('--example', action='store_true', help='运行基础控制示例')

    args = parser.parse_args()

    port = args.port or '/dev/ttyUSB0'

    if args.interactive:
        # 交互式模式
        sys.exit(example_interactive())
    elif args.example:
        # 示例模式
        sys.exit(example_basic_control())
    else:
        print("请指定 --interactive 或 --example")
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
