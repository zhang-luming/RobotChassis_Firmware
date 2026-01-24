#!/usr/bin/env python3
"""
机器人底盘控制指令发送脚本

功能：
- 发送电机速度控制指令
- 发送PID参数设置指令
- 发送舵机控制指令

协议格式：[0xFC][FuncCode][Data...][XORChecksum][0xDF]

协议说明：
- 帧头: 0xFC
- 功能码: 见下方 FUNC_* 定义
- 数据: 各功能码对应不同格式
- 校验: XOR校验（帧头+功能码+数据）
- 帧尾: 0xDF
"""

import sys
import time
import struct
from typing import Optional

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
FUNC_MOTOR_SPEED = 0x06      # 电机目标速度
FUNC_PID_PARAM = 0x07        # PID参数设置
FUNC_SERVO_CONTROL = 0x08    # 舵机控制


# ==================== 协议函数 ====================

def calculate_xor_checksum(data: bytes) -> int:
    """计算XOR校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_frame(func_code: int, data: bytes = b'') -> bytes:
    """
    构建协议帧

    参数：
        func_code: 功能码
        data: 数据内容（bytes）

    返回：
        完整的协议帧（bytes）
    """
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


# ==================== 控制器类 ====================

class RobotController:
    """机器人控制器（仅发送）"""

    def __init__(self, port: str, baudrate: int = 115200):
        """
        初始化控制器

        Args:
            port: 串口设备路径 (例: /dev/ttyUSB0 或 COM3)
            baudrate: 波特率 (默认: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None

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

    def send_frame(self, frame: bytes) -> bool:
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

        参数：
            motor_a/b/c/d: 电机速度 (encoder ticks/40ms)
                           正值=正转，负值=反转，0=停止

        示例：
            controller.set_motor_speed(100, 100, 100, 100)  # 4个电机正向
            controller.set_motor_speed(-100, -100, -100, -100)  # 4个电机反向
            controller.set_motor_speed(0, 0, 0, 0)  # 停止
        """
        # 4个int16_t，大端序
        data = struct.pack('>hhhh', motor_a, motor_b, motor_c, motor_d)
        frame = build_frame(FUNC_MOTOR_SPEED, data)
        self.send_frame(frame)
        print(f"→ 发送电机速度: A={motor_a:5d}, B={motor_b:5d}, C={motor_c:5d}, D={motor_d:5d}")

    def set_pid(self, kp: float, ki: float, kd: float):
        """
        设置PID参数

        参数：
            kp/ki/kd: PID参数（实际值，会自动×100）
                     所有电机使用相同的PID参数

        示例：
            controller.set_pid(1.5, 0.2, 0.8)
        """
        # 转换为协议格式（×100）
        kp_scaled = int(kp * 100)
        ki_scaled = int(ki * 100)
        kd_scaled = int(kd * 100)

        # 3个int16_t，大端序
        data = struct.pack('>hhh', kp_scaled, ki_scaled, kd_scaled)
        frame = build_frame(FUNC_PID_PARAM, data)
        self.send_frame(frame)
        print(f"→ 发送PID参数: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")

    def set_servo(self, servo1: int, servo2: int):
        """
        设置舵机角度

        参数：
            servo1/servo2: 舵机角度值 (0-180度)

        示例：
            controller.set_servo(90, 45)  # 舵机1到90度，舵机2到45度
        """
        # 限制范围
        servo1 = max(0, min(180, servo1))
        servo2 = max(0, min(180, servo2))

        # 2个uint8_t（单字节）
        data = struct.pack('BB', servo1, servo2)
        frame = build_frame(FUNC_SERVO_CONTROL, data)
        self.send_frame(frame)
        print(f"→ 发送舵机角度: 舵机1={servo1:3d}°, 舵机2={servo2:3d}°")

    def stop_all_motors(self):
        """停止所有电机"""
        self.set_motor_speed(0, 0, 0, 0)


# ==================== 示例程序 ====================

def example_basic_control(port: str, baudrate: int):
    """基础控制示例"""
    controller = RobotController(port, baudrate)

    if not controller.connect():
        return 1

    print("\n开始控制示例...\n")

    try:
        # 1. 设置PID参数
        print("【步骤1】设置PID参数")
        controller.set_pid(1.5, 0.2, 0.8)
        time.sleep(0.2)

        # 2. 设置舵机
        print("\n【步骤2】设置舵机角度")
        controller.set_servo(90, 90)
        time.sleep(0.2)

        # 3. 电机前进
        print("\n【步骤3】电机前进 (速度=100)")
        controller.set_motor_speed(100, 100, 100, 100)
        time.sleep(2)

        # 4. 电机后退
        print("\n【步骤4】电机后退 (速度=-100)")
        controller.set_motor_speed(-100, -100, -100, -100)
        time.sleep(2)

        # 5. 停止
        print("\n【步骤5】停止电机")
        controller.stop_all_motors()

        print("\n✓ 控制示例完成")

    except KeyboardInterrupt:
        print("\n✗ 用户中断")
        controller.stop_all_motors()

    finally:
        controller.disconnect()

    return 0


def example_interactive(port: str, baudrate: int):
    """交互式控制"""
    controller = RobotController(port, baudrate)

    if not controller.connect():
        return 1

    print("\n" + "=" * 50)
    print("         机器人底盘交互式控制")
    print("=" * 50)
    print("运动控制:")
    print("  w/s/a/d - 前进/后退/左转/右转")
    print("  space   - 停止电机")
    print("  + / -   - 加速/减速")
    print("\n参数设置:")
    print("  p <kp> <ki> <kd> - 设置PID参数")
    print("  s <s1> <s2>      - 设置舵机角度")
    print("\n其他:")
    print("  q       - 退出程序")
    print("  help    - 显示帮助")
    print("=" * 50)

    try:
        speed = 100  # 默认速度

        while True:
            try:
                cmd = input("\n> ").strip().lower()
            except EOFError:
                break

            if cmd == 'q':
                break

            elif cmd == ' ' or cmd == '':
                controller.stop_all_motors()

            elif cmd == 'w':
                print(f"前进 (速度={speed})")
                controller.set_motor_speed(speed, speed, speed, speed)

            elif cmd == 's' and not cmd.startswith('s '):
                print(f"后退 (速度={speed})")
                controller.set_motor_speed(-speed, -speed, -speed, -speed)

            elif cmd == 'a':
                print(f"左转 (速度={speed})")
                controller.set_motor_speed(-speed, speed, -speed, speed)

            elif cmd == 'd':
                print(f"右转 (速度={speed})")
                controller.set_motor_speed(speed, -speed, speed, -speed)

            elif cmd == '+':
                speed = min(500, speed + 50)
                print(f"速度增加到: {speed}")

            elif cmd == '-':
                speed = max(50, speed - 50)
                print(f"速度减少到: {speed}")

            elif cmd.startswith('p '):
                parts = cmd.split()
                if len(parts) == 4:
                    try:
                        kp, ki, kd = map(float, parts[1:])
                        controller.set_pid(kp, ki, kd)
                    except ValueError:
                        print("✗ 参数格式错误，应为浮点数")
                else:
                    print("用法: p <kp> <ki> <kd>")
                    print("示例: p 1.5 0.2 0.8")

            elif cmd.startswith('s '):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        servo1, servo2 = map(int, parts[1:])
                        controller.set_servo(servo1, servo2)
                    except ValueError:
                        print("✗ 参数格式错误，应为整数")
                else:
                    print("用法: s <servo1> <servo2>")
                    print("示例: s 90 45")

            elif cmd == 'help':
                print("\n" + "=" * 50)
                print("运动控制:")
                print("  w/s/a/d - 前进/后退/左转/右转")
                print("  space   - 停止电机")
                print("  + / -   - 加速/减速")
                print("\n参数设置:")
                print("  p <kp> <ki> <kd> - 设置PID参数")
                print("  s <s1> <s2>      - 设置舵机角度")
                print("\n其他:")
                print("  q       - 退出程序")
                print("=" * 50)

            else:
                print("✗ 未知命令，输入 'help' 查看帮助")

    except KeyboardInterrupt:
        print("\n✗ 用户中断")

    finally:
        controller.stop_all_motors()
        controller.disconnect()

    return 0


# ==================== 主程序 ====================

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='机器人底盘控制指令发送器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 交互式控制模式
  %(prog)s -p /dev/ttyUSB0 -i

  # 运行基础控制示例
  %(prog)s -p /dev/ttyUSB0 --example

  # Windows系统
  %(prog)s -p COM3 -i

  # 指定波特率
  %(prog)s -p /dev/ttyUSB0 -b 9600 -i

  # 列出可用串口
  %(prog)s --list
        """
    )
    parser.add_argument('-p', '--port', type=str,
                        help='串口设备 (例: /dev/ttyUSB0 或 COM3)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='交互式控制模式')
    parser.add_argument('--example', action='store_true',
                        help='运行基础控制示例')
    parser.add_argument('--list', action='store_true',
                        help='列出可用串口设备')

    args = parser.parse_args()

    # 列出串口
    if args.list:
        print("可用串口设备:")
        ports = serial.tools.list_ports.comports()
        if ports:
            for port in ports:
                print(f"  {port.device}: {port.description}")
        else:
            print("  未找到可用串口")
        return 0

    # 检查串口参数
    if not args.port:
        # 尝试自动检测串口
        ports = serial.tools.list_ports.comports()
        if len(ports) == 1:
            port = ports[0].device
            print(f"自动选择串口: {port}")
        elif len(ports) > 1:
            print("找到多个串口，请使用 -p 参数指定:")
            for p in ports:
                print(f"  {p.device}: {p.description}")
            return 1
        else:
            print("未找到可用串口，请使用 -p 参数指定")
            print("提示: 使用 --list 查看可用串口")
            return 1
    else:
        port = args.port

    baudrate = args.baudrate

    # 运行对应模式
    if args.interactive:
        sys.exit(example_interactive(port, baudrate))
    elif args.example:
        sys.exit(example_basic_control(port, baudrate))
    else:
        parser.print_help()
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
