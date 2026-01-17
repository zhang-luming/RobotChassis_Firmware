#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID 命令生成器
用于生成 RobotChassis 电机 PID 控制参数的串口调试命令
"""

import sys


def calculate_checksum(data):
    """计算 XOR 校验和"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def generate_pid_command(kp, ki, kd):
    """
    生成 PID 设置命令

    参数:
        kp: 比例系数 (int16)
        ki: 积分系数 (int16)
        kd: 微分系数 (int16)

    返回:
        bytes: 完整的命令帧
    """
    # 命令帧结构
    # [0xFC][0x07][P_H][P_L][I_H][I_L][D_H][D_L][0x00][0x00][CHECKSUM][0xDF]
    cmd = bytearray([
        0xFC,                              # 帧头
        0x07,                              # 命令类型 (PID设置)
    ])

    # 添加 Kp (int16, 大端序)
    cmd.append((kp >> 8) & 0xFF)  # 高字节
    cmd.append(kp & 0xFF)         # 低字节

    # 添加 Ki (int16, 大端序)
    cmd.append((ki >> 8) & 0xFF)
    cmd.append(ki & 0xFF)

    # 添加 Kd (int16, 大端序)
    cmd.append((kd >> 8) & 0xFF)
    cmd.append(kd & 0xFF)

    # 保留字节
    cmd.append(0x00)
    cmd.append(0x00)

    # 计算校验和 (前10字节的 XOR)
    checksum = calculate_checksum(cmd[:10])
    cmd.append(checksum)

    # 帧尾
    cmd.append(0xDF)

    return bytes(cmd)


def format_output(cmd):
    """格式化输出命令"""
    # 十六进制字符串 (空格分隔)
    hex_str = ' '.join(f'{b:02X}' for b in cmd)

    # 十六进制字符串 (紧凑)
    hex_compact = ''.join(f'{b:02X}' for b in cmd)

    # C 数组格式
    c_array = 'uint8_t pid_cmd[] = {' + ', '.join(f'0x{b:02X}' for b in cmd) + '};'

    return hex_str, hex_compact, c_array


def print_command(kp, ki, kd, cmd):
    """打印命令信息"""
    print("=" * 70)
    print(f" PID 参数设置命令 ".center(70, "="))
    print("=" * 70)
    print(f"参数: Kp = {kp}, Ki = {ki}, Kd = {kd}")
    print("-" * 70)

    hex_str, hex_compact, c_array = format_output(cmd)

    print("\n【方式一】串口助手使用 (复制以下内容):")
    print(f"  {hex_str}")

    print("\n【方式二】紧凑十六进制:")
    print(f"  {hex_compact}")

    print("\n【方式三】C 代码数组:")
    print(f"  {c_array}")

    print("\n【方式四】二进制数据 (用于 Python serial 等):")
    print(f"  bytes([{', '.join(hex(b) for b in cmd)}])")

    print("\n【命令长度】: {} 字节".format(len(cmd)))
    print("=" * 70)


def interactive_mode():
    """交互式模式"""
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║                    PID 命令生成器 - 交互模式                        ║
╠══════════════════════════════════════════════════════════════════════╣
║  输入 PID 参数，自动生成串口调试命令                                ║
║  协议: [0xFC][0x07][P_H][P_L][I_H][I_L][D_H][D_L][0x00][0x00][CS][0xDF] ║
╚══════════════════════════════════════════════════════════════════════╝
    """)

    while True:
        try:
            print("\n" + "-" * 70)
            kp = int(input("请输入 Kp (比例系数): ").strip() or "0")
            ki = int(input("请输入 Ki (积分系数): ").strip() or "0")
            kd = int(input("请输入 Kd (微分系数): ").strip() or "0")

            # 生成命令
            cmd = generate_pid_command(kp, ki, kd)

            # 打印结果
            print_command(kp, ki, kd, cmd)

            # 询问是否继续
            cont = input("\n是否继续? (y/n): ").strip().lower()
            if cont not in ['y', 'yes', '是', 'Y']:
                print("\n退出程序。")
                break

        except ValueError:
            print("\n❌ 输入错误! 请输入有效的整数。")
        except KeyboardInterrupt:
            print("\n\n退出程序。")
            break
        except Exception as e:
            print(f"\n❌ 发生错误: {e}")


def batch_mode(params_list):
    """批量模式"""
    print("=" * 70)
    print(f" PID 批量命令生成 ".center(70, "="))
    print("=" * 70)

    for kp, ki, kd in params_list:
        cmd = generate_pid_command(kp, ki, kd)
        hex_str = ' '.join(f'{b:02X}' for b in cmd)
        print(f"Kp={kp:3d}, Ki={ki:3d}, Kd={kd:3d}  →  {hex_str}")

    print("=" * 70)


def main():
    """主函数"""
    if len(sys.argv) > 1:
        # 命令行参数模式
        if sys.argv[1] in ['-h', '--help']:
            print("""
使用方法:
  1. 交互模式:
     python pid_command_generator.py

  2. 命令行模式:
     python pid_command_generator.py <Kp> <Ki> <Kd>
     示例: python pid_command_generator.py 55 10 10

  3. 批量模式:
     python pid_command_generator.py --batch
     (将在脚本内编辑 params_list)
            """)
            return

        if sys.argv[1] == '--batch':
            # 批量模式，在此编辑需要生成的参数列表
            params_list = [
                (35, 0, 20),
                (45, 0, 15),
                (55, 0, 10),
                (55, 5, 10),
                (55, 10, 10),
            ]
            batch_mode(params_list)
            return

        # 解析命令行参数
        try:
            kp = int(sys.argv[1])
            ki = int(sys.argv[2]) if len(sys.argv) > 2 else 0
            kd = int(sys.argv[3]) if len(sys.argv) > 3 else 0

            cmd = generate_pid_command(kp, ki, kd)
            print_command(kp, ki, kd, cmd)

            # 仅输出十六进制字符串（便于管道处理）
            print('\n' + ' '.join(f'{b:02X}' for b in cmd))

        except ValueError:
            print("❌ 参数错误! 请输入有效的整数。")
            print("用法: python pid_command_generator.py <Kp> <Ki> <Kd>")
    else:
        # 交互式模式
        interactive_mode()


if __name__ == "__main__":
    main()
