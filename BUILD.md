# 快速构建指南

## 构建脚本选择

根据您的环境选择合适的构建脚本：

| 环境 | 使用脚本 | 命令示例 |
|------|---------|---------|
| **Windows PowerShell** | `build.ps1` | `.\build.ps1` |
| **Windows Git Bash** | `build.sh` | `./build.sh` |
| **Linux/macOS** | `build.sh` | `./build.sh` |

---

## Windows 使用方法（推荐 PowerShell）

```powershell
# 构建调试版本
.\build.ps1

# 构建发布版本（优化）
.\build.ps1 -Release

# 清理构建目录
.\build.ps1 -Clean

# 构建并烧录到开发板
.\build.ps1 -Flash

# 查看帮助
.\build.ps1 -Help
```

---

## Linux/macOS 使用方法

```bash
# 构建调试版本
./build.sh

# 构建发布版本（优化）
./build.sh release

# 清理构建目录
./build.sh -c

# 构建并烧录到开发板
./build.sh -f

# 查看帮助
./build.sh -h
```

---

## 首次使用前

### Windows 安装工具链

1. **安装 gcc-arm-none-eabi**
   ```powershell
   scoop install gcc-arm-none-eabi
   # 或
   choco install gcc-arm-none-eabi
   ```

2. **安装 CMake**
   ```powershell
   scoop install cmake
   ```

3. **安装 Ninja**（可选，加速构建）
   ```powershell
   scoop install ninja
   ```

### Linux 安装工具链

```bash
# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi cmake ninja-build

# Arch Linux
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib cmake ninja
```

---

## 输出文件

构建成功后，固件位于：
```
build/
├── RobotChassis_Firmware.elf         # 用于 GDB 调试
└── output/
    ├── RobotChassis_Firmware.hex     # 烧录用（常用）
    └── RobotChassis_Firmware.bin     # 二进制格式
```

---

## 详细文档

完整构建说明请参考：[Documentation/构建说明.md](Documentation/构建说明.md)
