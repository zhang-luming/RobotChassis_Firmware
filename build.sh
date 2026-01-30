#!/bin/bash
################################################################################
# RobotChassis_Firmware 构建脚本
# 跨平台构建脚本，支持 Linux/macOS/Windows(Git Bash/MSYS2)
################################################################################

set -e  # 遇到错误立即退出

# 颜色输出 (Windows CMD 可能不支持，但 Git Bash 支持)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目配置
PROJECT_NAME="RobotChassis_Firmware"
BUILD_ROOT="build"
TOOLCHAIN_PREFIX="arm-none-eabi-"

# 检测操作系统
detect_os() {
    case "$(uname -s)" in
        Linux*)     OS="Linux";;
        Darwin*)    OS="macOS";;
        MINGW*|MSYS*|CYGWIN*) OS="Windows";;
        *)          OS="Unknown";;
    esac
    export OS
}

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查工具链是否安装
check_toolchain() {
    print_info "检查 gcc-arm-none-eabi 工具链..."

    if ! command -v ${TOOLCHAIN_PREFIX}gcc &> /dev/null; then
        print_error "未找到 ${TOOLCHAIN_PREFIX}gcc"
        echo ""
        echo "请安装 gcc-arm-none-eabi 工具链:"
        if [ "$OS" = "Linux" ]; then
            echo "  Ubuntu/Debian:  sudo apt-get install gcc-arm-none-eabi"
            echo "  Arch Linux:     sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib"
        elif [ "$OS" = "macOS" ]; then
            echo "  使用 Homebrew:  brew install arm-none-eabi-gcc"
        elif [ "$OS" = "Windows" ]; then
            echo "  1. 从以下地址下载安装:"
            echo "     https://developer.arm.com/downloads/-/gnu-rm"
            echo "     选择 'gcc-arm-none-eabi' 最新版本"
            echo "  2. 或使用包管理器:"
            echo "     scoop install gcc-arm-none-eabi"
            echo "     choco install gcc-arm-none-eabi"
        fi
        echo ""
        exit 1
    fi

    # 显示版本信息
    local version=$(${TOOLCHAIN_PREFIX}gcc --version | head -n1)
    print_success "工具链已安装: $version"
}

# 检查构建工具
check_build_tools() {
    print_info "检查构建工具..."

    # 检查 CMake
    if ! command -v cmake &> /dev/null; then
        print_error "未找到 cmake"
        echo "请安装 cmake:"
        if [ "$OS" = "Linux" ]; then
            echo "  sudo apt-get install cmake"
        elif [ "$OS" = "macOS" ]; then
            echo "  brew install cmake"
        elif [ "$OS" = "Windows" ]; then
            echo "  下载: https://cmake.org/download/ 或"
            echo "  scoop install cmake"
            echo "  choco install cmake"
        fi
        exit 1
    fi

    # 检查 Ninja
    if ! command -v ninja &> /dev/null; then
        print_warning "未找到 ninja，将使用 Unix Makefiles"
        USE_NINJA=false
    else
        USE_NINJA=true
    fi

    local cmake_version=$(cmake --version | head -n1)
    print_success "构建工具: $cmake_version"
}

# 获取 CPU 核心数（用于并行编译）
get_cpu_cores() {
    if [ "$OS" = "Windows" ]; then
        # Windows 环境
        echo ${NUMBER_OF_PROCESSORS:-1}
    elif command -v nproc &> /dev/null; then
        # Linux
        nproc
    elif command -v sysctl &> /dev/null; then
        # macOS
        sysctl -n hw.ncpu
    else
        echo 1
    fi
}

# 获取文件大小（跨平台）
get_file_size() {
    local file="$1"
    if [ "$OS" = "Windows" ]; then
        # Windows (Git Bash/MSYS2)
        wc -c < "$file" 2>/dev/null || stat -c %s "$file" 2>/dev/null || stat -f %z "$file" 2>/dev/null
    else
        # Linux/macOS
        stat -c %s "$file" 2>/dev/null || stat -f %z "$file" 2>/dev/null
    fi
}

# 显示帮助信息
show_help() {
    cat << EOF
用法: $0 [选项] [构建类型]

选项:
    -h, --help          显示此帮助信息
    -c, --clean         清理构建目录（默认清理当前配置，使用 -a 清理所有）
    -a, --all           配合 -c 使用，清理所有构建配置
    -f, --flash         使用 OpenOCD 烧录固件
    -d, --debug         启用调试输出
    -r, --release       构建 Release 版本（默认 Debug）

构建类型:
    debug               构建调试版本 (默认, -O0 -g3)
    release             构建发布版本 (-Os -g0)

示例:
    $0                  # 构建调试版本 (输出到 build/Debug/)
    $0 release          # 构建发布版本 (输出到 build/Release/)
    $0 -c               # 清理当前配置的构建目录
    $0 -c -a            # 清理所有构建配置
    $0 -f               # 烧录固件到开发板
    $0 -c && $0         # 清理并重新构建

环境变量:
    BUILD_ROOT          构建根目录 (默认: build)
    OPENOCD_INTERFACE   OpenOCD 配置文件 (默认: interface/stlink.cfg)
    OPENOCD_TARGET      OpenOCD 目标配置 (默认: target/stm32f1x.cfg)

支持平台:
    Linux (Ubuntu, Arch, Fedora, etc.)
    macOS (Homebrew)
    Windows (Git Bash, MSYS2, WSL)

EOF
}

# 清理构建目录
clean_build() {
    local clean_all=$1

    if [ "$clean_all" = true ]; then
        print_info "清理所有构建目录: ${BUILD_ROOT}"
        if [ "$OS" = "Windows" ]; then
            if [ -d "$BUILD_ROOT" ]; then
                rm -rf "${BUILD_ROOT:?}"/*
                rmdir "${BUILD_ROOT}" 2>/dev/null || true
            fi
        else
            rm -rf ${BUILD_ROOT}
        fi
    else
        print_info "清理构建目录: ${BUILD_DIR}"
        if [ "$OS" = "Windows" ]; then
            if [ -d "$BUILD_DIR" ]; then
                rm -rf "${BUILD_DIR:?}"/*
                rmdir "${BUILD_DIR}" 2>/dev/null || true
            fi
        else
            rm -rf ${BUILD_DIR}
        fi
    fi

    print_success "清理完成"
}

# 配置项目
configure_project() {
    local build_type=$1

    print_info "配置项目 (构建类型: ${build_type})..."
    print_info "输出目录: ${BUILD_DIR}"

    # 创建构建目录
    mkdir -p ${BUILD_DIR}

    # 配置 CMake
    local generator="Ninja"
    if [ "$USE_NINJA" = false ]; then
        generator="Unix Makefiles"
    fi

    # Windows 下可能需要指定路径格式
    local toolchain_file="cmake/gcc-arm-none-eabi.cmake"
    if [ "$OS" = "Windows" ]; then
        # 转换为 Unix 风格路径（Git Bash 兼容）
        toolchain_file="$(pwd)/cmake/gcc-arm-none-eabi.cmake"
    fi

    cmake -S . -B ${BUILD_DIR} \
        -DCMAKE_BUILD_TYPE=${build_type} \
        -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}" \
        -G "${generator}" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

    print_success "配置完成"
}

# 构建项目
build_project() {
    print_info "开始编译..."

    local cores=$(get_cpu_cores)

    if [ "$USE_NINJA" = true ]; then
        ninja -C ${BUILD_DIR}
    else
        make -C ${BUILD_DIR} -j${cores}
    fi

    print_success "编译完成"
}

# 显示构建信息
show_build_info() {
    local build_type=$1
    print_info "固件信息 ($build_type):"
    echo ""

    # ELF 文件大小
    local elf_file="${BUILD_DIR}/${PROJECT_NAME}.elf"
    if [ -f "$elf_file" ]; then
        ${TOOLCHAIN_PREFIX}size $elf_file
        echo ""
    fi

    # 输出目录
    local output_dir="${BUILD_DIR}/output"

    # HEX 文件大小
    local hex_file="${output_dir}/${PROJECT_NAME}.hex"
    if [ -f "$hex_file" ]; then
        local hex_size=$(get_file_size "$hex_file")
        print_info "HEX: ${hex_file} ($(($hex_size / 1024))KB)"
    fi

    # BIN 文件大小
    local bin_file="${output_dir}/${PROJECT_NAME}.bin"
    if [ -f "$bin_file" ]; then
        local bin_size=$(get_file_size "$bin_file")
        print_info "BIN: ${bin_file} ($(($bin_size / 1024))KB)"
    fi
}

# 烧录固件
flash_firmware() {
    print_info "烧录固件到开发板..."

    # 检查 OpenOCD
    if ! command -v openocd &> /dev/null; then
        print_error "未找到 openocd"
        echo "请安装 openocd:"
        if [ "$OS" = "Linux" ]; then
            echo "  sudo apt-get install openocd"
        elif [ "$OS" = "macOS" ]; then
            echo "  brew install openocd"
        elif [ "$OS" = "Windows" ]; then
            echo "  scoop install openocd"
            echo "  choco install openocd"
        fi
        exit 1
    fi

    # 检查 ELF 文件
    local elf_file="${BUILD_DIR}/${PROJECT_NAME}.elf"
    if [ ! -f "$elf_file" ]; then
        print_error "未找到固件文件: $elf_file"
        print_info "请先构建项目: $0"
        exit 1
    fi

    local interface=${OPENOCD_INTERFACE:-interface/stlink.cfg}
    local target=${OPENOCD_TARGET:-target/stm32f1x.cfg}

    print_info "使用 OpenOCD 配置: $interface, $target"

    openocd -f $interface -f $target \
        -c "program $elf_file verify reset exit"

    print_success "烧录完成"
}

# 主函数
main() {
    detect_os

    local build_type="Debug"
    local do_clean=false
    local clean_all=false
    local do_flash=false
    local extra_args=()

    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -c|--clean)
                do_clean=true
                shift
                ;;
            -a|--all)
                clean_all=true
                shift
                ;;
            -f|--flash)
                do_flash=true
                shift
                ;;
            -d|--debug)
                set -x
                shift
                ;;
            -r|--release)
                build_type="Release"
                shift
                ;;
            debug)
                build_type="Debug"
                shift
                ;;
            release)
                build_type="Release"
                shift
                ;;
            *)
                print_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # 根据构建类型设置构建目录
    BUILD_DIR="${BUILD_ROOT}/${build_type}"

    # 显示标题
    echo ""
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║     RobotChassis_Firmware 构建脚本                         ║"
    echo "║     STM32F103xE 四驱机器人底盘固件                          ║"
    echo "║     平台: $OS                            ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""

    # 检查工具链和构建工具
    check_toolchain
    check_build_tools
    echo ""

    # 清理构建目录
    if [ "$do_clean" = true ]; then
        clean_build $clean_all
        if [ "$do_flash" = false ] && [ ${#extra_args[@]} -eq 0 ]; then
            exit 0
        fi
    fi

    # 配置项目
    configure_project $build_type

    # 构建项目
    build_project

    # 显示构建信息
    echo ""
    show_build_info $build_type

    # 烧录固件
    if [ "$do_flash" = true ]; then
        echo ""
        flash_firmware
    fi

    echo ""
    print_success "构建完成！"
    echo ""
    print_info "输出文件位于:"
    print_info "  - ${BUILD_DIR}/${PROJECT_NAME}.elf"
    print_info "  - ${BUILD_DIR}/output/${PROJECT_NAME}.hex"
    print_info "  - ${BUILD_DIR}/output/${PROJECT_NAME}.bin"
    print_info "  - ${BUILD_DIR}/${PROJECT_NAME}.map"
    echo ""
}

# 运行主函数
main "$@"
