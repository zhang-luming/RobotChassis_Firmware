# ============================================================================
# RobotChassis_Firmware Windows PowerShell 构建脚本
# 用于在 Windows 下使用 gcc-arm-none-eabi 工具链编译 STM32F103 项目
# ============================================================================

param(
    [switch]$Clean,
    [switch]$Flash,
    [switch]$Release,
    [switch]$Help
)

# 项目配置
$PROJECT_NAME = "RobotChassis_Firmware"
$BUILD_DIR = "build"
$TOOLCHAIN_PREFIX = "arm-none-eabi-"
$script:USE_NINJA = $false

# 颜色输出函数
function Print-Info {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor Cyan
}

function Print-Success {
    param([string]$Message)
    Write-Host "[SUCCESS] $Message" -ForegroundColor Green
}

function Print-Warning {
    param([string]$Message)
    Write-Host "[WARNING] $Message" -ForegroundColor Yellow
}

function Print-Error {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

# 显示帮助信息
function Show-Help {
    Write-Host @"

用法: .\build.ps1 [选项]

选项:
    -Clean              清理构建目录
    -Flash              使用 OpenOCD 烧录固件
    -Release            构建 Release 版本（默认 Debug）
    -Help               显示此帮助信息

示例:
    .\build.ps1                  # 构建调试版本
    .\build.ps1 -Release         # 构建发布版本
    .\build.ps1 -Clean           # 清理构建目录
    .\build.ps1 -Flash           # 烧录固件到开发板
    .\build.ps1 -Clean; .\build.ps1 -Release  # 清理并重新构建

环境变量:
    BUILD_DIR           自定义构建目录 (默认: build)
    OPENOCD_INTERFACE   OpenOCD 配置文件 (默认: interface/stlink.cfg)
    OPENOCD_TARGET      OpenOCD 目标配置 (默认: target/stm32f1x.cfg)

支持平台:
    Windows (PowerShell 5.1+, PowerShell Core 6+, 7+)

"@
    exit 0
}

# 检查工具链
function Check-Toolchain {
    Print-Info "检查 gcc-arm-none-eabi 工具链..."

    $gccCmd = "$($TOOLCHAIN_PREFIX)gcc"

    try {
        # 使用 Get-Command 检查命令是否存在
        $cmdInfo = Get-Command $gccCmd -ErrorAction SilentlyContinue
        if ($cmdInfo) {
            # 执行命令获取版本
            $output = & $gccCmd --version 2>&1
            $gccVersion = $output | Select-Object -First 1
            Print-Success "工具链已安装: $gccVersion"
            return $true
        }
    }
    catch {
        # 忽略错误，继续检查
    }

    Print-Error "未找到 $($TOOLCHAIN_PREFIX)gcc"
    Write-Host ""
    Write-Host "请安装 gcc-arm-none-eabi 工具链:" -ForegroundColor Yellow
    Write-Host "  方法1: 从 ARM 官网下载安装"
    Write-Host "    https://developer.arm.com/downloads/-/gnu-rm"
    Write-Host "  方法2: 使用包管理器"
    Write-Host "    scoop install gcc-arm-none-eabi"
    Write-Host "    choco install gcc-arm-none-eabi"
    Write-Host ""
    return $false
}

# 检查构建工具
function Check-BuildTools {
    Print-Info "检查构建工具..."

    # 检查 CMake
    try {
        $cmakeVersion = cmake --version 2>&1 | Select-Object -First 1
        if ($LASTEXITCODE -eq 0) {
            Print-Success "构建工具: $cmakeVersion"
        }
        else {
            Print-Error "未找到 cmake"
            Write-Host "请安装: https://cmake.org/download/ 或使用包管理器安装"
            return $false
        }
    }
    catch {
        Print-Error "未找到 cmake"
        Write-Host "请安装: https://cmake.org/download/ 或使用包管理器安装"
        return $false
    }

    # 检查 Ninja
    $script:USE_NINJA = $false
    $ninjaCmd = Get-Command ninja -ErrorAction SilentlyContinue
    if ($ninjaCmd) {
        $script:USE_NINJA = $true
        Write-Host "  找到 ninja，将使用 Ninja 生成器" -ForegroundColor Cyan
    }

    return $true
}

# 清理构建目录
function Clean-BuildDir {
    Print-Info "清理构建目录: $BUILD_DIR"

    if (Test-Path $BUILD_DIR) {
        Remove-Item -Path $BUILD_DIR -Recurse -Force
    }

    Print-Success "清理完成"
}

# 配置项目
function Configure-Project {
    param([string]$BuildType)

    Print-Info "配置项目 (构建类型: $BuildType)..."

    # 创建构建目录
    if (-not (Test-Path $BUILD_DIR)) {
        New-Item -ItemType Directory -Path $BUILD_DIR | Out-Null
    }

    # 配置 CMake
    $toolchainFile = Join-Path $PWD "cmake\gcc-arm-none-eabi.cmake"

    $generator = "Ninja"
    if (-not $script:USE_NINJA) {
        $generator = "Unix Makefiles"
    }

    $cmakeArgs = @(
        "-S",
        ".",
        "-B",
        $BUILD_DIR,
        "-DCMAKE_BUILD_TYPE=$BuildType",
        "-DCMAKE_TOOLCHAIN_FILE=$toolchainFile",
        "-G",
        $generator,
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    )

    & cmake $cmakeArgs

    if ($LASTEXITCODE -ne 0) {
        Print-Error "配置失败"
        return $false
    }

    Print-Success "配置完成"
    return $true
}

# 构建项目
function Build-Project {
    Print-Info "开始编译..."

    if ($script:USE_NINJA) {
        & ninja -C $BUILD_DIR
    }
    else {
        & cmake --build $BUILD_DIR -- "-j$env:NUMBER_OF_PROCESSORS"
    }

    if ($LASTEXITCODE -ne 0) {
        Print-Error "编译失败"
        return $false
    }

    Print-Success "编译完成"
    return $true
}

# 显示构建信息
function Show-BuildInfo {
    Print-Info "固件信息:"
    Write-Host ""

    $elfFile = Join-Path $BUILD_DIR "$PROJECT_NAME.elf"
    if (Test-Path $elfFile) {
        & "$($TOOLCHAIN_PREFIX)size" $elfFile
        Write-Host ""
    }

    $outputDir = Join-Path $BUILD_DIR "output"

    $hexFile = Join-Path $outputDir "$PROJECT_NAME.hex"
    if (Test-Path $hexFile) {
        $hexSize = (Get-Item $hexFile).Length
        $hexSizeKB = [math]::Floor($hexSize / 1KB)
        Print-Info "HEX 文件: output\$PROJECT_NAME.hex ($hexSizeKB KB)"
    }

    $binFile = Join-Path $outputDir "$PROJECT_NAME.bin"
    if (Test-Path $binFile) {
        $binSize = (Get-Item $binFile).Length
        $binSizeKB = [math]::Floor($binSize / 1KB)
        Print-Info "BIN 文件: output\$PROJECT_NAME.bin ($binSizeKB KB)"
    }
}

# 烧录固件
function Flash-Firmware {
    Print-Info "烧录固件到开发板..."

    # 检查 OpenOCD
    try {
        $null = openocd --version 2>&1
    }
    catch {
        Print-Error "未找到 openocd"
        Write-Host "请安装: scoop install openocd 或 choco install openocd"
        return $false
    }

    if ($LASTEXITCODE -ne 0) {
        Print-Error "未找到 openocd"
        Write-Host "请安装: scoop install openocd 或 choco install openocd"
        return $false
    }

    $elfFile = Join-Path $BUILD_DIR "$PROJECT_NAME.elf"
    if (-not (Test-Path $elfFile)) {
        Print-Error "未找到固件文件: $elfFile"
        Print-Info "请先构建项目: .\build.ps1"
        return $false
    }

    $interface = if ($env:OPENOCD_INTERFACE) { $env:OPENOCD_INTERFACE } else { "interface/stlink.cfg" }
    $target = if ($env:OPENOCD_TARGET) { $env:OPENOCD_TARGET } else { "target/stm32f1x.cfg" }

    Print-Info "使用 OpenOCD 配置: $interface, $target"

    & openocd -f $interface -f $target -c "program $elfFile verify reset exit"

    if ($LASTEXITCODE -eq 0) {
        Print-Success "烧录完成"
        return $true
    }
    else {
        Print-Error "烧录失败"
        return $false
    }
}

# 主函数
function Main {
    # 显示标题
    Write-Host ""
    Write-Host "╔══════════════════════════════════════════════════════════╗"
    Write-Host "║     RobotChassis_Firmware 构建脚本                         ║"
    Write-Host "║     STM32F103xE 四驱机器人底盘固件                          ║"
    Write-Host "║     平台: Windows (PowerShell)                             ║"
    Write-Host "╚══════════════════════════════════════════════════════════╝"
    Write-Host ""

    # 处理帮助
    if ($Help) {
        Show-Help
    }

    # 确定构建类型
    $buildType = if ($Release) { "Release" } else { "Debug" }

    # 检查工具链
    if (-not (Check-Toolchain)) {
        exit 1
    }

    # 检查构建工具
    if (-not (Check-BuildTools)) {
        exit 1
    }

    Write-Host ""

    # 清理构建目录
    if ($Clean) {
        Clean-BuildDir
        if (-not $Flash) {
            exit 0
        }
    }

    # 配置项目
    if (-not (Configure-Project -BuildType $buildType)) {
        exit 1
    }

    # 构建项目
    if (-not (Build-Project)) {
        exit 1
    }

    # 显示构建信息
    Write-Host ""
    Show-BuildInfo

    # 烧录固件
    if ($Flash) {
        Write-Host ""
        Flash-Firmware
    }

    Write-Host ""
    Print-Success "构建完成！"
    Write-Host ""
    Print-Info "输出文件位于:"
    Print-Info "  - $BUILD_DIR\$PROJECT_NAME.elf  (可执行文件，用于调试)"
    Print-Info "  - $BUILD_DIR\output\$PROJECT_NAME.hex  (Intel HEX 格式，用于烧录)"
    Print-Info "  - $BUILD_DIR\output\$PROJECT_NAME.bin  (二进制文件)"
    Print-Info "  - $BUILD_DIR\$PROJECT_NAME.map  (内存映射文件)"
    Write-Host ""
}

# 运行主函数
Main
