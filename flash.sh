#!/bin/bash
################################################################################
# RobotChassis Firmware Flash Script
#
# Auto-detect firmware files in build directory and flash selected one
################################################################################

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Find firmware files
find_firmware() {
    local found=()

    # Search for .elf, .hex and .bin files
    for dir in build build/Debug build/Release build/Debug/output build/Release/output; do
        [ -d "$dir" ] || continue

        for f in "$dir"/*.elf "$dir"/*.hex "$dir"/*.bin; do
            [ -f "$f" ] && found+=("$f")
        done
    done

    # Sort by modification time
    IFS=$'\n' sorted=($(ls -t "${found[@]}" 2>/dev/null))

    echo "${sorted[@]}"
}

show_menu() {
    local firmwares=("$@")
    local count=${#firmwares[@]}

    echo -e "${CYAN}============================================================${NC}"
    echo -e "${CYAN}         Select Firmware to Flash                             ${NC}"
    echo -e "${CYAN}============================================================${NC}"
    echo ""

    if [ $count -eq 0 ]; then
        echo -e "${YELLOW}No firmware files found${NC}"
        echo ""
        echo "Please run: ./build.sh"
        exit 1
    fi

    echo "Found $count firmware file(s):"
    echo ""

    for i in "${!firmwares[@]}"; do
        local fw="${firmwares[$i]}"
        local filename=$(basename "$fw")
        local dir=$(dirname "$fw")
        local size=$(arm-none-eabi-size "$fw" 2>/dev/null | tail -1 | awk '{print "FLASH:" $1 " RAM:" $2}')

        # Extract build type
        local build_type="Debug"
        [[ "$dir" =~ "Release" ]] && build_type="Release"

        printf "  [%d] ${GREEN}%s${NC} (%s)\n" $((i+1)) "$filename" "$build_type"
        echo "      Path: $fw"
        echo "      Size: $size"
        echo ""
    done

    echo "  [0] Exit"
    echo ""

    read -p "Select firmware [1-$count, default:1]: " choice
    choice=${choice:-1}

    if [ "$choice" -eq 0 ]; then
        echo "Cancelled"
        exit 0
    fi

    if [ "$choice" -lt 1 ] || [ "$choice" -gt $count ]; then
        echo -e "${YELLOW}Invalid choice, using default 1${NC}"
        choice=1
    fi

    selected="${firmwares[$((choice-1))]}"
    echo ""
    echo "Selected: $(basename "$selected")"
}

echo -e "${CYAN}============================================================${NC}"
echo -e "${CYAN}          RobotChassis Firmware Flasher                     ${NC}"
echo -e "${CYAN}============================================================${NC}"
echo ""

# Find and show menu
firmwares=($(find_firmware))
show_menu "${firmwares[@]}"

# Flash
echo ""
echo -e "${CYAN}Flashing...${NC}"
echo "MCU: STM32F103xE"
echo "Debugger: ST-Link (SWD)"
echo ""

# Check file extension and set OpenOCD command accordingly
# .bin files need explicit address (0x08000000 for STM32F103)
# .elf and .hex files contain address information
if [[ "$selected" == *.bin ]]; then
    echo "Note: BIN file requires explicit flash address 0x08000000"
    sudo openocd \
        -f interface/stlink.cfg \
        -f target/stm32f1x.cfg \
        -c "program \"$selected\" 0x08000000 verify reset exit"
else
    sudo openocd \
        -f interface/stlink.cfg \
        -f target/stm32f1x.cfg \
        -c "program \"$selected\" verify reset exit"
fi

echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}                  Flash Complete!                           ${GREEN}"
echo -e "${GREEN}============================================================${NC}"
