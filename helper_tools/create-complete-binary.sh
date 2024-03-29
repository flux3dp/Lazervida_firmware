#!/bin/zsh

# Flash memory layout
# Bootloader occupies 35KB (0 - 35KB)
# Marker occupies 1KB      (35KB - 36KB)
# Main firmware occurpies < 96KB (36KB - N)

# Pad Bootloader (0-35KB)
declare -i BootloaderSize=$(ls -al | grep "Lazervida_Bootloader.bin" | awk '{print $5}')
declare -i BOOTLOADER_PADDING_SIZE=35840-$BootloaderSize # Padding to the end of 35KB
# NOTE: 0o377 = 0xFF
dd if=/dev/zero ibs=1 count=$BOOTLOADER_PADDING_SIZE | LC_ALL=C tr "\000" "\377" >> Lazervida_Bootloader.bin
# Create and Pad Marker section (35KB - 36KB)
# Create marker page with "Lazervida" at the start (to be appended to 36KB location)
echo -n "Lazervida\0" > marker.bin
declare -i MarkerSize=$(ls -al | grep "marker.bin" | awk '{print $5}')
declare -i MARKER_PADDING_SIZE=1024-$MarkerSize # Padding to 36KB
# NOTE: 0o377 = 0xFF
dd if=/dev/zero ibs=1 count=$MARKER_PADDING_SIZE | LC_ALL=C tr "\000" "\377" >> marker.bin
# Merge all
cat Lazervida_Bootloader.bin marker.bin Lazervida.bin > Lazervida_complete_fw.bin
