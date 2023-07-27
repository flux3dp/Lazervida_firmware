#!/bin/zsh
# Pad Bootloader
make
cd /Users/simon/Dev/Lazervida_firmware/build
declare -i BootloaderSize=$(ls -al | grep "Lazervida_Bootloader.bin" | awk '{print $5}')
# if bootloader size < 35kb
if [ $BootloaderSize -lt 35840 ]; then
    declare -i BOOTLOADER_PADDING_SIZE=35840-$BootloaderSize # Padding to the end of 35KB
    # NOTE: 0o377 = 0xFF
    dd if=/dev/zero ibs=1 count=$BOOTLOADER_PADDING_SIZE | LC_ALL=C tr "\000" "\377" >> Lazervida_Bootloader.bin
fi
# Create and Pad Marker section
# Create marker page with "Lazervida" at the start (to be appended to 36KB location)
rm marker.bin
echo -n "Lazervida\0" > marker.bin
declare -i MarkerSize=$(ls -al | grep "marker.bin" | awk '{print $5}')
declare -i MARKER_PADDING_SIZE=1024-$MarkerSize # Padding to the end of 35KB
# NOTE: 0o377 = 0xFF
dd if=/dev/zero ibs=1 count=$MARKER_PADDING_SIZE | LC_ALL=C tr "\000" "\377" >> marker.bin

# Merge all
cat Lazervida_Bootloader.bin marker.bin BeamAirPro.bin > BeamAirPro_Complete_Firmware.bin
st-flash --flash=128K --reset write BeamAirPro_Complete_Firmware.bin 0x8000000