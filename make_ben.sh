#!/bin/bash

echo "build kernel"
source /opt/poky/1.6.1/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi
export ARCH=arm
export CROSS_COMPILE=arm-poky-linux-gnueabi-

#make imx_v7_defconfig
make -j4 uImage LOADADDR=0x10008000
make imx6q-sabresd.dtb
#make modules

rm uImage
rm *.dtb
cp arch/arm/boot/uImage ./
cp arch/arm/boot/dts/*.dtb ./

echo "build end"
