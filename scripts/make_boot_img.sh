#!/bin/bash
#update-initramfs -u -v -k 3.0.36
../rockchip-mkbootimg/mkbootimg  --pagesize 16384 --kernel ../kernel/arch/arm/boot/Image --ramdisk ../initrd/initrd.img-3.0.36 -o ../out/boot.img
#./rkflashtool w 0x008000 0x00008000 < ../rockchip-mkbootimg/boot.img
#dd if=boot.img of=/dev/mtdblock2  bs=4M
