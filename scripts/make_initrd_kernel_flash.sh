#!/bin/bash
cd ../kernel/
make Image -j4
sudo update-initramfs -u -v -k 3.0.36
cp -v initrd.img-3.0.36 ../initrd/
cd ../scripts
ls -al ../initrd/*
ls -al ../out/*.img
./make_boot_img.sh
./flash_mtd.sh
