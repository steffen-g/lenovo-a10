# lenovo-a10
Everything for setting up a native Linux on a Lenovo A10 netbook

![LTSpice in wine on armhf on a Lenovo A10](http://gsg-elektronik.de/~steffen/IMG_20150530_125345.jpg)


###1. Kernel

Stock Kernel from Lenovo with modified defconfig to boot Linux instead of Android from eMMC flash.
Keyboard and battery driver are patched:

Keyboard:
  * Use normal layout with function keys
  * Search, switch Window and ... Keys are still unchanged
  
Battery:
  * Reports to system as BAT0, charge and discharge current with changed sign, so battery plugins (e.g. from XFCE4) show correct data
  
####Building Kernel

```
cd kernel
ARCH=arm make rk3188_a10_linux_flash_defconfig
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make Image 
```

###2. WIFI Driver RTL8723AS

From https://github.com/Android4SAM/platform_hardware_realtek/tree/master/cm8723u/wlan/rtl8723A_WiFi_linux_v4.1.3_6044.20121224

Modified for rtl8723as and Lenovo A10 by Steffen Graf

####Building Kernel Module
When arm-linux-gnueabihf- cross compiler is installed and kernel source is in ../kernel/ simply enter make and a kernel module will be built.

```
cd rtl8723as
make
```

###3. Bootimage
Mkbootimage from https://github.com/neo-technologies/rockchip-mkbootimg is used.

Generating boot.img using compiled Kernel and prebuilt initrd including fbcon, wifi driver and nanddriver

####Building mkbootimg
```
cd rockchip-mkbootimg
make
```

####Generating Bootimage using mkbootimg
```
cd scripts
mkdir ../out
./make_boot_img.sh
````

###4. Flashtool
rkflashtool from http://sourceforge.net/projects/rkflashtool/

####Building flashtool
```
cd rkflashtool-5.1-src
make
```

####Flashing the boot.img
When flashing via USB:
```
cd scripts
./flash_usb.sh
```

When running the script on the A10:
```
cd scripts
./flash_mtd.sh
```

Both scripts must be run as root.


