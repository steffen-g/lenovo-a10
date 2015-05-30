# lenovo-a10
Everything for setting up a native Linux on a Lenovo A10 netbook

1. Kernel
Stock Kernel from Lenovo with modified defconfig to boot Linux instead of Android from eMMC flash.
Keyboard and battery driver are patched:

Keyboard:
  * Use normal layout with function keys
  * Search, switch Window and ... Keys are still unchanged
  
Battery:
  * Reports to system as BAT0, charge and discharge current with changed sign, so battery plugins (e.g. from XFCE4) shows correct data)
  
Building Kernel:

ARCH=arm make rk3188_a10_linux_flash_defconfig

ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make Image 
