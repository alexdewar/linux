#!/bin/sh

set -e

RPATH=/run/media/alex/789c240c-f22e-4351-82ce-5b118c5eeebb/
MAKE="make ARCH=arm64 CROSS=aarch64-linux-gnu-"
$MAKE INSTALL_MOD_PATH="$RPATH/usr" modules_install
$MAKE INSTALL_DTBS_PATH="$RPATH/boot/dtbs" dtbs_install

mkimage -D "-I dts -O dtb -p 2048" -f kernel.its vmlinux.uimg

vbutil_kernel \
    --pack vmlinux.kpart \
    --version 1 \
    --vmlinuz vmlinux.uimg \
    --arch aarch64 \
    --keyblock kernel.keyblock \
    --signprivate kernel_data_key.vbprivk \
    --config cmdline \
    --bootloader bootloader.bin

#cp vmlinux.kpart "${pkgdir}/boot"
#depmod -b "$RPATH/usr" -F System.map $(make kernelrelease)

dd if=vmlinux.kpart of=/dev/sdf1
sync
