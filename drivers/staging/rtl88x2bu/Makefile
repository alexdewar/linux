# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_RTL8822BU) += rtl88x2bu.o
rtl88x2bu-objs := os_dep/osdep_service.o os_dep/linux/os_intfs.o os_dep/linux/usb_intf.o os_dep/linux/usb_ops_linux.o os_dep/linux/ioctl_linux.o os_dep/linux/xmit_linux.o os_dep/linux/mlme_linux.o os_dep/linux/recv_linux.o os_dep/linux/rtw_cfgvendor.o os_dep/linux/wifi_regd.o os_dep/linux/rtw_android.o os_dep/linux/rtw_proc.o os_dep/linux/rtw_rhashtable.o os_dep/linux/ioctl_mp.o hal/hal_intf.o hal/hal_com.o hal/hal_com_phycfg.o hal/hal_phy.o hal/hal_dm.o hal/hal_dm_acs.o hal/hal_btcoex_wifionly.o hal/hal_btcoex.o hal/hal_mp.o hal/hal_mcc.o hal/hal_hci/hal_usb.o hal/led/hal_led.o hal/led/hal_usb_led.o platform/platform_ops.o

#ccflags-$(CONFIG_RTL8822BU) += -I$(srctree)/$(src)/include
subdir-ccflags-y += -I$(srctree)/$(src)/{include,hal/phydm,platform} \
	-Wno-undef -Wno-unused-variable
