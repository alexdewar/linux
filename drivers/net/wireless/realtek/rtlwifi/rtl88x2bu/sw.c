// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/usb.h>
#include <net/mac80211.h>

// #include "../usb.h"
#include "../wifi.h"
#include "reg.h"

#define REALTEK_USB_VENQT_CMD_REQ              0x05
#define        REALTEK_USB_VENQT_CMD_IDX               0x00
#define        REALTEK_USB_VENQT_READ                  0xC0
#define MAX_USBCTRL_VENDORREQ_TIMES            10

static int _usbctrl_vendorreq_sync_read(struct usb_device *udev, u8 request,
					u16 value, u16 index, void *pdata,
					u16 len)
{
	unsigned int pipe;
	int status;
	u8 reqtype;
	int vendorreq_times = 0;
	static int count;

	pipe = usb_rcvctrlpipe(udev, 0); /* read_in */
	reqtype =  REALTEK_USB_VENQT_READ;

	do {
		status = usb_control_msg(udev, pipe, request, reqtype, value,
					 index, pdata, len, 1000);
		if (status < 0) {
		// 	/* firmware download is checksumed, don't retry */
		// 	if ((value >= FW_8192C_START_ADDRESS &&
		// 	    value <= FW_8192C_END_ADDRESS))
		// 		break;
		// } else {
			break;
		}
	} while (++vendorreq_times < MAX_USBCTRL_VENDORREQ_TIMES);

	if (status < 0 && count++ < 4)
		pr_err("reg 0x%x, usbctrl_vendorreq TimeOut! status:0x%x value=0x%x\n",
		       value, status, *(u32 *)pdata);
	return status;
}

static u32 _usb_read_sync(struct usb_device *udev, u32 addr, u16 len)
{
	u8 request;
	u16 wvalue;
	u16 index;
	__le32 data;
	__le32 *pdata = kmalloc(4, GFP_KERNEL);

	request = REALTEK_USB_VENQT_CMD_REQ;
	index = REALTEK_USB_VENQT_CMD_IDX; /* n/a */

	wvalue = (u16)addr;
	_usbctrl_vendorreq_sync_read(udev, request, wvalue, index, pdata, len);
	data = *pdata;
	kfree(pdata);
	return le32_to_cpu(data);
}

static u32 _usb_read32_sync(struct usb_device *udev, u32 addr)
{
	return _usb_read_sync(udev, addr, 4);
}

void read_chip_version(struct usb_device *udev)
{
	// struct rtl_priv *rtlpriv = rtl_priv(hw);
	// struct rtl_phy *rtlphy = &(rtlpriv->phy);
	// struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	// value32 = rtw_read32(adapter, REG_SYS_CFG1_8822B);
	// hal->version_id.ICType = CHIP_8822B;
	// hal->version_id.ChipType = ((value32 & BIT_RTL_ID_8822B) ? TEST_CHIP : NORMAL_CHIP);
	// hal->version_id.CUTVersion = BIT_GET_CHIP_VER_8822B(value32);
	// hal->version_id.VendorType = BIT_GET_VENDOR_ID_8822B(value32);
	// hal->version_id.VendorType >>= 2;
	// switch (hal->version_id.VendorType) {
	// case 0:
	// 	hal->version_id.VendorType = CHIP_VENDOR_TSMC;
	// 	break;
	// case 1:
	// 	hal->version_id.VendorType = CHIP_VENDOR_SMIC;
	// 	break;
	// case 2:
	// 	hal->version_id.VendorType = CHIP_VENDOR_UMC;
	// 	break;
	// }

	u32 version, cut_version, vendor_type;
	bool is_test, is_2t2r;

	version = _usb_read32_sync(udev, REG_SYS_CFG1);
	printk("CHIP VERSION: %d", version);
	
	is_test = (version & TRP_VAUX_EN);
	cut_version = BIT_GET_CHIP_VER_8822B(version);
	vendor_type = BIT_GET_VENDOR_ID_8822B(version) >> 2;
	is_2t2r = (version & BIT_RF_TYPE_ID_8822B);
	
	printk("ALEX: IS_TEST: %d", is_test);
	printk("ALEX: CUT_VERSION: %d", cut_version);
	printk("ALEX: VENDOR_TYPE: %d", vendor_type);
	printk("ALEX: IS_2T2R: %d", is_2t2r);
}

int rtl_usb_probe_88x2bu(struct usb_interface *intf,
			 const struct usb_device_id *id);

static int rtl88x2bu_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;
	struct usb_device *udev = interface_to_usbdev(intf);
	int err;

	err = usb_find_common_endpoints(intf->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
	if (err)
		printk("Could not find endpoints :-(");
	else {
		if (intf->cur_altsetting->string)
			printk("NAME: %s", intf->cur_altsetting->string);
		else
			printk("NO NAME FOUND :-(");
	}

	read_chip_version(udev);
	return 0;
}

static void rtl88x2bu_disconnect(struct usb_interface *interface)
{
	printk("RTL88X2BU DISCONNECTED!!!!!!!!!!!!!");
}


#define USB_VENDOR_ID_REALTEK		0x0bda

static const struct usb_device_id rtl88x2bu_usb_ids[] = {
	{USB_DEVICE(USB_VENDOR_ID_REALTEK, 0xb812)},
	{}
};
MODULE_DEVICE_TABLE(usb, rtl88x2bu_usb_ids);

static struct usb_driver rtl88x2bu_driver = {
	.name = "rtl88x2bu",
	.probe = rtl88x2bu_probe,
	.disconnect = rtl88x2bu_disconnect, 
	.id_table = rtl88x2bu_usb_ids,
};

module_usb_driver(rtl88x2bu_driver);

MODULE_AUTHOR("Alex Dewar <alex.dewar90@gmail.com>");
MODULE_LICENSE("GPL");
