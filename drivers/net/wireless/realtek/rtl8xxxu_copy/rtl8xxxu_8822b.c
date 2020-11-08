#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/wireless.h>
#include <linux/firmware.h>
#include <linux/moduleparam.h>
#include <net/mac80211.h>
#include "rtl8xxxu.h"
#include "rtl8xxxu_regs.h"

/*
 * ====================================================
 *	EEPROM/Efuse PG Offset for 8822B
 * ====================================================
 */
#define	EEPROM_ChannelPlan_8822B		0xB8
#define	EEPROM_XTAL_8822B			0xB9
#define	EEPROM_THERMAL_METER_8822B		0xBA
#define	EEPROM_IQK_LCK_8822B			0xBB
#define	EEPROM_2G_5G_PA_TYPE_8822B		0xBC
/* PATH A & PATH B */
#define	EEPROM_2G_LNA_TYPE_GAIN_SEL_AB_8822B	0xBD
/* PATH C & PATH D */
#define	EEPROM_2G_LNA_TYPE_GAIN_SEL_CD_8822B	0xBE
/* PATH A & PATH B */
#define	EEPROM_5G_LNA_TYPE_GAIN_SEL_AB_8822B	0xBF
/* PATH C & PATH D */
#define	EEPROM_5G_LNA_TYPE_GAIN_SEL_CD_8822B	0xC0

#define	EEPROM_RF_BOARD_OPTION_8822B		0xC1
#define	EEPROM_FEATURE_OPTION_8822B		0xC2
#define	EEPROM_RF_BT_SETTING_8822B		0xC3
#define	EEPROM_VERSION_8822B			0xC4
#define	EEPROM_CustomID_8822B			0xC5
#define	EEPROM_TX_BBSWING_2G_8822B		0xC6
#define	EEPROM_TX_PWR_CALIBRATE_RATE_8822B	0xC8
#define	EEPROM_RF_ANTENNA_OPT_8822B		0xC9
#define	EEPROM_RFE_OPTION_8822B			0xCA
#define EEPROM_COUNTRY_CODE_8822B		0xCB

/* RTL8822BU */
#define EEPROM_MAC_ADDR_8822BU			0x107
#define EEPROM_VID_8822BU			0x100
#define EEPROM_PID_8822BU			0x102
#define EEPROM_USB_OPTIONAL_FUNCTION0_8822BU	0x104
#define EEPROM_USB_MODE_8822BU			0x06



static int rtl8822bu_parse_efuse(struct rtl8xxxu_priv *priv)
{
	struct rtl8822bu_efuse *efuse = &priv->efuse_wifi.efuse8822bu;

	BUILD_BUG_ON(offsetof(struct rtl8822bu_efuse, mac_addr) != 0x107);
	
	if (efuse->rtl_id != cpu_to_le16(0x8129))
		return -EINVAL;

	ether_addr_copy(priv->mac_addr, efuse->mac_addr);

	// printk("FOUND EFUSE: %x:%x:%x:%x:%x:%x", efuse->mac_addr[5],
	// 	efuse->mac_addr[4], efuse->mac_addr[3], efuse->mac_addr[2],
	// 	efuse->mac_addr[1], efuse->mac_addr[0]);
	// return -1;

	return 0;
}

static int rtl8822bu_load_firmware(struct rtl8xxxu_priv *priv)
{
	return rtl8xxxu_load_firmware(priv, "rtlwifi/rtl8822bu_nic.bin");
}

struct rtl8xxxu_fileops rtl8822bu_fops = {
	.parse_efuse = rtl8822bu_parse_efuse,
	.load_firmware = rtl8822bu_load_firmware,
};
