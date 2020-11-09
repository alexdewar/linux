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

static int rtl8822bu_power_on(struct rtl8xxxu_priv *priv)
{
	// 	u32 value32;
	// 	struct halmac_api *api = (struct halmac_api *)adapter->halmac_api;
	// 	u8 enable_bb;
	u32 val32;
	u8 val8;

	// 	PLTFM_MSG_TRACE("[TRACE]%s ===>\n", __func__);

	// 	HALMAC_REG_W8(REG_RSV_CTRL, 0);
	rtl8xxxu_write8(priv, REG_RSV_CTRL, 0x0);

// 	if (adapter->intf == HALMAC_INTERFACE_SDIO) {
// #if HALMAC_SDIO_SUPPORT
// 		if (leave_sdio_suspend_88xx(adapter) != HALMAC_RET_SUCCESS)
// 			return HALMAC_RET_SDIO_LEAVE_SUSPEND_FAIL;
// #endif
// 	} else if (adapter->intf == HALMAC_INTERFACE_USB) {
// #if HALMAC_USB_SUPPORT
// 		if (HALMAC_REG_R8(REG_SYS_CFG2 + 3) == 0x20)
// 			HALMAC_REG_W8(0xFE5B, HALMAC_REG_R8(0xFE5B) | BIT(4));
	if (rtl8xxxu_read8(priv, REG_SYS_CFG2 + 3) == 0x20) {
		printk("DOING EXTRA THING");
		val8 = rtl8xxxu_read8(priv, 0xfe5b);
		val8 |= BIT(4);
		rtl8xxxu_write8(priv, 0xfe5b, val8);
	} else {
		printk("NOT DOING EXTRA THING");
	}
// #endif
// 	} else if (adapter->intf == HALMAC_INTERFACE_PCIE) {
// #if HALMAC_PCIE_SUPPORT
// 		/* For PCIE power on fail issue */
// 		HALMAC_REG_W8(REG_HCI_OPT_CTRL + 1,
// 			      HALMAC_REG_R8(REG_HCI_OPT_CTRL + 1) | BIT(0));
// #endif
// 	}

// 	/* Config PIN Mux */
// 	value32 = HALMAC_REG_R32(REG_PAD_CTRL1);
// 	value32 = value32 & (~(BIT(28) | BIT(29)));
// 	value32 = value32 | BIT(28) | BIT(29);
// 	HALMAC_REG_W32(REG_PAD_CTRL1, value32);
	val32 = rtl8xxxu_read32(priv, REG_PAD_CTRL1);
	val32 |= BIT(28) | BIT(29);
	rtl8xxxu_write32(priv, REG_PAD_CTRL1, val32);

// 	value32 = HALMAC_REG_R32(REG_LED_CFG);
// 	value32 = value32 & (~(BIT(25) | BIT(26)));
// 	HALMAC_REG_W32(REG_LED_CFG, value32);
	val32 = rtl8xxxu_read32(priv, REG_LEDCFG0);
	val32 &= ~(BIT(25) | BIT(26));
	rtl8xxxu_write32(priv, REG_LEDCFG0, val32);

// 	value32 = HALMAC_REG_R32(REG_GPIO_MUXCFG);
// 	value32 = value32 & (~(BIT(2)));
// 	value32 = value32 | BIT(2);
// 	HALMAC_REG_W32(REG_GPIO_MUXCFG, value32);
	val32 = rtl8xxxu_read32(priv, REG_GPIO_MUXCFG);
	val32 |= BIT(2);
	rtl8xxxu_write32(priv, REG_GPIO_MUXCFG, val32);

// 	enable_bb = 0;
// 	set_hw_value_88xx(adapter, HALMAC_HW_EN_BB_RF, &enable_bb);

// 	if (HALMAC_REG_R8(REG_SYS_CFG1 + 2) & BIT(4)) {
// 		PLTFM_MSG_ERR("[ERR]test mode!!\n");
// 		return HALMAC_RET_WLAN_MODE_FAIL;
// 	}

	val8 = rtl8xxxu_read8(priv, 0x00f2);
	if (val8 & BIT(4))
		return -1;

	// 	PLTFM_MSG_TRACE("[TRACE]%s <===\n", __func__);

	// 	return HALMAC_RET_SUCCESS;
	return 0;
}

struct rtl8xxxu_fileops rtl8822bu_fops = {
	.parse_efuse = rtl8822bu_parse_efuse,
	.load_firmware = rtl8822bu_load_firmware,
	.power_on = rtl8822bu_power_on,
};
