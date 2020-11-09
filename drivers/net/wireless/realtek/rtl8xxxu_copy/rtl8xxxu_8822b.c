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

static struct rtl8xxxu_reg8val rtl8822b_mac_init_table[] = {
	{ 0x0029, 0xf9 }, { 0x0420, 0x80 }, { 0x0421, 0x1f }, { 0x0428, 0x0a },
	{ 0x0429, 0x10 }, { 0x0430, 0x00 }, { 0x0431, 0x00 }, { 0x0432, 0x00 },
	{ 0x0433, 0x01 }, { 0x0434, 0x04 }, { 0x0435, 0x05 }, { 0x0436, 0x07 },
	{ 0x0437, 0x08 }, { 0x043c, 0x04 }, { 0x043d, 0x05 }, { 0x043e, 0x07 },
	{ 0x043f, 0x08 }, { 0x0440, 0x5d }, { 0x0441, 0x01 }, { 0x0442, 0x00 },
	{ 0x0444, 0x10 }, { 0x0445, 0xf0 }, { 0x0446, 0x01 }, { 0x0447, 0xfe },
	{ 0x0448, 0x00 }, { 0x0449, 0x00 }, { 0x044a, 0x00 }, { 0x044b, 0x40 },
	{ 0x044c, 0x10 }, { 0x044d, 0xf0 }, { 0x044e, 0x3f }, { 0x044f, 0x00 },
	{ 0x0450, 0x00 }, { 0x0451, 0x00 }, { 0x0452, 0x00 }, { 0x0453, 0x40 },
	{ 0x0455, 0x70 }, { 0x045e, 0x04 }, { 0x049c, 0x10 }, { 0x049d, 0xf0 },
	{ 0x049e, 0x00 }, { 0x049f, 0x06 }, { 0x04a0, 0xe0 }, { 0x04a1, 0x03 },
	{ 0x04a2, 0x00 }, { 0x04a3, 0x40 }, { 0x04a4, 0x15 }, { 0x04a5, 0xf0 },
	{ 0x04a6, 0x00 }, { 0x04a7, 0x06 }, { 0x04a8, 0xe0 }, { 0x04a9, 0x00 },
	{ 0x04aa, 0x00 }, { 0x04ab, 0x00 }, { 0x07da, 0x08 }, { 0x1448, 0x06 },
	{ 0x144a, 0x06 }, { 0x144c, 0x06 }, { 0x144e, 0x06 }, { 0x04c8, 0xff },
	{ 0x04c9, 0x08 }, { 0x04ca, 0x20 }, { 0x04cb, 0x20 }, { 0x04cc, 0xff },
	{ 0x04cd, 0xff }, { 0x04ce, 0x01 }, { 0x04cf, 0x08 }, { 0x0500, 0x26 },
	{ 0x0501, 0xa2 }, { 0x0502, 0x2f }, { 0x0503, 0x00 }, { 0x0504, 0x28 },
	{ 0x0505, 0xa3 }, { 0x0506, 0x5e }, { 0x0507, 0x00 }, { 0x0508, 0x2b },
	{ 0x0509, 0xa4 }, { 0x050a, 0x5e }, { 0x050b, 0x00 }, { 0x050c, 0x4f },
	{ 0x050d, 0xa4 }, { 0x050e, 0x00 }, { 0x050f, 0x00 }, { 0x0512, 0x1c },
	{ 0x0514, 0x0a }, { 0x0516, 0x0a }, { 0x0521, 0x2f }, { 0x0525, 0x4f },
	{ 0x0551, 0x10 }, { 0x0559, 0x02 }, { 0x055c, 0x50 }, { 0x055d, 0xff },
	{ 0x0577, 0x0b }, { 0x05be, 0x64 }, { 0x0605, 0x30 }, { 0x0608, 0x0e },
	{ 0x0609, 0x22 }, { 0x060c, 0x18 }, { 0x06a0, 0xff }, { 0x06a1, 0xff },
	{ 0x06a2, 0xff }, { 0x06a3, 0xff }, { 0x06a4, 0xff }, { 0x06a5, 0xff },
	{ 0x06de, 0x84 }, { 0x0620, 0xff }, { 0x0621, 0xff }, { 0x0622, 0xff },
	{ 0x0623, 0xff }, { 0x0624, 0xff }, { 0x0625, 0xff }, { 0x0626, 0xff },
	{ 0x0627, 0xff }, { 0x0638, 0x50 }, { 0x063c, 0x0a }, { 0x063d, 0x0a },
	{ 0x063e, 0x0e }, { 0x063f, 0x0e }, { 0x0640, 0x40 }, { 0x0642, 0x40 },
	{ 0x0643, 0x00 }, { 0x0652, 0xc8 }, { 0x066e, 0x05 }, { 0x0718, 0x40 },
	{ 0x07d4, 0x98 },
	{ 0xffff, 0xff },
};

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
	// ...
	.mactable = rtl8822b_mac_init_table,
};
