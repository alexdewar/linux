/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#define _RTW_MP_C_
#include <drv_types.h>

#include "../hal/phydm/phydm_precomp.h"


u32 read_macreg(_adapter *padapter, u32 addr, u32 sz)
{
	u32 val = 0;

	switch (sz) {
	case 1:
		val = rtw_read8(padapter, addr);
		break;
	case 2:
		val = rtw_read16(padapter, addr);
		break;
	case 4:
		val = rtw_read32(padapter, addr);
		break;
	default:
		val = 0xffffffff;
		break;
	}

	return val;

}

void write_macreg(_adapter *padapter, u32 addr, u32 val, u32 sz)
{
	switch (sz) {
	case 1:
		rtw_write8(padapter, addr, (u8)val);
		break;
	case 2:
		rtw_write16(padapter, addr, (u16)val);
		break;
	case 4:
		rtw_write32(padapter, addr, val);
		break;
	default:
		break;
	}

}

u32 read_bbreg(_adapter *padapter, u32 addr, u32 bitmask)
{
	return rtw_hal_read_bbreg(padapter, addr, bitmask);
}

void write_bbreg(_adapter *padapter, u32 addr, u32 bitmask, u32 val)
{
	rtw_hal_write_bbreg(padapter, addr, bitmask, val);
}

u32 _read_rfreg(PADAPTER padapter, u8 rfpath, u32 addr, u32 bitmask)
{
	return rtw_hal_read_rfreg(padapter, rfpath, addr, bitmask);
}

void _write_rfreg(PADAPTER padapter, u8 rfpath, u32 addr, u32 bitmask, u32 val)
{
	rtw_hal_write_rfreg(padapter, rfpath, addr, bitmask, val);
}

u32 read_rfreg(PADAPTER padapter, u8 rfpath, u32 addr)
{
	return _read_rfreg(padapter, rfpath, addr, bRFRegOffsetMask);
}

void write_rfreg(PADAPTER padapter, u8 rfpath, u32 addr, u32 val)
{
	_write_rfreg(padapter, rfpath, addr, bRFRegOffsetMask, val);
}

static void _init_mp_priv_(struct mp_priv *pmp_priv)
{
	WLAN_BSSID_EX *pnetwork;

	_rtw_memset(pmp_priv, 0, sizeof(struct mp_priv));

	pmp_priv->mode = MP_OFF;

	pmp_priv->channel = 1;
	pmp_priv->bandwidth = CHANNEL_WIDTH_20;
	pmp_priv->prime_channel_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
	pmp_priv->rateidx = RATE_1M;
	pmp_priv->txpoweridx = 0x2A;

	pmp_priv->antenna_tx = ANTENNA_A;
	pmp_priv->antenna_rx = ANTENNA_AB;

	pmp_priv->check_mp_pkt = 0;

	pmp_priv->tx_pktcount = 0;

	pmp_priv->rx_bssidpktcount = 0;
	pmp_priv->rx_pktcount = 0;
	pmp_priv->rx_crcerrpktcount = 0;

	pmp_priv->network_macaddr[0] = 0x00;
	pmp_priv->network_macaddr[1] = 0xE0;
	pmp_priv->network_macaddr[2] = 0x4C;
	pmp_priv->network_macaddr[3] = 0x87;
	pmp_priv->network_macaddr[4] = 0x66;
	pmp_priv->network_macaddr[5] = 0x55;

	pmp_priv->bSetRxBssid = _FALSE;
	pmp_priv->bRTWSmbCfg = _FALSE;
	pmp_priv->bloopback = _FALSE;

	pmp_priv->bloadefusemap = _FALSE;
	pmp_priv->brx_filter_beacon = _FALSE;
	pmp_priv->mplink_brx = _FALSE;

	pnetwork = &pmp_priv->mp_network.network;
	_rtw_memcpy(pnetwork->MacAddress, pmp_priv->network_macaddr, ETH_ALEN);

	pnetwork->Ssid.SsidLength = 8;
	_rtw_memcpy(pnetwork->Ssid.Ssid, "mp_871x", pnetwork->Ssid.SsidLength);

	pmp_priv->tx.payload = MP_TX_Payload_default_random;
	pmp_priv->tx.attrib.ht_en = 1;

	pmp_priv->mpt_ctx.mpt_rate_index = 1;

}


static void mp_init_xmit_attrib(struct mp_tx *pmptx, PADAPTER padapter)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	struct pkt_attrib *pattrib;

	/* init xmitframe attribute */
	pattrib = &pmptx->attrib;
	_rtw_memset(pattrib, 0, sizeof(struct pkt_attrib));
	_rtw_memset(pmptx->desc, 0, TXDESC_SIZE);

	pattrib->ether_type = 0x8712;
#if 0
	_rtw_memcpy(pattrib->src, adapter_mac_addr(padapter), ETH_ALEN);
	_rtw_memcpy(pattrib->ta, pattrib->src, ETH_ALEN);
#endif
	_rtw_memset(pattrib->dst, 0xFF, ETH_ALEN);

	/*	pattrib->dhcp_pkt = 0;
	 *	pattrib->pktlen = 0; */
	pattrib->ack_policy = 0;
	/*	pattrib->pkt_hdrlen = ETH_HLEN; */
	pattrib->hdrlen = WLAN_HDR_A3_LEN;
	pattrib->subtype = WIFI_DATA;
	pattrib->priority = 0;
	pattrib->qsel = pattrib->priority;
	/*	do_queue_select(padapter, pattrib); */
	pattrib->nr_frags = 1;
	pattrib->encrypt = 0;
	pattrib->bswenc = _FALSE;
	pattrib->qos_en = _FALSE;

	pattrib->pktlen = 1500;

	if (pHalData->rf_type == RF_2T2R)
		pattrib->raid = RATEID_IDX_BGN_40M_2SS;
	else
		pattrib->raid = RATEID_IDX_BGN_40M_1SS;

	if (pHalData->rf_type == RF_1T1R)
		pattrib->raid = RATEID_IDX_VHT_1SS;
	else if (pHalData->rf_type == RF_2T2R || pHalData->rf_type == RF_2T4R)
		pattrib->raid = RATEID_IDX_VHT_2SS;
	else if (pHalData->rf_type == RF_3T3R)
		pattrib->raid = RATEID_IDX_VHT_3SS;
	else
		pattrib->raid = RATEID_IDX_BGN_40M_1SS;
}

s32 init_mp_priv(PADAPTER padapter)
{
	struct mp_priv *pmppriv = &padapter->mppriv;
	PHAL_DATA_TYPE pHalData;

	pHalData = GET_HAL_DATA(padapter);

	_init_mp_priv_(pmppriv);
	pmppriv->papdater = padapter;
	if (IS_HARDWARE_TYPE_8822C(padapter))
		pmppriv->mp_dm = 1;/* default enable dpk tracking */
	else
		pmppriv->mp_dm = 0;

	pmppriv->tx.stop = 1;
	pmppriv->bSetTxPower = 0;		/*for  manually set tx power*/
	pmppriv->bTxBufCkFail = _FALSE;
	pmppriv->pktInterval = 0;
	pmppriv->pktLength = 1000;
	pmppriv->bprocess_mp_mode = _FALSE;

	mp_init_xmit_attrib(&pmppriv->tx, padapter);

	switch (GET_HAL_RFPATH(padapter)) {
	case RF_1T1R:
		pmppriv->antenna_tx = ANTENNA_A;
		pmppriv->antenna_rx = ANTENNA_A;
		break;
	case RF_1T2R:
	default:
		pmppriv->antenna_tx = ANTENNA_A;
		pmppriv->antenna_rx = ANTENNA_AB;
		break;
	case RF_2T2R:
		pmppriv->antenna_tx = ANTENNA_AB;
		pmppriv->antenna_rx = ANTENNA_AB;
		break;
	case RF_2T4R:
		pmppriv->antenna_tx = ANTENNA_BC;
		pmppriv->antenna_rx = ANTENNA_ABCD;
		break;
	}

	pHalData->AntennaRxPath = pmppriv->antenna_rx;
	pHalData->antenna_tx_path = pmppriv->antenna_tx;

	return _SUCCESS;
}

void free_mp_priv(struct mp_priv *pmp_priv)
{
	if (pmp_priv->pallocated_mp_xmitframe_buf) {
		rtw_mfree(pmp_priv->pallocated_mp_xmitframe_buf, 0);
		pmp_priv->pallocated_mp_xmitframe_buf = NULL;
	}
	pmp_priv->pmp_xmtframe_buf = NULL;
}

#if 0
static void PHY_IQCalibrate_default(
		PADAPTER	pAdapter,
		BOOLEAN	bReCovery
)
{
	RTW_INFO("%s\n", __func__);
}

static void PHY_LCCalibrate_default(
		PADAPTER	pAdapter
)
{
	RTW_INFO("%s\n", __func__);
}

static void PHY_SetRFPathSwitch_default(
		PADAPTER	pAdapter,
		BOOLEAN		bMain
)
{
	RTW_INFO("%s\n", __func__);
}
#endif

void mpt_InitHWConfig(PADAPTER Adapter)
{
	PHAL_DATA_TYPE hal;

	hal = GET_HAL_DATA(Adapter);

	if (IS_HARDWARE_TYPE_8723B(Adapter)) {
		/* TODO: <20130114, Kordan> The following setting is only for DPDT and Fixed board type. */
		/* TODO:  A better solution is configure it according EFUSE during the run-time. */

		phy_set_mac_reg(Adapter, 0x64, BIT20, 0x0);		/* 0x66[4]=0		 */
		phy_set_mac_reg(Adapter, 0x64, BIT24, 0x0);		/* 0x66[8]=0 */
		phy_set_mac_reg(Adapter, 0x40, BIT4, 0x0);		/* 0x40[4]=0		 */
		phy_set_mac_reg(Adapter, 0x40, BIT3, 0x1);		/* 0x40[3]=1		 */
		phy_set_mac_reg(Adapter, 0x4C, BIT24, 0x1);		/* 0x4C[24:23]=10 */
		phy_set_mac_reg(Adapter, 0x4C, BIT23, 0x0);		/* 0x4C[24:23]=10 */
		phy_set_bb_reg(Adapter, 0x944, BIT1 | BIT0, 0x3);	/* 0x944[1:0]=11	 */
		phy_set_bb_reg(Adapter, 0x930, bMaskByte0, 0x77);/* 0x930[7:0]=77	  */
		phy_set_mac_reg(Adapter, 0x38, BIT11, 0x1);/* 0x38[11]=1 */

		/* TODO: <20130206, Kordan> The default setting is wrong, hard-coded here. */
		phy_set_mac_reg(Adapter, 0x778, 0x3, 0x3);					/* Turn off hardware PTA control (Asked by Scott) */
		phy_set_mac_reg(Adapter, 0x64, bMaskDWord, 0x36000000);/* Fix BT S0/S1 */
		phy_set_mac_reg(Adapter, 0x948, bMaskDWord, 0x0);		/* Fix BT can't Tx */

		/* <20130522, Kordan> Turn off equalizer to improve Rx sensitivity. (Asked by EEChou) */
		phy_set_bb_reg(Adapter, 0xA00, BIT8, 0x0);			/*0xA01[0] = 0*/
	} else if (IS_HARDWARE_TYPE_8821(Adapter)) {
		/* <20131121, VincentL> Add for 8821AU DPDT setting and fix switching antenna issue (Asked by Rock)
		<20131122, VincentL> Enable for all 8821A/8811AU  (Asked by Alex)*/
		phy_set_mac_reg(Adapter, 0x4C, BIT23, 0x0);		/*0x4C[23:22]=01*/
		phy_set_mac_reg(Adapter, 0x4C, BIT22, 0x1);		/*0x4C[23:22]=01*/
	} else if (IS_HARDWARE_TYPE_8188ES(Adapter))
		phy_set_mac_reg(Adapter, 0x4C , BIT23, 0);		/*select DPDT_P and DPDT_N as output pin*/



	else if (IS_HARDWARE_TYPE_8822B(Adapter)) {
		u32 tmp_reg = 0;

		PlatformEFIOWrite2Byte(Adapter, REG_RXFLTMAP1_8822B, 0x2000);
		/* fixed wifi can't 2.4g tx suggest by Szuyitasi 20160504 */
		phy_set_bb_reg(Adapter, 0x70, bMaskByte3, 0x0e);
		RTW_INFO(" 0x73 = 0x%x\n", phy_query_bb_reg(Adapter, 0x70, bMaskByte3));
		phy_set_bb_reg(Adapter, 0x1704, bMaskDWord, 0x0000ff00);
		RTW_INFO(" 0x1704 = 0x%x\n", phy_query_bb_reg(Adapter, 0x1704, bMaskDWord));
		phy_set_bb_reg(Adapter, 0x1700, bMaskDWord, 0xc00f0038);
		RTW_INFO(" 0x1700 = 0x%x\n", phy_query_bb_reg(Adapter, 0x1700, bMaskDWord));
	}

}

static void PHY_IQCalibrate(PADAPTER padapter, u8 bReCovery)
{
	halrf_iqk_trigger(&(GET_HAL_DATA(padapter)->odmpriv), bReCovery);
}

static void PHY_LCCalibrate(PADAPTER padapter)
{
	halrf_lck_trigger(&(GET_HAL_DATA(padapter)->odmpriv));
}

static u8 PHY_QueryRFPathSwitch(PADAPTER padapter)
{
	u8 bmain = 0;
/*
	if (IS_HARDWARE_TYPE_8723B(padapter)) {
#ifdef CONFIG_RTL8723B
		bmain = PHY_QueryRFPathSwitch_8723B(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8188E(padapter)) {
#ifdef CONFIG_RTL8188E
		bmain = PHY_QueryRFPathSwitch_8188E(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8814A(padapter)) {
#ifdef CONFIG_RTL8814A
		bmain = PHY_QueryRFPathSwitch_8814A(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8812(padapter) || IS_HARDWARE_TYPE_8821(padapter)) {
#if defined(CONFIG_RTL8812A) || defined(CONFIG_RTL8821A)
		bmain = PHY_QueryRFPathSwitch_8812A(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8192E(padapter)) {
#ifdef CONFIG_RTL8192E
		bmain = PHY_QueryRFPathSwitch_8192E(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8703B(padapter)) {
#ifdef CONFIG_RTL8703B
		bmain = PHY_QueryRFPathSwitch_8703B(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8188F(padapter)) {
#ifdef CONFIG_RTL8188F
		bmain = PHY_QueryRFPathSwitch_8188F(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8188GTV(padapter)) {
#ifdef CONFIG_RTL8188GTV
		bmain = PHY_QueryRFPathSwitch_8188GTV(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8822B(padapter)) {
#ifdef CONFIG_RTL8822B
		bmain = PHY_QueryRFPathSwitch_8822B(padapter);
#endif
	} else if (IS_HARDWARE_TYPE_8723D(padapter)) {
#ifdef CONFIG_RTL8723D
		bmain = PHY_QueryRFPathSwitch_8723D(padapter);
#endif
	} else
*/

	if (IS_HARDWARE_TYPE_8821C(padapter)) {
	}

	return bmain;
}

static void  PHY_SetRFPathSwitch(PADAPTER padapter , BOOLEAN bMain) {

	PHAL_DATA_TYPE hal = GET_HAL_DATA(padapter);
	struct dm_struct *phydm = &hal->odmpriv;

	if (IS_HARDWARE_TYPE_8723B(padapter)) {
	} else if (IS_HARDWARE_TYPE_8188E(padapter)) {
	} else if (IS_HARDWARE_TYPE_8814A(padapter)) {
	} else if (IS_HARDWARE_TYPE_8812(padapter) || IS_HARDWARE_TYPE_8821(padapter)) {
	} else if (IS_HARDWARE_TYPE_8192E(padapter)) {
	} else if (IS_HARDWARE_TYPE_8703B(padapter)) {
#ifdef CONFIG_RTL8703B
		phy_set_rf_path_switch_8703b(phydm, bMain);
#endif
	} else if (IS_HARDWARE_TYPE_8188F(padapter) || IS_HARDWARE_TYPE_8188GTV(padapter)) {
	} else if (IS_HARDWARE_TYPE_8192F(padapter)) {
	} else if (IS_HARDWARE_TYPE_8822B(padapter)) {
		phy_set_rf_path_switch_8822b(phydm, bMain);
	} else if (IS_HARDWARE_TYPE_8723D(padapter)) {
	} else if (IS_HARDWARE_TYPE_8821C(padapter)) {
	} else if (IS_HARDWARE_TYPE_8822C(padapter)) {
	} else if (IS_HARDWARE_TYPE_8814B(padapter)) {
	}
}


static void phy_switch_rf_path_set(PADAPTER padapter , u8 *prf_set_State) {

}



s32
MPT_InitializeAdapter(
		PADAPTER			pAdapter,
		u8				Channel
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	s32		rtStatus = _SUCCESS;
	PMPT_CONTEXT	pMptCtx = &pAdapter->mppriv.mpt_ctx;
	u32		ledsetting;

	pMptCtx->bMptDrvUnload = _FALSE;
	pMptCtx->bMassProdTest = _FALSE;
	pMptCtx->bMptIndexEven = _TRUE;	/* default gain index is -6.0db */
	pMptCtx->h2cReqNum = 0x0;
	/* init for BT MP */

	mpt_InitHWConfig(pAdapter);


	pMptCtx->bMptWorkItemInProgress = _FALSE;
	pMptCtx->CurrMptAct = NULL;
	pMptCtx->mpt_rf_path = RF_PATH_A;
	/* ------------------------------------------------------------------------- */
	/* Don't accept any packets */
	rtw_write32(pAdapter, REG_RCR, 0);

	/* ledsetting = rtw_read32(pAdapter, REG_LEDCFG0); */
	/* rtw_write32(pAdapter, REG_LEDCFG0, ledsetting & ~LED0DIS); */

	/* rtw_write32(pAdapter, REG_LEDCFG0, 0x08080); */
	ledsetting = rtw_read32(pAdapter, REG_LEDCFG0);


	PHY_LCCalibrate(pAdapter);
	PHY_IQCalibrate(pAdapter, _FALSE);
	/* dm_check_txpowertracking(&pHalData->odmpriv);	*/ /* trigger thermal meter */

	PHY_SetRFPathSwitch(pAdapter, 1/*pHalData->bDefaultAntenna*/); /* default use Main */

	pMptCtx->backup0xc50 = (u8)phy_query_bb_reg(pAdapter, rOFDM0_XAAGCCore1, bMaskByte0);
	pMptCtx->backup0xc58 = (u8)phy_query_bb_reg(pAdapter, rOFDM0_XBAGCCore1, bMaskByte0);
	pMptCtx->backup0xc30 = (u8)phy_query_bb_reg(pAdapter, rOFDM0_RxDetector1, bMaskByte0);
	pMptCtx->backup0x52_RF_A = (u8)phy_query_rf_reg(pAdapter, RF_PATH_A, RF_0x52, 0x000F0);
	pMptCtx->backup0x52_RF_B = (u8)phy_query_rf_reg(pAdapter, RF_PATH_B, RF_0x52, 0x000F0);
	return	rtStatus;
}

/*-----------------------------------------------------------------------------
 * Function:	MPT_DeInitAdapter()
 *
 * Overview:	Extra DeInitialization for Mass Production Test.
 *
 * Input:		PADAPTER	pAdapter
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	05/08/2007	MHC		Create Version 0.
 *	05/18/2007	MHC		Add normal driver MPHalt code.
 *
 *---------------------------------------------------------------------------*/
void
MPT_DeInitAdapter(
		PADAPTER	pAdapter
)
{
	PMPT_CONTEXT		pMptCtx = &pAdapter->mppriv.mpt_ctx;

	pMptCtx->bMptDrvUnload = _TRUE;
#if 0 /* for Windows */
	PlatformFreeWorkItem(&(pMptCtx->MptWorkItem));

	while (pMptCtx->bMptWorkItemInProgress) {
		if (NdisWaitEvent(&(pMptCtx->MptWorkItemEvent), 50))
			break;
	}
	NdisFreeSpinLock(&(pMptCtx->MptWorkItemSpinLock));
#endif
}

static u8 mpt_ProStartTest(PADAPTER padapter)
{
	PMPT_CONTEXT pMptCtx = &padapter->mppriv.mpt_ctx;

	pMptCtx->bMassProdTest = _TRUE;
	pMptCtx->is_start_cont_tx = _FALSE;
	pMptCtx->bCckContTx = _FALSE;
	pMptCtx->bOfdmContTx = _FALSE;
	pMptCtx->bSingleCarrier = _FALSE;
	pMptCtx->is_carrier_suppression = _FALSE;
	pMptCtx->is_single_tone = _FALSE;
	pMptCtx->HWTxmode = PACKETS_TX;

	return _SUCCESS;
}

/*
 * General use
 */
s32 SetPowerTracking(PADAPTER padapter, u8 enable)
{

	hal_mpt_SetPowerTracking(padapter, enable);
	return 0;
}

void GetPowerTracking(PADAPTER padapter, u8 *enable)
{
	hal_mpt_GetPowerTracking(padapter, enable);
}

void rtw_mp_trigger_iqk(PADAPTER padapter)
{
	PHY_IQCalibrate(padapter, _FALSE);
}

void rtw_mp_trigger_lck(PADAPTER padapter)
{
	PHY_LCCalibrate(padapter);
}

void rtw_mp_trigger_dpk(PADAPTER padapter)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;

	halrf_dpk_trigger(pDM_Odm);
}

static void init_mp_data(PADAPTER padapter)
{
	u8 v8;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;

	/*disable BCN*/
#ifdef CONFIG_PROTSEL_PORT
	rtw_hal_hw_port_disable(padapter);
#else
	v8 = rtw_read8(padapter, REG_BCN_CTRL);
	v8 &= ~EN_BCN_FUNCTION;
	rtw_write8(padapter, REG_BCN_CTRL, v8);
#endif

	pDM_Odm->rf_calibrate_info.txpowertrack_control = _FALSE;
}

void MPT_PwrCtlDM(PADAPTER padapter, u32 bstart)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;
	u32	rf_ability;

	padapter->mppriv.tssitrk_on = bstart == 3;

	if (bstart == 1) {
		RTW_INFO("in MPT_PwrCtlDM start\n");

		rf_ability = ((u32)halrf_cmn_info_get(pDM_Odm, HALRF_CMNINFO_ABILITY)) | HAL_RF_TX_PWR_TRACK;
		halrf_cmn_info_set(pDM_Odm, HALRF_CMNINFO_ABILITY, rf_ability);
		halrf_set_pwr_track(pDM_Odm, bstart);
		pDM_Odm->rf_calibrate_info.txpowertrack_control = _TRUE;
		padapter->mppriv.mp_dm = 1;

	} else {
		RTW_INFO("in MPT_PwrCtlDM stop\n");
		rf_ability = ((u32)halrf_cmn_info_get(pDM_Odm, HALRF_CMNINFO_ABILITY)) & ~HAL_RF_TX_PWR_TRACK;
		halrf_cmn_info_set(pDM_Odm, HALRF_CMNINFO_ABILITY, rf_ability);
		halrf_set_pwr_track(pDM_Odm, bstart);
		pDM_Odm->rf_calibrate_info.txpowertrack_control = _FALSE;
		if (IS_HARDWARE_TYPE_8822C(padapter))
			padapter->mppriv.mp_dm = 1; /* default enable dpk tracking */
		else
			padapter->mppriv.mp_dm = 0;
		{
			struct txpwrtrack_cfg c;
			u8	chnl = 0 ;
			_rtw_memset(&c, 0, sizeof(struct txpwrtrack_cfg));
			configure_txpower_track(pDM_Odm, &c);
			odm_clear_txpowertracking_state(pDM_Odm);
			if (*c.odm_tx_pwr_track_set_pwr) {
				if (pDM_Odm->support_ic_type == ODM_RTL8188F)
					(*c.odm_tx_pwr_track_set_pwr)(pDM_Odm, MIX_MODE, RF_PATH_A, chnl);
				else if (pDM_Odm->support_ic_type == ODM_RTL8723D) {
					(*c.odm_tx_pwr_track_set_pwr)(pDM_Odm, BBSWING, RF_PATH_A, chnl);
					SetTxPower(padapter);
				} else if (pDM_Odm->support_ic_type == ODM_RTL8192F) {
					(*c.odm_tx_pwr_track_set_pwr)(pDM_Odm, MIX_MODE, RF_PATH_A, chnl);
					(*c.odm_tx_pwr_track_set_pwr)(pDM_Odm, MIX_MODE, RF_PATH_B, chnl);
				} else {
					(*c.odm_tx_pwr_track_set_pwr)(pDM_Odm, BBSWING, RF_PATH_A, chnl);
					(*c.odm_tx_pwr_track_set_pwr)(pDM_Odm, BBSWING, RF_PATH_B, chnl);
				}
			}
		}
	}

}


u32 mp_join(PADAPTER padapter, u8 mode)
{
	WLAN_BSSID_EX bssid;
	struct sta_info *psta;
	u32 length;
	_irqL irqL;
	s32 res = _SUCCESS;

	struct mp_priv *pmppriv = &padapter->mppriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct wlan_network *tgt_network = &pmlmepriv->cur_network;
	struct mlme_ext_priv	*pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX		*pnetwork = (WLAN_BSSID_EX *)(&(pmlmeinfo->network));

	/* 1. initialize a new WLAN_BSSID_EX */
	_rtw_memset(&bssid, 0, sizeof(WLAN_BSSID_EX));
	RTW_INFO("%s ,pmppriv->network_macaddr=%x %x %x %x %x %x\n", __func__,
		pmppriv->network_macaddr[0], pmppriv->network_macaddr[1], pmppriv->network_macaddr[2], pmppriv->network_macaddr[3], pmppriv->network_macaddr[4],
		 pmppriv->network_macaddr[5]);
	_rtw_memcpy(bssid.MacAddress, pmppriv->network_macaddr, ETH_ALEN);

	if (mode == WIFI_FW_ADHOC_STATE) {
		bssid.Ssid.SsidLength = strlen("mp_pseudo_adhoc");
		_rtw_memcpy(bssid.Ssid.Ssid, (u8 *)"mp_pseudo_adhoc", bssid.Ssid.SsidLength);
		bssid.InfrastructureMode = Ndis802_11IBSS;
		bssid.IELength = 0;
		bssid.Configuration.DSConfig = pmppriv->channel;

	} else if (mode == WIFI_FW_STATION_STATE) {
		bssid.Ssid.SsidLength = strlen("mp_pseudo_STATION");
		_rtw_memcpy(bssid.Ssid.Ssid, (u8 *)"mp_pseudo_STATION", bssid.Ssid.SsidLength);
		bssid.InfrastructureMode = Ndis802_11Infrastructure;
		bssid.IELength = 0;
	}

	length = get_WLAN_BSSID_EX_sz(&bssid);
	if (length % 4)
		bssid.Length = ((length >> 2) + 1) << 2; /* round up to multiple of 4 bytes. */
	else
		bssid.Length = length;

	_enter_critical_bh(&pmlmepriv->lock, &irqL);

	if (check_fwstate(pmlmepriv, WIFI_MP_STATE) == _TRUE)
		goto end_of_mp_start_test;

	/* init mp_start_test status */
	if (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE) {
		rtw_disassoc_cmd(padapter, 500, 0);
		rtw_indicate_disconnect(padapter, 0, _FALSE);
		rtw_free_assoc_resources_cmd(padapter, _TRUE, 0);
	}
	pmppriv->prev_fw_state = get_fwstate(pmlmepriv);
	/*pmlmepriv->fw_state = WIFI_MP_STATE;*/
	init_fwstate(pmlmepriv, WIFI_MP_STATE);

	set_fwstate(pmlmepriv, _FW_UNDER_LINKING);

	/* 3 2. create a new psta for mp driver */
	/* clear psta in the cur_network, if any */
	psta = rtw_get_stainfo(&padapter->stapriv, tgt_network->network.MacAddress);
	if (psta)
		rtw_free_stainfo(padapter, psta);

	psta = rtw_alloc_stainfo(&padapter->stapriv, bssid.MacAddress);
	if (psta == NULL) {
		/*pmlmepriv->fw_state = pmppriv->prev_fw_state;*/
		init_fwstate(pmlmepriv, pmppriv->prev_fw_state);
		res = _FAIL;
		goto end_of_mp_start_test;
	}
	if (mode == WIFI_FW_ADHOC_STATE)
	set_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
	else
		set_fwstate(pmlmepriv, WIFI_STATION_STATE);
	/* 3 3. join psudo AdHoc */
	tgt_network->join_res = 1;
	tgt_network->aid = psta->cmn.aid = 1;

	_rtw_memcpy(&padapter->registrypriv.dev_network, &bssid, length);
	rtw_update_registrypriv_dev_network(padapter);
	_rtw_memcpy(&tgt_network->network, &padapter->registrypriv.dev_network, padapter->registrypriv.dev_network.Length);
	_rtw_memcpy(pnetwork, &padapter->registrypriv.dev_network, padapter->registrypriv.dev_network.Length);

	rtw_indicate_connect(padapter);
	_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
	set_fwstate(pmlmepriv, _FW_LINKED);

end_of_mp_start_test:

	_exit_critical_bh(&pmlmepriv->lock, &irqL);

	if (1) { /* (res == _SUCCESS) */
		/* set MSR to WIFI_FW_ADHOC_STATE */
		if (mode == WIFI_FW_ADHOC_STATE) {
			/* set msr to WIFI_FW_ADHOC_STATE */
			pmlmeinfo->state = WIFI_FW_ADHOC_STATE;
			Set_MSR(padapter, (pmlmeinfo->state & 0x3));
			rtw_hal_set_hwreg(padapter, HW_VAR_BSSID, padapter->registrypriv.dev_network.MacAddress);
			rtw_hal_rcr_set_chk_bssid(padapter, MLME_ADHOC_STARTED);
			pmlmeinfo->state |= WIFI_FW_ASSOC_SUCCESS;
		} else {
			Set_MSR(padapter, WIFI_FW_STATION_STATE);

			RTW_INFO("%s , pmppriv->network_macaddr =%x %x %x %x %x %x\n", __func__,
				pmppriv->network_macaddr[0], pmppriv->network_macaddr[1], pmppriv->network_macaddr[2], pmppriv->network_macaddr[3], pmppriv->network_macaddr[4],
				 pmppriv->network_macaddr[5]);

			rtw_hal_set_hwreg(padapter, HW_VAR_BSSID, pmppriv->network_macaddr);
		}
	}

	return res;
}
/* This function initializes the DUT to the MP test mode */
s32 mp_start_test(PADAPTER padapter)
{
	struct mp_priv *pmppriv = &padapter->mppriv;
	s32 res = _SUCCESS;

	padapter->registrypriv.mp_mode = 1;

	init_mp_data(padapter);
#ifdef CONFIG_RTL8703B
	rtl8703b_InitHalDm(padapter);
#endif /* CONFIG_RTL8703B */



	/* 3 0. update mp_priv */
	switch (GET_HAL_RFPATH(padapter)) {
		case RF_1T1R:
			pmppriv->antenna_tx = ANTENNA_A;
			pmppriv->antenna_rx = ANTENNA_A;
			break;
		case RF_1T2R:
		default:
			pmppriv->antenna_tx = ANTENNA_A;
			pmppriv->antenna_rx = ANTENNA_AB;
			break;
		case RF_2T2R:
			pmppriv->antenna_tx = ANTENNA_AB;
			pmppriv->antenna_rx = ANTENNA_AB;
			break;
		case RF_2T4R:
			pmppriv->antenna_tx = ANTENNA_AB;
			pmppriv->antenna_rx = ANTENNA_ABCD;
			break;
	}

	mpt_ProStartTest(padapter);

	mp_join(padapter, WIFI_FW_ADHOC_STATE);

	return res;
}
/* ------------------------------------------------------------------------------
 * This function change the DUT from the MP test mode into normal mode */
void mp_stop_test(PADAPTER padapter)
{
	struct mp_priv *pmppriv = &padapter->mppriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct wlan_network *tgt_network = &pmlmepriv->cur_network;
	struct sta_info *psta;

	_irqL irqL;

	if (pmppriv->mode == MP_ON) {
		pmppriv->bSetTxPower = 0;
		_enter_critical_bh(&pmlmepriv->lock, &irqL);
		if (check_fwstate(pmlmepriv, WIFI_MP_STATE) == _FALSE)
			goto end_of_mp_stop_test;

		/* 3 1. disconnect psudo AdHoc */
		rtw_indicate_disconnect(padapter, 0, _FALSE);

		/* 3 2. clear psta used in mp test mode.
		*	rtw_free_assoc_resources(padapter, _TRUE); */
		psta = rtw_get_stainfo(&padapter->stapriv, tgt_network->network.MacAddress);
		if (psta)
			rtw_free_stainfo(padapter, psta);

		/* 3 3. return to normal state (default:station mode) */
		/*pmlmepriv->fw_state = pmppriv->prev_fw_state; */ /* WIFI_STATION_STATE;*/
		init_fwstate(pmlmepriv, pmppriv->prev_fw_state);

		/* flush the cur_network */
		_rtw_memset(tgt_network, 0, sizeof(struct wlan_network));

		_clr_fwstate_(pmlmepriv, WIFI_MP_STATE);

end_of_mp_stop_test:

		_exit_critical_bh(&pmlmepriv->lock, &irqL);


#ifdef CONFIG_RTL8703B
		rtl8703b_InitHalDm(padapter);
#endif
	}
}
/*---------------------------hal\rtl8192c\MPT_Phy.c---------------------------*/
#if 0
/* #ifdef CONFIG_USB_HCI */
static void mpt_AdjustRFRegByRateByChan92CU(PADAPTER pAdapter, u8 RateIdx, u8 Channel, u8 BandWidthID)
{
	u8		eRFPath;
	u32		rfReg0x26;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);


	if (RateIdx < MPT_RATE_6M) 	/* CCK rate,for 88cu */
		rfReg0x26 = 0xf400;
	else if ((RateIdx >= MPT_RATE_6M) && (RateIdx <= MPT_RATE_54M)) {/* OFDM rate,for 88cu */
		if ((4 == Channel) || (8 == Channel) || (12 == Channel))
			rfReg0x26 = 0xf000;
		else if ((5 == Channel) || (7 == Channel) || (13 == Channel) || (14 == Channel))
			rfReg0x26 = 0xf400;
		else
			rfReg0x26 = 0x4f200;
	} else if ((RateIdx >= MPT_RATE_MCS0) && (RateIdx <= MPT_RATE_MCS15)) {
		/* MCS 20M ,for 88cu */ /* MCS40M rate,for 88cu */

		if (CHANNEL_WIDTH_20 == BandWidthID) {
			if ((4 == Channel) || (8 == Channel))
				rfReg0x26 = 0xf000;
			else if ((5 == Channel) || (7 == Channel) || (13 == Channel) || (14 == Channel))
				rfReg0x26 = 0xf400;
			else
				rfReg0x26 = 0x4f200;
		} else {
			if ((4 == Channel) || (8 == Channel))
				rfReg0x26 = 0xf000;
			else if ((5 == Channel) || (7 == Channel))
				rfReg0x26 = 0xf400;
			else
				rfReg0x26 = 0x4f200;
		}
	}

	for (eRFPath = 0; eRFPath < pHalData->NumTotalRFPath; eRFPath++)
		write_rfreg(pAdapter, eRFPath, RF_SYN_G2, rfReg0x26);
}
#endif
/*-----------------------------------------------------------------------------
 * Function:	mpt_SwitchRfSetting
 *
 * Overview:	Change RF Setting when we siwthc channel/rate/BW for MP.
 *
 * Input:       	PADAPTER				pAdapter
 *
 * Output:      NONE
 *
 * Return:      NONE
 *
 * Revised History:
 * When			Who		Remark
 * 01/08/2009	MHC		Suggestion from SD3 Willis for 92S series.
 * 01/09/2009	MHC		Add CCK modification for 40MHZ. Suggestion from SD3.
 *
 *---------------------------------------------------------------------------*/
#if 0
static void mpt_SwitchRfSetting(PADAPTER pAdapter)
{
	hal_mpt_SwitchRfSetting(pAdapter);
}

/*---------------------------hal\rtl8192c\MPT_Phy.c---------------------------*/
/*---------------------------hal\rtl8192c\MPT_HelperFunc.c---------------------------*/
static void MPT_CCKTxPowerAdjust(PADAPTER Adapter, BOOLEAN bInCH14)
{
	hal_mpt_CCKTxPowerAdjust(Adapter, bInCH14);
}
#endif

/*---------------------------hal\rtl8192c\MPT_HelperFunc.c---------------------------*/

/*
 * SetChannel
 * Description
 *	Use H2C command to change channel,
 *	not only modify rf register, but also other setting need to be done.
 */
void SetChannel(PADAPTER pAdapter)
{
	hal_mpt_SetChannel(pAdapter);
}

/*
 * Notice
 *	Switch bandwitdth may change center frequency(channel)
 */
void SetBandwidth(PADAPTER pAdapter)
{
	hal_mpt_SetBandwidth(pAdapter);

}

void SetAntenna(PADAPTER pAdapter)
{
	hal_mpt_SetAntenna(pAdapter);
}

int SetTxPower(PADAPTER pAdapter)
{

	hal_mpt_SetTxPower(pAdapter);
	return _TRUE;
}

void SetTxAGCOffset(PADAPTER pAdapter, u32 ulTxAGCOffset)
{
	u32 TxAGCOffset_B, TxAGCOffset_C, TxAGCOffset_D, tmpAGC;

	TxAGCOffset_B = (ulTxAGCOffset & 0x000000ff);
	TxAGCOffset_C = ((ulTxAGCOffset & 0x0000ff00) >> 8);
	TxAGCOffset_D = ((ulTxAGCOffset & 0x00ff0000) >> 16);

	tmpAGC = (TxAGCOffset_D << 8 | TxAGCOffset_C << 4 | TxAGCOffset_B);
	write_bbreg(pAdapter, rFPGA0_TxGainStage,
		    (bXBTxAGC | bXCTxAGC | bXDTxAGC), tmpAGC);
}

void SetDataRate(PADAPTER pAdapter)
{
	hal_mpt_SetDataRate(pAdapter);
}

void MP_PHY_SetRFPathSwitch(PADAPTER pAdapter , BOOLEAN bMain)
{

	PHY_SetRFPathSwitch(pAdapter, bMain);

}

void mp_phy_switch_rf_path_set(PADAPTER pAdapter , u8 *pstate)
{

	phy_switch_rf_path_set(pAdapter, pstate);

}

u8 MP_PHY_QueryRFPathSwitch(PADAPTER pAdapter)
{
	return PHY_QueryRFPathSwitch(pAdapter);
}

s32 SetThermalMeter(PADAPTER pAdapter, u8 target_ther)
{
	return hal_mpt_SetThermalMeter(pAdapter, target_ther);
}

#if 0
static void TriggerRFThermalMeter(PADAPTER pAdapter)
{
	hal_mpt_TriggerRFThermalMeter(pAdapter);
}

static u8 ReadRFThermalMeter(PADAPTER pAdapter)
{
	return hal_mpt_ReadRFThermalMeter(pAdapter);
}
#endif

void GetThermalMeter(PADAPTER pAdapter, u8 rfpath ,u8 *value)
{
	hal_mpt_GetThermalMeter(pAdapter, rfpath, value);
}

void SetSingleCarrierTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	hal_mpt_SetSingleCarrierTx(pAdapter, bStart);
}

void SetSingleToneTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	hal_mpt_SetSingleToneTx(pAdapter, bStart);
}

void SetCarrierSuppressionTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	hal_mpt_SetCarrierSuppressionTx(pAdapter, bStart);
}

void SetContinuousTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	hal_mpt_SetContinuousTx(pAdapter, bStart);
}


void PhySetTxPowerLevel(PADAPTER pAdapter)
{
	struct mp_priv *pmp_priv = &pAdapter->mppriv;


	if (pmp_priv->bSetTxPower == 0) /* for NO manually set power index */
		rtw_hal_set_tx_power_level(pAdapter, pmp_priv->channel);
}

/* ------------------------------------------------------------------------------ */
static void dump_mpframe(PADAPTER padapter, struct xmit_frame *pmpframe)
{
	rtw_hal_mgnt_xmit(padapter, pmpframe);
}

static struct xmit_frame *alloc_mp_xmitframe(struct xmit_priv *pxmitpriv)
{
	struct xmit_frame	*pmpframe;
	struct xmit_buf	*pxmitbuf;

	pmpframe = rtw_alloc_xmitframe(pxmitpriv);
	if (pmpframe == NULL)
		return NULL;

	pxmitbuf = rtw_alloc_xmitbuf(pxmitpriv);
	if (pxmitbuf == NULL) {
		rtw_free_xmitframe(pxmitpriv, pmpframe);
		return NULL;
	}

	pmpframe->frame_tag = MP_FRAMETAG;

	pmpframe->pxmitbuf = pxmitbuf;

	pmpframe->buf_addr = pxmitbuf->pbuf;

	pxmitbuf->priv_data = pmpframe;

	return pmpframe;

}


static thread_return mp_xmit_packet_thread(thread_context context)
{
	struct xmit_frame	*pxmitframe;
	struct mp_tx		*pmptx;
	struct mp_priv	*pmp_priv;
	struct xmit_priv	*pxmitpriv;
	PADAPTER padapter;

	pmp_priv = (struct mp_priv *)context;
	pmptx = &pmp_priv->tx;
	padapter = pmp_priv->papdater;
	pxmitpriv = &(padapter->xmitpriv);

	thread_enter("RTW_MP_THREAD");

	RTW_INFO("%s:pkTx Start\n", __func__);
	while (1) {
		pxmitframe = alloc_mp_xmitframe(pxmitpriv);
		if (pxmitframe == NULL) {
			if (pmptx->stop ||
			    RTW_CANNOT_RUN(padapter))
				goto exit;
			else {
				rtw_usleep_os(10);
				continue;
			}
		}
		_rtw_memcpy((u8 *)(pxmitframe->buf_addr + TXDESC_OFFSET), pmptx->buf, pmptx->write_size);
		_rtw_memcpy(&(pxmitframe->attrib), &(pmptx->attrib), sizeof(struct pkt_attrib));


		rtw_usleep_os(padapter->mppriv.pktInterval);
		dump_mpframe(padapter, pxmitframe);

		pmptx->sended++;
		pmp_priv->tx_pktcount++;

		if (pmptx->stop ||
		    RTW_CANNOT_RUN(padapter))
			goto exit;
		if ((pmptx->count != 0) &&
		    (pmptx->count == pmptx->sended))
			goto exit;

		flush_signals_thread();
	}

exit:
	/* RTW_INFO("%s:pkTx Exit\n", __func__); */
	rtw_mfree(pmptx->pallocated_buf, pmptx->buf_size);
	pmptx->pallocated_buf = NULL;
	pmptx->stop = 1;

	thread_exit(NULL);
	return 0;
}

void fill_txdesc_for_mp(PADAPTER padapter, u8 *ptxdesc)
{
	struct mp_priv *pmp_priv = &padapter->mppriv;
	_rtw_memcpy(ptxdesc, pmp_priv->tx.desc, TXDESC_SIZE);
}





#if defined(CONFIG_RTL8703B)
void fill_tx_desc_8703b(PADAPTER padapter)
{
	struct mp_priv *pmp_priv = &padapter->mppriv;
	struct pkt_attrib *pattrib = &(pmp_priv->tx.attrib);
	u8 *ptxdesc = pmp_priv->tx.desc;

	SET_TX_DESC_AGG_BREAK_8703B(ptxdesc, 1);
	SET_TX_DESC_MACID_8703B(ptxdesc, pattrib->mac_id);
	SET_TX_DESC_QUEUE_SEL_8703B(ptxdesc, pattrib->qsel);

	SET_TX_DESC_RATE_ID_8703B(ptxdesc, pattrib->raid);
	SET_TX_DESC_SEQ_8703B(ptxdesc, pattrib->seqnum);
	SET_TX_DESC_HWSEQ_EN_8703B(ptxdesc, 1);
	SET_TX_DESC_USE_RATE_8703B(ptxdesc, 1);
	SET_TX_DESC_DISABLE_FB_8703B(ptxdesc, 1);

	if (pmp_priv->preamble) {
		if (HwRateToMPTRate(pmp_priv->rateidx) <=  MPT_RATE_54M)
			SET_TX_DESC_DATA_SHORT_8703B(ptxdesc, 1);
	}

	if (pmp_priv->bandwidth == CHANNEL_WIDTH_40)
		SET_TX_DESC_DATA_BW_8703B(ptxdesc, 1);

	SET_TX_DESC_TX_RATE_8703B(ptxdesc, pmp_priv->rateidx);

	SET_TX_DESC_DATA_RATE_FB_LIMIT_8703B(ptxdesc, 0x1F);
	SET_TX_DESC_RTS_RATE_FB_LIMIT_8703B(ptxdesc, 0xF);
}
#endif





static void Rtw_MPSetMacTxEDCA(PADAPTER padapter)
{

	rtw_write32(padapter, 0x508 , 0x00a422); /* Disable EDCA BE Txop for MP pkt tx adjust Packet interval */
	/* RTW_INFO("%s:write 0x508~~~~~~ 0x%x\n", __func__,rtw_read32(padapter, 0x508)); */
	phy_set_mac_reg(padapter, 0x458 , bMaskDWord , 0x0);
	/*RTW_INFO("%s()!!!!! 0x460 = 0x%x\n" ,__func__, phy_query_bb_reg(padapter, 0x460, bMaskDWord));*/
	phy_set_mac_reg(padapter, 0x460 , bMaskLWord , 0x0); /* fast EDCA queue packet interval & time out value*/
	/*phy_set_mac_reg(padapter, ODM_EDCA_VO_PARAM ,bMaskLWord , 0x431C);*/
	/*phy_set_mac_reg(padapter, ODM_EDCA_BE_PARAM ,bMaskLWord , 0x431C);*/
	/*phy_set_mac_reg(padapter, ODM_EDCA_BK_PARAM ,bMaskLWord , 0x431C);*/
	RTW_INFO("%s()!!!!! 0x460 = 0x%x\n" , __func__, phy_query_bb_reg(padapter, 0x460, bMaskDWord));

}

void SetPacketTx(PADAPTER padapter)
{
	u8 *ptr, *pkt_start, *pkt_end;
	u32 pkt_size = 0, i = 0, idx = 0, tmp_idx = 0;
	struct rtw_ieee80211_hdr *hdr;
	u8 payload;
	s32 bmcast;
	struct pkt_attrib *pattrib;
	struct mp_priv *pmp_priv;

	pmp_priv = &padapter->mppriv;

	if (pmp_priv->tx.stop)
		return;
	pmp_priv->tx.sended = 0;
	pmp_priv->tx.stop = 0;
	pmp_priv->tx_pktcount = 0;

	/* 3 1. update_attrib() */
	pattrib = &pmp_priv->tx.attrib;
	_rtw_memcpy(pattrib->src, adapter_mac_addr(padapter), ETH_ALEN);
	_rtw_memcpy(pattrib->ta, pattrib->src, ETH_ALEN);
	_rtw_memcpy(pattrib->ra, pattrib->dst, ETH_ALEN);
	bmcast = IS_MCAST(pattrib->ra);
	if (bmcast)
		pattrib->psta = rtw_get_bcmc_stainfo(padapter);
	else
		pattrib->psta = rtw_get_stainfo(&padapter->stapriv, get_bssid(&padapter->mlmepriv));

	if (pattrib->psta == NULL) {
		RTW_INFO("%s:psta = NULL !!\n", __func__);
		return;
	}

	pattrib->mac_id = pattrib->psta->cmn.mac_id;
	pattrib->mbssid = 0;

	pattrib->last_txcmdsz = pattrib->hdrlen + pattrib->pktlen;

	/* 3 2. allocate xmit buffer */
	pkt_size = pattrib->last_txcmdsz;

	if (pmp_priv->tx.pallocated_buf)
		rtw_mfree(pmp_priv->tx.pallocated_buf, pmp_priv->tx.buf_size);
	pmp_priv->tx.write_size = pkt_size;
	pmp_priv->tx.buf_size = pkt_size + XMITBUF_ALIGN_SZ;
	pmp_priv->tx.pallocated_buf = rtw_zmalloc(pmp_priv->tx.buf_size);
	if (pmp_priv->tx.pallocated_buf == NULL) {
		RTW_INFO("%s: malloc(%d) fail!!\n", __func__, pmp_priv->tx.buf_size);
		return;
	}
	pmp_priv->tx.buf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(pmp_priv->tx.pallocated_buf), XMITBUF_ALIGN_SZ);
	ptr = pmp_priv->tx.buf;

	_rtw_memset(pmp_priv->tx.desc, 0, TXDESC_SIZE);
	pkt_start = ptr;
	pkt_end = pkt_start + pkt_size;

	/* 3 3. init TX descriptor */


	if (IS_HARDWARE_TYPE_8822B(padapter))
		rtl8822b_prepare_mp_txdesc(padapter, pmp_priv);




#if defined(CONFIG_RTL8703B)
	if (IS_HARDWARE_TYPE_8703B(padapter))
		fill_tx_desc_8703b(padapter);
#endif






	/* 3 4. make wlan header, make_wlanhdr() */
	hdr = (struct rtw_ieee80211_hdr *)pkt_start;
	set_frame_sub_type(&hdr->frame_ctl, pattrib->subtype);

	_rtw_memcpy(hdr->addr1, pattrib->dst, ETH_ALEN); /* DA */
	_rtw_memcpy(hdr->addr2, pattrib->src, ETH_ALEN); /* SA */
	_rtw_memcpy(hdr->addr3, get_bssid(&padapter->mlmepriv), ETH_ALEN); /* RA, BSSID */

	/* 3 5. make payload */
	ptr = pkt_start + pattrib->hdrlen;

	if (pmp_priv->mplink_btx == _TRUE) {
		_rtw_memcpy(ptr, pmp_priv->mplink_buf, pkt_end - ptr);
	} else {
		switch (pmp_priv->tx.payload) {
		case MP_TX_Payload_00:
			RTW_INFO("MP packet tx 0x00 payload!\n");
			payload = 0x00;
			_rtw_memset(ptr, 0x00, pkt_end - ptr);
			break;
		case MP_TX_Payload_5a:
			RTW_INFO("MP packet tx 0x5a payload!\n");
			payload = 0x5a;
			_rtw_memset(ptr, 0x5a, pkt_end - ptr);
			break;
		case MP_TX_Payload_a5:
			RTW_INFO("MP packet tx 0xa5 payload!\n");
			payload = 0xa5;
			_rtw_memset(ptr, 0xa5, pkt_end - ptr);
			break;
		case MP_TX_Payload_ff:
			RTW_INFO("MP packet tx 0xff payload!\n");
			payload = 0xff;
			_rtw_memset(ptr, 0xff, pkt_end - ptr);
			break;
		case MP_TX_Payload_prbs9:
			RTW_INFO("MP packet tx PRBS9 payload!\n");
			while (idx <= pkt_end - ptr) {
				int start = 0x02;
				int a = start;

				for (i = 0;; i++) {
						int newbit = (((a >> 8) ^ (a >> 4)) & 1);
						a = ((a << 1) | newbit) & 0x1ff;
						RTW_DBG("%x ", a);
						ptr[idx + i] = a;

						if (a == start) {
							RTW_INFO("payload repetition period is %d , end %d\n", i , idx);
							tmp_idx += i;
							break;
						}
						if (idx + i >= (pkt_end - ptr)) {
							tmp_idx += (idx + i);
							RTW_INFO(" repetition period payload end curr ptr %d\n", idx + i);
							break;
						}
				}
				idx = tmp_idx;
			}
			break;
		case MP_TX_Payload_default_random:
			RTW_INFO("MP packet tx default random payload!\n");
			for (i = 0; i < pkt_end - ptr; i++)
				ptr[i] = rtw_random32() % 0xFF;
			break;
		default:
			RTW_INFO("Config payload type default use 0x%x\n!", pmp_priv->tx.payload);
			_rtw_memset(ptr, pmp_priv->tx.payload, pkt_end - ptr);
			break;
		}
	}
	/* 3 6. start thread */
	pmp_priv->tx.PktTxThread = kthread_run(mp_xmit_packet_thread, pmp_priv, "RTW_MP_THREAD");
	if (IS_ERR(pmp_priv->tx.PktTxThread)) {
		RTW_ERR("Create PktTx Thread Fail !!!!!\n");
		pmp_priv->tx.PktTxThread = NULL;
	}

	Rtw_MPSetMacTxEDCA(padapter);
	return;
}

void SetPacketRx(PADAPTER pAdapter, u8 bStartRx, u8 bAB)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(pAdapter);
	struct mp_priv *pmppriv = &pAdapter->mppriv;


	if (bStartRx) {
		pHalData->ReceiveConfig = RCR_AAP | RCR_APM | RCR_AM | RCR_AMF | RCR_HTC_LOC_CTRL;
		pHalData->ReceiveConfig |= RCR_ACRC32;
		pHalData->ReceiveConfig |= RCR_APP_PHYST_RXFF | RCR_APP_ICV | RCR_APP_MIC;

		if (pmppriv->bSetRxBssid == _TRUE) {
			RTW_INFO("%s: pmppriv->network_macaddr=" MAC_FMT "\n", __func__,
				 MAC_ARG(pmppriv->network_macaddr));
			pHalData->ReceiveConfig = 0;
			pHalData->ReceiveConfig |= RCR_CBSSID_DATA | RCR_CBSSID_BCN |RCR_APM | RCR_AM | RCR_AB |RCR_AMF;
			pHalData->ReceiveConfig |= RCR_APP_PHYST_RXFF;

			write_bbreg(pAdapter, 0x550, BIT3, bEnable);
			rtw_write16(pAdapter, REG_RXFLTMAP0, 0xFFEF); /* REG_RXFLTMAP0 (RX Filter Map Group 0) */
			pmppriv->brx_filter_beacon = _TRUE;

		} else {
			pHalData->ReceiveConfig |= RCR_ADF;
			/* Accept all data frames */
			rtw_write16(pAdapter, REG_RXFLTMAP2, 0xFFFF);
		}

		if (bAB)
			pHalData->ReceiveConfig |= RCR_AB;
	} else {
		pHalData->ReceiveConfig = 0;
		rtw_write16(pAdapter, REG_RXFLTMAP0, 0xFFFF); /* REG_RXFLTMAP0 (RX Filter Map Group 0) */
	}

	rtw_write32(pAdapter, REG_RCR, pHalData->ReceiveConfig);
}

void ResetPhyRxPktCount(PADAPTER pAdapter)
{
	u32 i, phyrx_set = 0;

	for (i = 0; i <= 0xF; i++) {
		phyrx_set = 0;
		phyrx_set |= _RXERR_RPT_SEL(i);	/* select */
		phyrx_set |= RXERR_RPT_RST;	/* set counter to zero */
		rtw_write32(pAdapter, REG_RXERR_RPT, phyrx_set);
	}
}

static u32 GetPhyRxPktCounts(PADAPTER pAdapter, u32 selbit)
{
	/* selection */
	u32 phyrx_set = 0, count = 0;

	phyrx_set = _RXERR_RPT_SEL(selbit & 0xF);
	rtw_write32(pAdapter, REG_RXERR_RPT, phyrx_set);

	/* Read packet count */
	count = rtw_read32(pAdapter, REG_RXERR_RPT) & RXERR_COUNTER_MASK;

	return count;
}

u32 GetPhyRxPktReceived(PADAPTER pAdapter)
{
	u32 OFDM_cnt = 0, CCK_cnt = 0, HT_cnt = 0;

	OFDM_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_OFDM_MPDU_OK);
	CCK_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_CCK_MPDU_OK);
	HT_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_HT_MPDU_OK);

	return OFDM_cnt + CCK_cnt + HT_cnt;
}

u32 GetPhyRxPktCRC32Error(PADAPTER pAdapter)
{
	u32 OFDM_cnt = 0, CCK_cnt = 0, HT_cnt = 0;

	OFDM_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_OFDM_MPDU_FAIL);
	CCK_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_CCK_MPDU_FAIL);
	HT_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_HT_MPDU_FAIL);

	return OFDM_cnt + CCK_cnt + HT_cnt;
}

struct psd_init_regs {
	/* 3 wire */
	int reg_88c;
	int reg_c00;
	int reg_e00;
	int reg_1800;
	int reg_1a00;
	/* cck */
	int reg_800;
	int reg_808;
};

static int rtw_mp_psd_init(PADAPTER padapter, struct psd_init_regs *regs)
{
	HAL_DATA_TYPE	*phal_data	= GET_HAL_DATA(padapter);

	switch (phal_data->rf_type) {
	/* 1R */
	case RF_1T1R:
		if (hal_chk_proto_cap(padapter, PROTO_CAP_11AC)) {
			/* 11AC 1R PSD Setting 3wire & cck off */
			regs->reg_c00 = rtw_read32(padapter, 0xC00);
			phy_set_bb_reg(padapter, 0xC00, 0x3, 0x00);
			regs->reg_808 = rtw_read32(padapter, 0x808);
			phy_set_bb_reg(padapter, 0x808, 0x10000000, 0x0);
		} else {
			/* 11N 3-wire off 1 */
			regs->reg_88c = rtw_read32(padapter, 0x88C);
			phy_set_bb_reg(padapter, 0x88C, 0x300000, 0x3);
			/* 11N CCK off */
			regs->reg_800 = rtw_read32(padapter, 0x800);
			phy_set_bb_reg(padapter, 0x800, 0x1000000, 0x0);
		}
	break;

	/* 2R */
	case RF_1T2R:
	case RF_2T2R:
		if (hal_chk_proto_cap(padapter, PROTO_CAP_11AC)) {
			/* 11AC 2R PSD Setting 3wire & cck off */
			regs->reg_c00 = rtw_read32(padapter, 0xC00);
			regs->reg_e00 = rtw_read32(padapter, 0xE00);
			phy_set_bb_reg(padapter, 0xC00, 0x3, 0x00);
			phy_set_bb_reg(padapter, 0xE00, 0x3, 0x00);
			regs->reg_808 = rtw_read32(padapter, 0x808);
			phy_set_bb_reg(padapter, 0x808, 0x10000000, 0x0);
		} else {
			/* 11N 3-wire off 2 */
			regs->reg_88c = rtw_read32(padapter, 0x88C);
			phy_set_bb_reg(padapter, 0x88C, 0xF00000, 0xF);
			/* 11N CCK off */
			regs->reg_800 = rtw_read32(padapter, 0x800);
			phy_set_bb_reg(padapter, 0x800, 0x1000000, 0x0);
		}
	break;

	/* 3R */
	case RF_2T3R:
	case RF_3T3R:
		if (hal_chk_proto_cap(padapter, PROTO_CAP_11AC)) {
			/* 11AC 3R PSD Setting 3wire & cck off */
			regs->reg_c00 = rtw_read32(padapter, 0xC00);
			regs->reg_e00 = rtw_read32(padapter, 0xE00);
			regs->reg_1800 = rtw_read32(padapter, 0x1800);
			phy_set_bb_reg(padapter, 0xC00, 0x3, 0x00);
			phy_set_bb_reg(padapter, 0xE00, 0x3, 0x00);
			phy_set_bb_reg(padapter, 0x1800, 0x3, 0x00);
			regs->reg_808 = rtw_read32(padapter, 0x808);
			phy_set_bb_reg(padapter, 0x808, 0x10000000, 0x0);
		} else {
			RTW_ERR("%s: 11n don't support 3R\n", __func__);
			return -1;
		}
		break;

	/* 4R */
	case RF_2T4R:
	case RF_3T4R:
	case RF_4T4R:
		if (hal_chk_proto_cap(padapter, PROTO_CAP_11AC)) {
			/* 11AC 4R PSD Setting 3wire & cck off */
			regs->reg_c00 = rtw_read32(padapter, 0xC00);
			regs->reg_e00 = rtw_read32(padapter, 0xE00);
			regs->reg_1800 = rtw_read32(padapter, 0x1800);
			regs->reg_1a00 = rtw_read32(padapter, 0x1A00);
			phy_set_bb_reg(padapter, 0xC00, 0x3, 0x00);
			phy_set_bb_reg(padapter, 0xE00, 0x3, 0x00);
			phy_set_bb_reg(padapter, 0x1800, 0x3, 0x00);
			phy_set_bb_reg(padapter, 0x1A00, 0x3, 0x00);
			regs->reg_808 = rtw_read32(padapter, 0x808);
			phy_set_bb_reg(padapter, 0x808, 0x10000000, 0x0);
		} else {
			RTW_ERR("%s: 11n don't support 4R\n", __func__);
			return -1;
		}
		break;

	default:
		RTW_ERR("%s: unknown %d rf type\n", __func__, phal_data->rf_type);
		return -1;
	}

	/* Set PSD points, 0=128, 1=256, 2=512, 3=1024 */
	if (hal_chk_proto_cap(padapter, PROTO_CAP_11AC))
		phy_set_bb_reg(padapter, 0x910, 0xC000, 3);
	else
		phy_set_bb_reg(padapter, 0x808, 0xC000, 3);

	RTW_INFO("%s: set %d rf type done\n", __func__, phal_data->rf_type);
	return 0;
}

static int rtw_mp_psd_close(PADAPTER padapter, struct psd_init_regs *regs)
{
	HAL_DATA_TYPE	*phal_data	= GET_HAL_DATA(padapter);


	if (!hal_chk_proto_cap(padapter, PROTO_CAP_11AC)) {
		/* 11n 3wire restore */
		rtw_write32(padapter, 0x88C, regs->reg_88c);
		/* 11n cck restore */
		rtw_write32(padapter, 0x800, regs->reg_800);
		RTW_INFO("%s: restore %d rf type\n", __func__, phal_data->rf_type);
		return 0;
	}

	/* 11ac 3wire restore */
	switch (phal_data->rf_type) {
	case RF_1T1R:
		rtw_write32(padapter, 0xC00, regs->reg_c00);
		break;
	case RF_1T2R:
	case RF_2T2R:
		rtw_write32(padapter, 0xC00, regs->reg_c00);
		rtw_write32(padapter, 0xE00, regs->reg_e00);
		break;
	case RF_2T3R:
	case RF_3T3R:
		rtw_write32(padapter, 0xC00, regs->reg_c00);
		rtw_write32(padapter, 0xE00, regs->reg_e00);
		rtw_write32(padapter, 0x1800, regs->reg_1800);
		break;
	case RF_2T4R:
	case RF_3T4R:
	case RF_4T4R:
		rtw_write32(padapter, 0xC00, regs->reg_c00);
		rtw_write32(padapter, 0xE00, regs->reg_e00);
		rtw_write32(padapter, 0x1800, regs->reg_1800);
		rtw_write32(padapter, 0x1A00, regs->reg_1a00);
		break;
	default:
		RTW_WARN("%s: unknown %d rf type\n", __func__, phal_data->rf_type);
		break;
	}

	/* 11ac cck restore */
	rtw_write32(padapter, 0x808, regs->reg_808);
	RTW_INFO("%s: restore %d rf type done\n", __func__, phal_data->rf_type);
	return 0;
}

/* reg 0x808[9:0]: FFT data x
 * reg 0x808[22]:  0  -->  1  to get 1 FFT data y
 * reg 0x8B4[15:0]: FFT data y report */
static u32 rtw_GetPSDData(PADAPTER pAdapter, u32 point)
{
	u32 psd_val = 0;

	u16 psd_reg = 0x910;
	u16 psd_regL = 0xF44;

	psd_val = rtw_read32(pAdapter, psd_reg);

	psd_val &= 0xFFBFFC00;
	psd_val |= point;

	rtw_write32(pAdapter, psd_reg, psd_val);
	rtw_mdelay_os(1);
	psd_val |= 0x00400000;

	rtw_write32(pAdapter, psd_reg, psd_val);
	rtw_mdelay_os(1);

	psd_val = rtw_read32(pAdapter, psd_regL);
	psd_val &= 0x0000FFFF;

	return psd_val;
}

/*
 * pts	start_point_min		stop_point_max
 * 128	64			64 + 128 = 192
 * 256	128			128 + 256 = 384
 * 512	256			256 + 512 = 768
 * 1024	512			512 + 1024 = 1536
 *
 */
u32 mp_query_psd(PADAPTER pAdapter, u8 *data)
{
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(pAdapter);
	struct dm_struct *p_dm = adapter_to_phydm(pAdapter);

	u32 i, psd_pts = 0, psd_start = 0, psd_stop = 0;
	u32 psd_data = 0;
	struct psd_init_regs regs = {};
	int psd_analysis = 0;
	char *pdata = NULL;


	if (!netif_running(pAdapter->pnetdev)) {
		return 0;
	}

	if (check_fwstate(&pAdapter->mlmepriv, WIFI_MP_STATE) == _FALSE) {
		return 0;
	}

	if (strlen(data) == 0) { /* default value */
		psd_pts = 128;
		psd_start = 64;
		psd_stop = 128;
	} else if (strncmp(data, "analysis,", 9) == 0) {
		if (rtw_mp_psd_init(pAdapter, &regs) != 0)
			return 0;
		psd_analysis = 1;
		sscanf(data + 9, "pts=%d,start=%d,stop=%d", &psd_pts, &psd_start, &psd_stop);
	} else
		sscanf(data, "pts=%d,start=%d,stop=%d", &psd_pts, &psd_start, &psd_stop);

	data[0] = '\0';
	pdata = data;

	if (psd_stop > 1536 || psd_stop < 1) {
		rtw_warn_on(1);
		psd_stop = 1536;
	}

	if (IS_HARDWARE_TYPE_8822C(pAdapter)) {
			u32 *psdbuf = rtw_zmalloc(sizeof(u32)*256);

			if (psdbuf == NULL) {
				RTW_INFO("%s: psd buf malloc fail!!\n", __func__);
				return 0;
			}

			halrf_cmn_info_set(p_dm, HALRF_CMNINFO_MP_PSD_POINT, psd_pts);
			halrf_cmn_info_set(p_dm, HALRF_CMNINFO_MP_PSD_START_POINT, psd_start);
			halrf_cmn_info_set(p_dm, HALRF_CMNINFO_MP_PSD_STOP_POINT, psd_stop);
			halrf_cmn_info_set(p_dm, HALRF_CMNINFO_MP_PSD_AVERAGE, 0x20000);

			halrf_psd_init(p_dm);
		rtw_msleep_os(100);
			halrf_psd_query(p_dm, psdbuf, 256);

			i = 0;
			while (i < 256) {
				pdata += sprintf(pdata, "%x ", (psdbuf[i]));
				i++;
			}

		if (psdbuf)
			rtw_mfree(psdbuf, sizeof(u32)*256);

	} else {
			i = psd_start;

			while (i < psd_stop) {
				if (i >= psd_pts)
					psd_data = rtw_GetPSDData(pAdapter, i - psd_pts);
				else
					psd_data = rtw_GetPSDData(pAdapter, i);

				pdata += sprintf(pdata, "%x ", psd_data);
				i++;
			}

	}

	rtw_msleep_os(100);

	if (psd_analysis)
		rtw_mp_psd_close(pAdapter, &regs);

	return strlen(data) + 1;
}


#if 0
void _rtw_mp_xmit_priv(struct xmit_priv *pxmitpriv)
{
	int i, res;
	_adapter *padapter = pxmitpriv->adapter;
	struct xmit_frame	*pxmitframe = (struct xmit_frame *) pxmitpriv->pxmit_frame_buf;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmitbuf;

	u32 max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
	u32 num_xmit_extbuf = NR_XMIT_EXTBUFF;
	if (padapter->registrypriv.mp_mode == 0) {
		max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
		num_xmit_extbuf = NR_XMIT_EXTBUFF;
	} else {
		max_xmit_extbuf_size = 6000;
		num_xmit_extbuf = 8;
	}

	pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmit_extbuf;
	for (i = 0; i < num_xmit_extbuf; i++) {
		rtw_os_xmit_resource_free(padapter, pxmitbuf, (max_xmit_extbuf_size + XMITBUF_ALIGN_SZ), _FALSE);

		pxmitbuf++;
	}

	if (pxmitpriv->pallocated_xmit_extbuf)
		rtw_vmfree(pxmitpriv->pallocated_xmit_extbuf, num_xmit_extbuf * sizeof(struct xmit_buf) + 4);

	if (padapter->registrypriv.mp_mode == 0) {
		max_xmit_extbuf_size = 6000;
		num_xmit_extbuf = 8;
	} else {
		max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
		num_xmit_extbuf = NR_XMIT_EXTBUFF;
	}

	/* Init xmit extension buff */
	_rtw_init_queue(&pxmitpriv->free_xmit_extbuf_queue);

	pxmitpriv->pallocated_xmit_extbuf = rtw_zvmalloc(num_xmit_extbuf * sizeof(struct xmit_buf) + 4);

	if (pxmitpriv->pallocated_xmit_extbuf  == NULL) {
		res = _FAIL;
		goto exit;
	}

	pxmitpriv->pxmit_extbuf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(pxmitpriv->pallocated_xmit_extbuf), 4);

	pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmit_extbuf;

	for (i = 0; i < num_xmit_extbuf; i++) {
		_rtw_init_listhead(&pxmitbuf->list);

		pxmitbuf->priv_data = NULL;
		pxmitbuf->padapter = padapter;
		pxmitbuf->buf_tag = XMITBUF_MGNT;

		res = rtw_os_xmit_resource_alloc(padapter, pxmitbuf, max_xmit_extbuf_size + XMITBUF_ALIGN_SZ, _TRUE);
		if (res == _FAIL) {
			res = _FAIL;
			goto exit;
		}


		rtw_list_insert_tail(&pxmitbuf->list, &(pxmitpriv->free_xmit_extbuf_queue.queue));
		pxmitbuf++;

	}

	pxmitpriv->free_xmit_extbuf_cnt = num_xmit_extbuf;

exit:
	;
}
#endif

u8
mpt_to_mgnt_rate(
		u32	MptRateIdx
)
{
	/* Mapped to MGN_XXX defined in MgntGen.h */
	switch (MptRateIdx) {
	/* CCK rate. */
	case	MPT_RATE_1M:
		return MGN_1M;
	case	MPT_RATE_2M:
		return MGN_2M;
	case	MPT_RATE_55M:
		return MGN_5_5M;
	case	MPT_RATE_11M:
		return MGN_11M;

	/* OFDM rate. */
	case	MPT_RATE_6M:
		return MGN_6M;
	case	MPT_RATE_9M:
		return MGN_9M;
	case	MPT_RATE_12M:
		return MGN_12M;
	case	MPT_RATE_18M:
		return MGN_18M;
	case	MPT_RATE_24M:
		return MGN_24M;
	case	MPT_RATE_36M:
		return MGN_36M;
	case	MPT_RATE_48M:
		return MGN_48M;
	case	MPT_RATE_54M:
		return MGN_54M;

	/* HT rate. */
	case	MPT_RATE_MCS0:
		return MGN_MCS0;
	case	MPT_RATE_MCS1:
		return MGN_MCS1;
	case	MPT_RATE_MCS2:
		return MGN_MCS2;
	case	MPT_RATE_MCS3:
		return MGN_MCS3;
	case	MPT_RATE_MCS4:
		return MGN_MCS4;
	case	MPT_RATE_MCS5:
		return MGN_MCS5;
	case	MPT_RATE_MCS6:
		return MGN_MCS6;
	case	MPT_RATE_MCS7:
		return MGN_MCS7;
	case	MPT_RATE_MCS8:
		return MGN_MCS8;
	case	MPT_RATE_MCS9:
		return MGN_MCS9;
	case	MPT_RATE_MCS10:
		return MGN_MCS10;
	case	MPT_RATE_MCS11:
		return MGN_MCS11;
	case	MPT_RATE_MCS12:
		return MGN_MCS12;
	case	MPT_RATE_MCS13:
		return MGN_MCS13;
	case	MPT_RATE_MCS14:
		return MGN_MCS14;
	case	MPT_RATE_MCS15:
		return MGN_MCS15;
	case	MPT_RATE_MCS16:
		return MGN_MCS16;
	case	MPT_RATE_MCS17:
		return MGN_MCS17;
	case	MPT_RATE_MCS18:
		return MGN_MCS18;
	case	MPT_RATE_MCS19:
		return MGN_MCS19;
	case	MPT_RATE_MCS20:
		return MGN_MCS20;
	case	MPT_RATE_MCS21:
		return MGN_MCS21;
	case	MPT_RATE_MCS22:
		return MGN_MCS22;
	case	MPT_RATE_MCS23:
		return MGN_MCS23;
	case	MPT_RATE_MCS24:
		return MGN_MCS24;
	case	MPT_RATE_MCS25:
		return MGN_MCS25;
	case	MPT_RATE_MCS26:
		return MGN_MCS26;
	case	MPT_RATE_MCS27:
		return MGN_MCS27;
	case	MPT_RATE_MCS28:
		return MGN_MCS28;
	case	MPT_RATE_MCS29:
		return MGN_MCS29;
	case	MPT_RATE_MCS30:
		return MGN_MCS30;
	case	MPT_RATE_MCS31:
		return MGN_MCS31;

	/* VHT rate. */
	case	MPT_RATE_VHT1SS_MCS0:
		return MGN_VHT1SS_MCS0;
	case	MPT_RATE_VHT1SS_MCS1:
		return MGN_VHT1SS_MCS1;
	case	MPT_RATE_VHT1SS_MCS2:
		return MGN_VHT1SS_MCS2;
	case	MPT_RATE_VHT1SS_MCS3:
		return MGN_VHT1SS_MCS3;
	case	MPT_RATE_VHT1SS_MCS4:
		return MGN_VHT1SS_MCS4;
	case	MPT_RATE_VHT1SS_MCS5:
		return MGN_VHT1SS_MCS5;
	case	MPT_RATE_VHT1SS_MCS6:
		return MGN_VHT1SS_MCS6;
	case	MPT_RATE_VHT1SS_MCS7:
		return MGN_VHT1SS_MCS7;
	case	MPT_RATE_VHT1SS_MCS8:
		return MGN_VHT1SS_MCS8;
	case	MPT_RATE_VHT1SS_MCS9:
		return MGN_VHT1SS_MCS9;
	case	MPT_RATE_VHT2SS_MCS0:
		return MGN_VHT2SS_MCS0;
	case	MPT_RATE_VHT2SS_MCS1:
		return MGN_VHT2SS_MCS1;
	case	MPT_RATE_VHT2SS_MCS2:
		return MGN_VHT2SS_MCS2;
	case	MPT_RATE_VHT2SS_MCS3:
		return MGN_VHT2SS_MCS3;
	case	MPT_RATE_VHT2SS_MCS4:
		return MGN_VHT2SS_MCS4;
	case	MPT_RATE_VHT2SS_MCS5:
		return MGN_VHT2SS_MCS5;
	case	MPT_RATE_VHT2SS_MCS6:
		return MGN_VHT2SS_MCS6;
	case	MPT_RATE_VHT2SS_MCS7:
		return MGN_VHT2SS_MCS7;
	case	MPT_RATE_VHT2SS_MCS8:
		return MGN_VHT2SS_MCS8;
	case	MPT_RATE_VHT2SS_MCS9:
		return MGN_VHT2SS_MCS9;
	case	MPT_RATE_VHT3SS_MCS0:
		return MGN_VHT3SS_MCS0;
	case	MPT_RATE_VHT3SS_MCS1:
		return MGN_VHT3SS_MCS1;
	case	MPT_RATE_VHT3SS_MCS2:
		return MGN_VHT3SS_MCS2;
	case	MPT_RATE_VHT3SS_MCS3:
		return MGN_VHT3SS_MCS3;
	case	MPT_RATE_VHT3SS_MCS4:
		return MGN_VHT3SS_MCS4;
	case	MPT_RATE_VHT3SS_MCS5:
		return MGN_VHT3SS_MCS5;
	case	MPT_RATE_VHT3SS_MCS6:
		return MGN_VHT3SS_MCS6;
	case	MPT_RATE_VHT3SS_MCS7:
		return MGN_VHT3SS_MCS7;
	case	MPT_RATE_VHT3SS_MCS8:
		return MGN_VHT3SS_MCS8;
	case	MPT_RATE_VHT3SS_MCS9:
		return MGN_VHT3SS_MCS9;
	case	MPT_RATE_VHT4SS_MCS0:
		return MGN_VHT4SS_MCS0;
	case	MPT_RATE_VHT4SS_MCS1:
		return MGN_VHT4SS_MCS1;
	case	MPT_RATE_VHT4SS_MCS2:
		return MGN_VHT4SS_MCS2;
	case	MPT_RATE_VHT4SS_MCS3:
		return MGN_VHT4SS_MCS3;
	case	MPT_RATE_VHT4SS_MCS4:
		return MGN_VHT4SS_MCS4;
	case	MPT_RATE_VHT4SS_MCS5:
		return MGN_VHT4SS_MCS5;
	case	MPT_RATE_VHT4SS_MCS6:
		return MGN_VHT4SS_MCS6;
	case	MPT_RATE_VHT4SS_MCS7:
		return MGN_VHT4SS_MCS7;
	case	MPT_RATE_VHT4SS_MCS8:
		return MGN_VHT4SS_MCS8;
	case	MPT_RATE_VHT4SS_MCS9:
		return MGN_VHT4SS_MCS9;

	case	MPT_RATE_LAST:	/* fully automatiMGN_VHT2SS_MCS1;	 */
	default:
		RTW_INFO("<===mpt_to_mgnt_rate(), Invalid Rate: %d!!\n", MptRateIdx);
		return 0x0;
	}
}


u8 HwRateToMPTRate(u8 rate)
{
	u8	ret_rate = MGN_1M;

	switch (rate) {
	case DESC_RATE1M:
		ret_rate = MPT_RATE_1M;
		break;
	case DESC_RATE2M:
		ret_rate = MPT_RATE_2M;
		break;
	case DESC_RATE5_5M:
		ret_rate = MPT_RATE_55M;
		break;
	case DESC_RATE11M:
		ret_rate = MPT_RATE_11M;
		break;
	case DESC_RATE6M:
		ret_rate = MPT_RATE_6M;
		break;
	case DESC_RATE9M:
		ret_rate = MPT_RATE_9M;
		break;
	case DESC_RATE12M:
		ret_rate = MPT_RATE_12M;
		break;
	case DESC_RATE18M:
		ret_rate = MPT_RATE_18M;
		break;
	case DESC_RATE24M:
		ret_rate = MPT_RATE_24M;
		break;
	case DESC_RATE36M:
		ret_rate = MPT_RATE_36M;
		break;
	case DESC_RATE48M:
		ret_rate = MPT_RATE_48M;
		break;
	case DESC_RATE54M:
		ret_rate = MPT_RATE_54M;
		break;
	case DESC_RATEMCS0:
		ret_rate = MPT_RATE_MCS0;
		break;
	case DESC_RATEMCS1:
		ret_rate = MPT_RATE_MCS1;
		break;
	case DESC_RATEMCS2:
		ret_rate = MPT_RATE_MCS2;
		break;
	case DESC_RATEMCS3:
		ret_rate = MPT_RATE_MCS3;
		break;
	case DESC_RATEMCS4:
		ret_rate = MPT_RATE_MCS4;
		break;
	case DESC_RATEMCS5:
		ret_rate = MPT_RATE_MCS5;
		break;
	case DESC_RATEMCS6:
		ret_rate = MPT_RATE_MCS6;
		break;
	case DESC_RATEMCS7:
		ret_rate = MPT_RATE_MCS7;
		break;
	case DESC_RATEMCS8:
		ret_rate = MPT_RATE_MCS8;
		break;
	case DESC_RATEMCS9:
		ret_rate = MPT_RATE_MCS9;
		break;
	case DESC_RATEMCS10:
		ret_rate = MPT_RATE_MCS10;
		break;
	case DESC_RATEMCS11:
		ret_rate = MPT_RATE_MCS11;
		break;
	case DESC_RATEMCS12:
		ret_rate = MPT_RATE_MCS12;
		break;
	case DESC_RATEMCS13:
		ret_rate = MPT_RATE_MCS13;
		break;
	case DESC_RATEMCS14:
		ret_rate = MPT_RATE_MCS14;
		break;
	case DESC_RATEMCS15:
		ret_rate = MPT_RATE_MCS15;
		break;
	case DESC_RATEMCS16:
		ret_rate = MPT_RATE_MCS16;
		break;
	case DESC_RATEMCS17:
		ret_rate = MPT_RATE_MCS17;
		break;
	case DESC_RATEMCS18:
		ret_rate = MPT_RATE_MCS18;
		break;
	case DESC_RATEMCS19:
		ret_rate = MPT_RATE_MCS19;
		break;
	case DESC_RATEMCS20:
		ret_rate = MPT_RATE_MCS20;
		break;
	case DESC_RATEMCS21:
		ret_rate = MPT_RATE_MCS21;
		break;
	case DESC_RATEMCS22:
		ret_rate = MPT_RATE_MCS22;
		break;
	case DESC_RATEMCS23:
		ret_rate = MPT_RATE_MCS23;
		break;
	case DESC_RATEMCS24:
		ret_rate = MPT_RATE_MCS24;
		break;
	case DESC_RATEMCS25:
		ret_rate = MPT_RATE_MCS25;
		break;
	case DESC_RATEMCS26:
		ret_rate = MPT_RATE_MCS26;
		break;
	case DESC_RATEMCS27:
		ret_rate = MPT_RATE_MCS27;
		break;
	case DESC_RATEMCS28:
		ret_rate = MPT_RATE_MCS28;
		break;
	case DESC_RATEMCS29:
		ret_rate = MPT_RATE_MCS29;
		break;
	case DESC_RATEMCS30:
		ret_rate = MPT_RATE_MCS30;
		break;
	case DESC_RATEMCS31:
		ret_rate = MPT_RATE_MCS31;
		break;
	case DESC_RATEVHTSS1MCS0:
		ret_rate = MPT_RATE_VHT1SS_MCS0;
		break;
	case DESC_RATEVHTSS1MCS1:
		ret_rate = MPT_RATE_VHT1SS_MCS1;
		break;
	case DESC_RATEVHTSS1MCS2:
		ret_rate = MPT_RATE_VHT1SS_MCS2;
		break;
	case DESC_RATEVHTSS1MCS3:
		ret_rate = MPT_RATE_VHT1SS_MCS3;
		break;
	case DESC_RATEVHTSS1MCS4:
		ret_rate = MPT_RATE_VHT1SS_MCS4;
		break;
	case DESC_RATEVHTSS1MCS5:
		ret_rate = MPT_RATE_VHT1SS_MCS5;
		break;
	case DESC_RATEVHTSS1MCS6:
		ret_rate = MPT_RATE_VHT1SS_MCS6;
		break;
	case DESC_RATEVHTSS1MCS7:
		ret_rate = MPT_RATE_VHT1SS_MCS7;
		break;
	case DESC_RATEVHTSS1MCS8:
		ret_rate = MPT_RATE_VHT1SS_MCS8;
		break;
	case DESC_RATEVHTSS1MCS9:
		ret_rate = MPT_RATE_VHT1SS_MCS9;
		break;
	case DESC_RATEVHTSS2MCS0:
		ret_rate = MPT_RATE_VHT2SS_MCS0;
		break;
	case DESC_RATEVHTSS2MCS1:
		ret_rate = MPT_RATE_VHT2SS_MCS1;
		break;
	case DESC_RATEVHTSS2MCS2:
		ret_rate = MPT_RATE_VHT2SS_MCS2;
		break;
	case DESC_RATEVHTSS2MCS3:
		ret_rate = MPT_RATE_VHT2SS_MCS3;
		break;
	case DESC_RATEVHTSS2MCS4:
		ret_rate = MPT_RATE_VHT2SS_MCS4;
		break;
	case DESC_RATEVHTSS2MCS5:
		ret_rate = MPT_RATE_VHT2SS_MCS5;
		break;
	case DESC_RATEVHTSS2MCS6:
		ret_rate = MPT_RATE_VHT2SS_MCS6;
		break;
	case DESC_RATEVHTSS2MCS7:
		ret_rate = MPT_RATE_VHT2SS_MCS7;
		break;
	case DESC_RATEVHTSS2MCS8:
		ret_rate = MPT_RATE_VHT2SS_MCS8;
		break;
	case DESC_RATEVHTSS2MCS9:
		ret_rate = MPT_RATE_VHT2SS_MCS9;
		break;
	case DESC_RATEVHTSS3MCS0:
		ret_rate = MPT_RATE_VHT3SS_MCS0;
		break;
	case DESC_RATEVHTSS3MCS1:
		ret_rate = MPT_RATE_VHT3SS_MCS1;
		break;
	case DESC_RATEVHTSS3MCS2:
		ret_rate = MPT_RATE_VHT3SS_MCS2;
		break;
	case DESC_RATEVHTSS3MCS3:
		ret_rate = MPT_RATE_VHT3SS_MCS3;
		break;
	case DESC_RATEVHTSS3MCS4:
		ret_rate = MPT_RATE_VHT3SS_MCS4;
		break;
	case DESC_RATEVHTSS3MCS5:
		ret_rate = MPT_RATE_VHT3SS_MCS5;
		break;
	case DESC_RATEVHTSS3MCS6:
		ret_rate = MPT_RATE_VHT3SS_MCS6;
		break;
	case DESC_RATEVHTSS3MCS7:
		ret_rate = MPT_RATE_VHT3SS_MCS7;
		break;
	case DESC_RATEVHTSS3MCS8:
		ret_rate = MPT_RATE_VHT3SS_MCS8;
		break;
	case DESC_RATEVHTSS3MCS9:
		ret_rate = MPT_RATE_VHT3SS_MCS9;
		break;
	case DESC_RATEVHTSS4MCS0:
		ret_rate = MPT_RATE_VHT4SS_MCS0;
		break;
	case DESC_RATEVHTSS4MCS1:
		ret_rate = MPT_RATE_VHT4SS_MCS1;
		break;
	case DESC_RATEVHTSS4MCS2:
		ret_rate = MPT_RATE_VHT4SS_MCS2;
		break;
	case DESC_RATEVHTSS4MCS3:
		ret_rate = MPT_RATE_VHT4SS_MCS3;
		break;
	case DESC_RATEVHTSS4MCS4:
		ret_rate = MPT_RATE_VHT4SS_MCS4;
		break;
	case DESC_RATEVHTSS4MCS5:
		ret_rate = MPT_RATE_VHT4SS_MCS5;
		break;
	case DESC_RATEVHTSS4MCS6:
		ret_rate = MPT_RATE_VHT4SS_MCS6;
		break;
	case DESC_RATEVHTSS4MCS7:
		ret_rate = MPT_RATE_VHT4SS_MCS7;
		break;
	case DESC_RATEVHTSS4MCS8:
		ret_rate = MPT_RATE_VHT4SS_MCS8;
		break;
	case DESC_RATEVHTSS4MCS9:
		ret_rate = MPT_RATE_VHT4SS_MCS9;
		break;

	default:
		RTW_INFO("hw_rate_to_m_rate(): Non supported Rate [%x]!!!\n", rate);
		break;
	}
	return ret_rate;
}

u8 rtw_mpRateParseFunc(PADAPTER pAdapter, u8 *targetStr)
{
	u16 i = 0;
	u8 *rateindex_Array[] = { "1M", "2M", "5.5M", "11M", "6M", "9M", "12M", "18M", "24M", "36M", "48M", "54M",
		"HTMCS0", "HTMCS1", "HTMCS2", "HTMCS3", "HTMCS4", "HTMCS5", "HTMCS6", "HTMCS7",
		"HTMCS8", "HTMCS9", "HTMCS10", "HTMCS11", "HTMCS12", "HTMCS13", "HTMCS14", "HTMCS15",
		"HTMCS16", "HTMCS17", "HTMCS18", "HTMCS19", "HTMCS20", "HTMCS21", "HTMCS22", "HTMCS23",
		"HTMCS24", "HTMCS25", "HTMCS26", "HTMCS27", "HTMCS28", "HTMCS29", "HTMCS30", "HTMCS31",
		"VHT1MCS0", "VHT1MCS1", "VHT1MCS2", "VHT1MCS3", "VHT1MCS4", "VHT1MCS5", "VHT1MCS6", "VHT1MCS7", "VHT1MCS8", "VHT1MCS9",
		"VHT2MCS0", "VHT2MCS1", "VHT2MCS2", "VHT2MCS3", "VHT2MCS4", "VHT2MCS5", "VHT2MCS6", "VHT2MCS7", "VHT2MCS8", "VHT2MCS9",
		"VHT3MCS0", "VHT3MCS1", "VHT3MCS2", "VHT3MCS3", "VHT3MCS4", "VHT3MCS5", "VHT3MCS6", "VHT3MCS7", "VHT3MCS8", "VHT3MCS9",
		"VHT4MCS0", "VHT4MCS1", "VHT4MCS2", "VHT4MCS3", "VHT4MCS4", "VHT4MCS5", "VHT4MCS6", "VHT4MCS7", "VHT4MCS8", "VHT4MCS9"
				};

	for (i = 0; i <= 83; i++) {
		if (strcmp(targetStr, rateindex_Array[i]) == 0) {
			RTW_INFO("%s , index = %d\n", __func__ , i);
			return i;
		}
	}

	printk("%s ,please input a Data RATE String as:", __func__);
	for (i = 0; i <= 83; i++) {
		printk("%s ", rateindex_Array[i]);
		if (i % 10 == 0)
			printk("\n");
	}
	return _FAIL;
}

u8 rtw_mp_mode_check(PADAPTER pAdapter)
{
	PADAPTER primary_adapter = GET_PRIMARY_ADAPTER(pAdapter);

	if (primary_adapter->registrypriv.mp_mode == 1 || primary_adapter->mppriv.bprocess_mp_mode == _TRUE)
		return _TRUE;
	else
		return _FALSE;
}

bool rtw_is_mp_tssitrk_on(_adapter *adapter)
{
	_adapter *primary_adapter = GET_PRIMARY_ADAPTER(adapter);

	return primary_adapter->mppriv.tssitrk_on;
}

u32 mpt_ProQueryCalTxPower(
	PADAPTER	pAdapter,
	u8		RfPath
)
{

	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(pAdapter);
	PMPT_CONTEXT		pMptCtx = &(pAdapter->mppriv.mpt_ctx);

	u32			TxPower = 1;
	struct txpwr_idx_comp tic;
	u8 mgn_rate = mpt_to_mgnt_rate(pMptCtx->mpt_rate_index);
	RATE_SECTION rs = mgn_rate_to_rs(mgn_rate);

	TxPower = rtw_hal_get_tx_power_index(pAdapter, RfPath, rs, mgn_rate
		, pHalData->current_channel_bw, pHalData->current_band_type, pHalData->current_channel, 0, &tic);

	dump_tx_power_index_inline(RTW_DBGDUMP, pAdapter, RfPath
		, pHalData->current_channel_bw, pHalData->current_channel
		, mgn_rate, TxPower, &tic);

	pAdapter->mppriv.txpoweridx = (u8)TxPower;
	if (RfPath == RF_PATH_A)
		pMptCtx->TxPwrLevel[RF_PATH_A] = (u8)TxPower;
	else if (RfPath == RF_PATH_B)
		pMptCtx->TxPwrLevel[RF_PATH_B] = (u8)TxPower;
	else if (RfPath == RF_PATH_C)
		pMptCtx->TxPwrLevel[RF_PATH_C] = (u8)TxPower;
	else if (RfPath == RF_PATH_D)
		pMptCtx->TxPwrLevel[RF_PATH_D] = (u8)TxPower;
	hal_mpt_SetTxPower(pAdapter);

	return TxPower;
}

u32 mpt_get_tx_power_finalabs_val(PADAPTER	padapter, u8 rf_path)
{
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	PMPT_CONTEXT		pMptCtx = &(padapter->mppriv.mpt_ctx);

	u8 mgn_rate = mpt_to_mgnt_rate(pMptCtx->mpt_rate_index);
	u32 powerdbm = 0;

	powerdbm = phy_get_tx_power_final_absolute_value(padapter, rf_path, mgn_rate, pHalData->current_channel_bw, pHalData->current_channel);
	
	RTW_INFO("bw=%d, ch=%d, rateid=%d, TSSI Power(dBm):%d\n",
		pHalData->current_channel_bw, pHalData->current_channel, mgn_rate ,powerdbm);

	return powerdbm;
}

