/******************************************************************************
 *
 * Copyright(c) 2007 - 2019 Realtek Corporation.
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
#ifndef __HAL_IC_CFG_H__
#define __HAL_IC_CFG_H__

#define RTL8188E_SUPPORT				0
#define RTL8812A_SUPPORT				0
#define RTL8821A_SUPPORT				0
#define RTL8723B_SUPPORT				0
#define RTL8723D_SUPPORT				0
#define RTL8192E_SUPPORT				0
#define RTL8192F_SUPPORT				0
#define RTL8814A_SUPPORT				0
#define RTL8195A_SUPPORT				0
#define RTL8197F_SUPPORT				0
#define RTL8703B_SUPPORT				0
#define RTL8188F_SUPPORT				0
#define RTL8822B_SUPPORT				0
#define RTL8821B_SUPPORT				0
#define RTL8821C_SUPPORT				0
#define RTL8710B_SUPPORT				0
#define RTL8814B_SUPPORT				0
#define RTL8824B_SUPPORT				0
#define RTL8198F_SUPPORT				0
#define RTL8195B_SUPPORT				0
#define RTL8822C_SUPPORT				0
#define RTL8721D_SUPPORT				0
#define RTL8812F_SUPPORT				0
#define RTL8197G_SUPPORT				0
#define RTL8710C_SUPPORT				0


/*#if (RTL8188E_SUPPORT==1)*/
#define RATE_ADAPTIVE_SUPPORT			0
#define POWER_TRAINING_ACTIVE			0










#ifdef CONFIG_RTL8703B
	#undef RTL8703B_SUPPORT
	#define RTL8703B_SUPPORT				1
	#ifndef CONFIG_FW_C2H_PKT
		#define CONFIG_FW_C2H_PKT
	#endif
	#ifndef CONFIG_RTW_MAC_HIDDEN_RPT
		#define CONFIG_RTW_MAC_HIDDEN_RPT
	#endif
	#define CONFIG_RTS_FULL_BW

	#ifndef CONFIG_TXPWR_PG_WITH_PWR_IDX
	#define CONFIG_TXPWR_PG_WITH_PWR_IDX
	#endif
#endif



	#undef RTL8822B_SUPPORT
	#define RTL8822B_SUPPORT				1
	#ifndef CONFIG_FW_C2H_PKT
		#define CONFIG_FW_C2H_PKT
	#endif /* CONFIG_FW_C2H_PKT */
	#define RTW_TX_PA_BIAS	/* Adjust TX PA Bias from eFuse */
	#define RTW_AMPDU_AGG_RETRY_AND_NEW


	#ifdef CONFIG_CONCURRENT_MODE
		#define CONFIG_AP_PORT_SWAP
		#define CONFIG_FW_MULTI_PORT_SUPPORT
	#endif /* CONFIG_CONCURRENT_MODE */

	/*
	 * Beamforming related definition
	 */
	/* Only support new beamforming mechanism */
		#define RTW_BEAMFORMING_VERSION_2

	#ifndef CONFIG_RTW_MAC_HIDDEN_RPT
		#define CONFIG_RTW_MAC_HIDDEN_RPT
	#endif /* CONFIG_RTW_MAC_HIDDEN_RPT */

	#ifndef DBG_RX_DFRAME_RAW_DATA
		#define DBG_RX_DFRAME_RAW_DATA
	#endif /* DBG_RX_DFRAME_RAW_DATA */

	#ifndef RTW_IQK_FW_OFFLOAD
		#define RTW_IQK_FW_OFFLOAD
	#endif /* RTW_IQK_FW_OFFLOAD */

	/* Checksum offload feature */
	/*#define CONFIG_TCP_CSUM_OFFLOAD_TX*/
	#define CONFIG_TCP_CSUM_OFFLOAD_RX

	#define CONFIG_ADVANCE_OTA



	#ifndef RTW_CHANNEL_SWITCH_OFFLOAD
		#ifdef CONFIG_TDLS_CH_SW_V2
			#define RTW_CHANNEL_SWITCH_OFFLOAD
		#endif
	#endif /* RTW_CHANNEL_SWITCH_OFFLOAD */

	#if defined(CONFIG_RTW_MESH) && !defined(RTW_PER_CMD_SUPPORT_FW)
		/* Supported since fw v22.1 */
		#define RTW_PER_CMD_SUPPORT_FW
	#endif /* RTW_PER_CMD_SUPPORT_FW */
	#define CONFIG_SUPPORT_FIFO_DUMP
	#define CONFIG_HW_P0_TSF_SYNC
	#define CONFIG_BCN_RECV_TIME
		#define CONFIG_P2P_PS_NOA_USE_MACID_SLEEP
	#define CONFIG_RTS_FULL_BW

		#define CONFIG_LPS_ACK	/* Supported after FW v30 & v27.9 */

	#ifndef CONFIG_TXPWR_PG_WITH_PWR_IDX
	#define CONFIG_TXPWR_PG_WITH_PWR_IDX
	#endif





#endif /*__HAL_IC_CFG_H__*/
