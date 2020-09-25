/******************************************************************************
 *
 * Copyright(c) 2013 - 2017 Realtek Corporation.
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
#ifndef __MP_PRECOMP_H__
#define __MP_PRECOMP_H__

#include <drv_types.h>
#include <hal_data.h>
#include "btc_basic_types.h"

#define BT_TMP_BUF_SIZE	100

#define rsprintf snprintf
#define rstrncat(dst, src, src_size) strncat(dst, src, src_size)

#define DCMD_Printf			DBG_BT_INFO

#define delay_ms(ms)		rtw_mdelay_os(ms)

#ifdef bEnable
#undef bEnable
#endif

#define WPP_SOFTWARE_TRACE 0

typedef enum _BTC_MSG_COMP_TYPE {
	COMP_COEX		= 0,
	COMP_MAX
} BTC_MSG_COMP_TYPE;
extern u4Byte GLBtcDbgType[];

#define DBG_OFF			0
#define DBG_SEC			1
#define DBG_SERIOUS		2
#define DBG_WARNING		3
#define DBG_LOUD		4
#define DBG_TRACE		5

#define BT_SUPPORT		1
#define COEX_SUPPORT	1
#define HS_SUPPORT		1

/* for wifi only mode */
#include "hal_btcoex_wifionly.h"

#define BTC_BTINFO_LENGTH_MAX 10

struct wifi_only_cfg;
struct btc_coexist;





#ifdef CONFIG_RTL8703B
#include "halbtc8703b1ant.h"
#endif


#include "halbtc8822bwifionly.h"
#include "halbtc8822b1ant.h"
#include "halbtc8822b2ant.h"



#if (CONFIG_BTCOEX_SUPPORT_BTC_CMN == 1)
#include "halbtccommon.h"



#endif

#include "halbtcoutsrc.h"


#endif /*  __MP_PRECOMP_H__ */
