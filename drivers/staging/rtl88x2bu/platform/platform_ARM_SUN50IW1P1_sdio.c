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
/*
 * Description:
 *	This file can be applied to following platforms:
 *	CONFIG_PLATFORM_ARM_SUN50IW1P1
 */
#include <drv_types.h>

#ifdef CONFIG_MMC
#endif /* CONFIG_MMC */

/*
 * Return:
 *	0:	power on successfully
 *	others: power on failed
 */
int platform_wifi_power_on(void)
{
	int ret = 0;

#ifdef CONFIG_MMC
	{

		RTW_INFO("%s: power up, rescan card.\n", __FUNCTION__);

	}
#endif /* CONFIG_MMC */

	return ret;
}

void platform_wifi_power_off(void)
{
#ifdef CONFIG_MMC
	RTW_INFO("%s: remove card, power off.\n", __FUNCTION__);
#endif /* CONFIG_MMC */
}
