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

#include <drv_types.h>
#include <rtw_mp.h>
#include "../../hal/phydm/phydm_precomp.h"



#define RTW_IWD_MAX_LEN	128
inline u8 rtw_do_mp_iwdata_len_chk(const char *caller, u32 len)
{
	u8 is_illegal = _FALSE;
	if (len >= RTW_IWD_MAX_LEN) {
		RTW_ERR("%s : iw data len(%u) > RTW_IWD_MAX_LEN(%u)",
			caller, len, RTW_IWD_MAX_LEN);
		is_illegal = _TRUE;
	}
	return is_illegal;
}

/*
 * Input Format: %s,%d,%d
 *	%s is width, could be
 *		"b" for 1 byte
 *		"w" for WORD (2 bytes)
 *		"dw" for DWORD (4 bytes)
 *	1st %d is address(offset)
 *	2st %d is data to write
 */
int rtw_mp_write_reg(struct net_device *dev,
		     struct iw_request_info *info,
		     struct iw_point *wrqu, char *extra)
{
	char *pch, *pnext;
	char *width_str;
	char width;
	u32 addr, data;
	int ret = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	char input[RTW_IWD_MAX_LEN];

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';

	_rtw_memset(extra, 0, wrqu->length);

	pch = input;

	pnext = strpbrk(pch, " ,.-");
	if (pnext == NULL) {
		ret = -EINVAL;
		goto exit;
	}
	*pnext = 0;
	width_str = pch;

	pch = pnext + 1;
	pnext = strpbrk(pch, " ,.-");
	if (pnext == NULL) {
		ret = -EINVAL;
		goto exit;
	}
	*pnext = 0;
	/*addr = simple_strtoul(pch, &ptmp, 16);
	_rtw_memset(buf, '\0', sizeof(buf));
	_rtw_memcpy(buf, pch, pnext-pch);
	kstrtoul(buf, 16, &addr);*/
	sscanf(pch, "%x", &addr);
	if (addr > 0x3FFF) {
		ret = -EINVAL;
		goto exit;
	}

	pch = pnext + 1;
	pnext = strpbrk(pch, " ,.-");
	if ((pch - input) >= wrqu->length) {
		ret = -EINVAL;
		goto exit;
	}
	/*data = simple_strtoul(pch, &ptmp, 16);*/
	sscanf(pch, "%x", &data);
	RTW_INFO("data=%x,addr=%x\n", (u32)data, (u32)addr);
	width = width_str[0];
	switch (width) {
	case 'b':
		/* 1 byte*/
		if (data > 0xFF) {
			ret = -EINVAL;
			break;
		}
		rtw_write8(padapter, addr, data);
		break;
	case 'w':
		/* 2 bytes*/
		if (data > 0xFFFF) {
			ret = -EINVAL;
			break;
		}
		rtw_write16(padapter, addr, data);
		break;
	case 'd':
		/* 4 bytes*/
		rtw_write32(padapter, addr, data);
		break;
	default:
		ret = -EINVAL;
		break;
	}

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


/*
 * Input Format: %s,%d
 *	%s is width, could be
 *		"b" for 1 byte
 *		"w" for WORD (2 bytes)
 *		"dw" for DWORD (4 bytes)
 *	%d is address(offset)
 *
 * Return:
 *	%d for data readed
 */
int rtw_mp_read_reg(struct net_device *dev,
		    struct iw_request_info *info,
		    struct iw_point *wrqu, char *extra)
{
	char input[RTW_IWD_MAX_LEN];
	char *pch, *pnext;
	char *width_str;
	char width;
	char data[20], tmp[20];
	u32 addr = 0, strtout = 0;
	u32 i = 0, j = 0, data32 = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	char *pextra = extra;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, wrqu->length);

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	_rtw_memset(extra, 0, wrqu->length);
	_rtw_memset(data, '\0', sizeof(data));
	_rtw_memset(tmp, '\0', sizeof(tmp));
	pch = input;
	pnext = strpbrk(pch, " ,.-");
	if (pnext == NULL) {
		ret = -EINVAL;
		goto exit;
	}
	*pnext = 0;
	width_str = pch;

	pch = pnext + 1;

	sscanf(pch, "%x", &addr);
	if (addr > MP_READ_REG_MAX_OFFSET) {
		ret = -EINVAL;
		goto exit;
	}

	width = width_str[0];

	switch (width) {
	case 'b':
		data32 = rtw_read8(padapter, addr);
		RTW_INFO("%x\n", data32);
		sprintf(extra, "%d", data32);
		wrqu->length = strlen(extra);
		break;
	case 'w':
		/* 2 bytes*/
		sprintf(data, "%04x\n", rtw_read16(padapter, addr));

		for (i = 0 ; i <= strlen(data) ; i++) {
			if (i % 2 == 0) {
				tmp[j] = ' ';
				j++;
			}
			if (data[i] != '\0')
				tmp[j] = data[i];

			j++;
		}
		pch = tmp;
		RTW_INFO("pch=%s", pch);

		while (*pch != '\0') {
			pnext = strpbrk(pch, " ");
			if (!pnext || ((pnext - tmp) > 4))
				break;

			pnext++;
			if (*pnext != '\0') {
				/*strtout = simple_strtoul(pnext , &ptmp, 16);*/
				sscanf(pnext, "%x", &strtout);
				pextra += sprintf(pextra, " %d", strtout);
			} else
				break;
			pch = pnext;
		}
		wrqu->length = strlen(extra);
		break;
	case 'd':
		/* 4 bytes */
		sprintf(data, "%08x", rtw_read32(padapter, addr));
		/*add read data format blank*/
		for (i = 0 ; i <= strlen(data) ; i++) {
			if (i % 2 == 0) {
				tmp[j] = ' ';
				j++;
			}
			if (data[i] != '\0')
				tmp[j] = data[i];

			j++;
		}
		pch = tmp;
		RTW_INFO("pch=%s", pch);

		while (*pch != '\0') {
			pnext = strpbrk(pch, " ");
			if (!pnext)
				break;

			pnext++;
			if (*pnext != '\0') {
				sscanf(pnext, "%x", &strtout);
				pextra += sprintf(pextra, " %d", strtout);
			} else
				break;
			pch = pnext;
		}
		wrqu->length = strlen(extra);
		break;

	default:
		wrqu->length = 0;
		ret = -EINVAL;
		break;
	}

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


/*
 * Input Format: %d,%x,%x
 *	%d is RF path, should be smaller than MAX_RF_PATH_NUMS
 *	1st %x is address(offset)
 *	2st %x is data to write
 */
int rtw_mp_write_rf(struct net_device *dev,
		    struct iw_request_info *info,
		    struct iw_point *wrqu, char *extra)
{

	u32 path, addr, data;
	int ret = 0, cnt;
	PADAPTER padapter = rtw_netdev_priv(dev);
	char input[RTW_IWD_MAX_LEN];

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, wrqu->length);

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	cnt = sscanf(input, "%d,%x,%x", &path, &addr, &data);
	if (cnt < 3) {
		ret = -EINVAL;
		goto exit;
	}

	if ((path >= GET_HAL_RFPATH_NUM(padapter))
	    || (addr > 0xFF) || (data > 0xFFFFF)) {
		ret = -EINVAL;
		goto exit;
	}

	_rtw_memset(extra, 0, wrqu->length);

	write_rfreg(padapter, path, addr, data);

	sprintf(extra, "write_rf completed\n");
	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}


/*
 * Input Format: %d,%x
 *	%d is RF path, should be smaller than MAX_RF_PATH_NUMS
 *	%x is address(offset)
 *
 * Return:
 *	%d for data readed
 */
int rtw_mp_read_rf(struct net_device *dev,
		   struct iw_request_info *info,
		   struct iw_point *wrqu, char *extra)
{
	char input[RTW_IWD_MAX_LEN];
	char *pch, *pnext;
	char data[20], tmp[20];
	u32 path, addr, strtou;
	u32 i = 0 , j = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	char *pextra = extra;
	int ret = 0, cnt;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, wrqu->length);

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	cnt = sscanf(input, "%d,%x", &path, &addr);
	if (cnt < 2) {
		ret = -EINVAL;
		goto exit;
	}

	if ((path >= GET_HAL_RFPATH_NUM(padapter))
	    || (addr > MP_READ_REG_MAX_OFFSET)) {
		ret = -EINVAL;
		goto exit;
	}

	_rtw_memset(extra, 0, wrqu->length);

	sprintf(data, "%08x", read_rfreg(padapter, path, addr));
	/*add read data format blank*/
	for (i = 0 ; i <= strlen(data) ; i++) {
		if (i % 2 == 0) {
			tmp[j] = ' ';
			j++;
		}
		tmp[j] = data[i];
		j++;
	}
	pch = tmp;
	RTW_INFO("pch=%s", pch);

	while (*pch != '\0') {
		pnext = strpbrk(pch, " ");
		if (!pnext)
			break;
		pnext++;
		if (*pnext != '\0') {
			/*strtou =simple_strtoul(pnext , &ptmp, 16);*/
			sscanf(pnext, "%x", &strtou);
			pextra += sprintf(pextra, " %d", strtou);
		} else
			break;
		pch = pnext;
	}
	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}


int rtw_mp_start(struct net_device *dev,
		 struct iw_request_info *info,
		 struct iw_point *wrqu, char *extra)
{
	int ret = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmppriv = &padapter->mppriv;

	rtw_pm_set_ips(padapter, IPS_NONE);
	LeaveAllPowerSaveMode(padapter);

	pmppriv->bprocess_mp_mode = _TRUE;

	if (rtw_mi_check_fwstate(padapter, _FW_UNDER_SURVEY)) {
		rtw_mi_buddy_set_scan_deny(padapter, 5000);
		rtw_mi_scan_abort(padapter, _TRUE);
	}

	rtw_hal_set_hwreg(padapter, HW_VAR_CHECK_TXBUF, 0);

	if (rtw_mp_cmd(padapter, MP_START, RTW_CMDF_WAIT_ACK) != _SUCCESS)
		ret = -EPERM;

	_rtw_memset(extra, 0, wrqu->length);
	sprintf(extra, "mp_start %s\n", ret == 0 ? "ok" : "fail");
	wrqu->length = strlen(extra);

	return ret;
}



int rtw_mp_stop(struct net_device *dev,
		struct iw_request_info *info,
		struct iw_point *wrqu, char *extra)
{
	int ret = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmppriv = &padapter->mppriv;

	if (rtw_mp_cmd(padapter, MP_STOP, RTW_CMDF_WAIT_ACK) != _SUCCESS)
		ret = -EPERM;

	if (pmppriv->mode != MP_ON)
		return -EPERM;

	pmppriv->bprocess_mp_mode = _FALSE;
	_rtw_memset(extra, 0, wrqu->length);
	sprintf(extra, "mp_stop %s\n", ret == 0 ? "ok" : "fail");
	wrqu->length = strlen(extra);

	return ret;
}


int rtw_mp_rate(struct net_device *dev,
		struct iw_request_info *info,
		struct iw_point *wrqu, char *extra)
{
	u32 rate = MPT_RATE_1M;
	u8 input[RTW_IWD_MAX_LEN];
	PADAPTER padapter = rtw_netdev_priv(dev);
	PMPT_CONTEXT		pMptCtx = &(padapter->mppriv.mpt_ctx);
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	rate = rtw_mpRateParseFunc(padapter, input);
	padapter->mppriv.rateidx = rate;

	if (rate == 0 && strcmp(input, "1M") != 0) {
		rate = rtw_atoi(input);
		padapter->mppriv.rateidx = MRateToHwRate(rate);
		/*if (rate <= 0x7f)
			rate = wifirate2_ratetbl_inx((u8)rate);
		else if (rate < 0xC8)
			rate = (rate - 0x79 + MPT_RATE_MCS0);
		HT  rate 0x80(MCS0)  ~ 0x8F(MCS15) ~ 0x9F(MCS31) 128~159
		VHT1SS~2SS rate 0xA0 (VHT1SS_MCS0 44) ~ 0xB3 (VHT2SS_MCS9 #63) 160~179
		VHT rate 0xB4 (VHT3SS_MCS0 64) ~ 0xC7 (VHT2SS_MCS9 #83) 180~199
		else
		VHT rate 0x90(VHT1SS_MCS0) ~ 0x99(VHT1SS_MCS9) 144~153
		rate =(rate - MPT_RATE_VHT1SS_MCS0);
		*/
	}
	_rtw_memset(extra, 0, wrqu->length);

	sprintf(extra, "Set data rate to %s index %d" , input, padapter->mppriv.rateidx);
	RTW_INFO("%s: %s rate index=%d\n", __func__, input, padapter->mppriv.rateidx);

	if (padapter->mppriv.rateidx >= DESC_RATEVHTSS4MCS9) {
		ret = -EINVAL;
		goto exit;
	}

	pMptCtx->mpt_rate_index = HwRateToMPTRate(padapter->mppriv.rateidx);
	SetDataRate(padapter);

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


int rtw_mp_channel(struct net_device *dev,
		   struct iw_request_info *info,
		   struct iw_point *wrqu, char *extra)
{

	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	u8 input[RTW_IWD_MAX_LEN];
	u32	channel = 1;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	channel = rtw_atoi(input);
	/*RTW_INFO("%s: channel=%d\n", __func__, channel);*/
	_rtw_memset(extra, 0, wrqu->length);
	sprintf(extra, "Change channel %d to channel %d", padapter->mppriv.channel , channel);
	padapter->mppriv.channel = channel;
	rtw_hal_set_hwreg(padapter, HW_VAR_CHECK_TXBUF, 0);
	SetChannel(padapter);
	pHalData->current_channel = channel;

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


int rtw_mp_ch_offset(struct net_device *dev,
		   struct iw_request_info *info,
		   struct iw_point *wrqu, char *extra)
{

	PADAPTER padapter = rtw_netdev_priv(dev);
	u8 input[RTW_IWD_MAX_LEN];
	u32	ch_offset = 0;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	ch_offset = rtw_atoi(input);
	/*RTW_INFO("%s: channel=%d\n", __func__, channel);*/
	_rtw_memset(extra, 0, wrqu->length);
	sprintf(extra, "Change prime channel offset %d to %d", padapter->mppriv.prime_channel_offset , ch_offset);
	padapter->mppriv.prime_channel_offset = ch_offset;
	SetChannel(padapter);

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


int rtw_mp_bandwidth(struct net_device *dev,
		     struct iw_request_info *info,
		     struct iw_point *wrqu, char *extra)
{
	u32 bandwidth = 0, sg = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	u8 input[RTW_IWD_MAX_LEN];
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	if (sscanf(input, "40M=%d,shortGI=%d", &bandwidth, &sg) > 0)
		RTW_INFO("%s: bw=%d sg=%d\n", __func__, bandwidth , sg);

	if (bandwidth == 1 && hal_chk_bw_cap(padapter, BW_CAP_40M))
		bandwidth = CHANNEL_WIDTH_40;
	else if (bandwidth == 2 && hal_chk_bw_cap(padapter, BW_CAP_80M))
		bandwidth = CHANNEL_WIDTH_80;
	else
		bandwidth = CHANNEL_WIDTH_20;

	padapter->mppriv.bandwidth = (u8)bandwidth;
	padapter->mppriv.preamble = sg;
	_rtw_memset(extra, 0, wrqu->length);
	sprintf(extra, "Change BW %d to BW %d\n", pHalData->current_channel_bw , bandwidth);
	rtw_hal_set_hwreg(padapter, HW_VAR_CHECK_TXBUF, 0);
	SetBandwidth(padapter);
	pHalData->current_channel_bw = bandwidth;

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}


int rtw_mp_txpower_index(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
 	HAL_DATA_TYPE	*phal_data	= GET_HAL_DATA(padapter);
	char input[RTW_IWD_MAX_LEN];
	u32 rfpath = 0 ;
	u32 txpower_inx = 0, tarpowerdbm = 0;
	char *pextra = extra;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	_rtw_memset(extra, 0, strlen(extra));

	if (wrqu->length == 2) {
		if (input[0] != '\0' ) {
			rfpath = rtw_atoi(input);
			txpower_inx = mpt_ProQueryCalTxPower(padapter, rfpath);
		}
		pextra += sprintf(pextra, " %d\n", txpower_inx);
		if (IS_HARDWARE_TYPE_JAGUAR3(padapter)) {
			tarpowerdbm = mpt_get_tx_power_finalabs_val(padapter, rfpath);
			if (tarpowerdbm > 0)
				pextra += sprintf(pextra, " dBm:%d", tarpowerdbm);
		}
	} else {
		txpower_inx = mpt_ProQueryCalTxPower(padapter, 0);
		pextra += sprintf(pextra, "patha=%d", txpower_inx);
		if (phal_data->rf_type > RF_1T2R) {
			txpower_inx = mpt_ProQueryCalTxPower(padapter, 1);
			pextra += sprintf(pextra, ",pathb=%d", txpower_inx);
		}
		if (phal_data->rf_type > RF_2T4R) {
			txpower_inx = mpt_ProQueryCalTxPower(padapter, 2);
			pextra += sprintf(pextra, ",pathc=%d", txpower_inx);
		}
		if (phal_data->rf_type > RF_3T4R) {
			txpower_inx = mpt_ProQueryCalTxPower(padapter, 3);
			pextra += sprintf(pextra, ",pathd=%d\n", txpower_inx);
		}

		if (IS_HARDWARE_TYPE_JAGUAR3(padapter)) {
			tarpowerdbm = mpt_get_tx_power_finalabs_val(padapter, 0);
			pextra += sprintf(pextra, "patha dBm=%d", tarpowerdbm);
			if (phal_data->rf_type > RF_1T2R) {
				tarpowerdbm = mpt_get_tx_power_finalabs_val(padapter, 1);
				pextra += sprintf(pextra, "pathb dBm=%d", tarpowerdbm);
			}
			if (phal_data->rf_type > RF_2T4R) {
				tarpowerdbm = mpt_get_tx_power_finalabs_val(padapter, 2);
				pextra += sprintf(pextra, "pathc dBm=%d", tarpowerdbm);
			}
			if (phal_data->rf_type > RF_3T4R) {
				tarpowerdbm = mpt_get_tx_power_finalabs_val(padapter, 3);
				pextra += sprintf(pextra, "pathd dBm=%d", tarpowerdbm);
			}
		}
	}
	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


int rtw_mp_txpower(struct net_device *dev,
		   struct iw_request_info *info,
		   struct iw_point *wrqu, char *extra)
{
	u32 idx_a = 0, idx_b = 0, idx_c = 0, idx_d = 0;
	int MsetPower = 1;
	u8 input[RTW_IWD_MAX_LEN];
	int ret = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	PMPT_CONTEXT		pMptCtx = &(padapter->mppriv.mpt_ctx);

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	MsetPower = strncmp(input, "off", 3);
	if (MsetPower == 0) {
		padapter->mppriv.bSetTxPower = 0;
		sprintf(extra, "MP Set power off");
	} else {
		if (sscanf(input, "patha=%d,pathb=%d,pathc=%d,pathd=%d", &idx_a, &idx_b, &idx_c, &idx_d) < 3)
			RTW_INFO("Invalid format on line %s ,patha=%d,pathb=%d,pathc=%d,pathd=%d\n", input , idx_a , idx_b , idx_c , idx_d);

		sprintf(extra, "Set power level path_A:%d path_B:%d path_C:%d path_D:%d", idx_a , idx_b , idx_c , idx_d);
		padapter->mppriv.txpoweridx = (u8)idx_a;

		pMptCtx->TxPwrLevel[RF_PATH_A] = (u8)idx_a;
		pMptCtx->TxPwrLevel[RF_PATH_B] = (u8)idx_b;
		pMptCtx->TxPwrLevel[RF_PATH_C] = (u8)idx_c;
		pMptCtx->TxPwrLevel[RF_PATH_D]  = (u8)idx_d;
		padapter->mppriv.bSetTxPower = 1;

		SetTxPower(padapter);
	}

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length);

	return 0;
}


int rtw_mp_ant_tx(struct net_device *dev,
		  struct iw_request_info *info,
		  struct iw_point *wrqu, char *extra)
{
	u8 i;
	u8 input[RTW_IWD_MAX_LEN];
	u16 antenna = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	sprintf(extra, "switch Tx antenna to %s", input);

	for (i = 0; i < strlen(input); i++) {
		switch (input[i]) {
		case 'a':
			antenna |= ANTENNA_A;
			break;
		case 'b':
			antenna |= ANTENNA_B;
			break;
		case 'c':
			antenna |= ANTENNA_C;
			break;
		case 'd':
			antenna |= ANTENNA_D;
			break;
		}
	}
	/*antenna |= BIT(extra[i]-'a');*/
	RTW_INFO("%s: antenna=0x%x\n", __func__, antenna);
	padapter->mppriv.antenna_tx = antenna;

	/*RTW_INFO("%s:mppriv.antenna_rx=%d\n", __func__, padapter->mppriv.antenna_tx);*/
	pHalData->antenna_tx_path = antenna;
	if (IS_HARDWARE_TYPE_8822C(padapter) && padapter->mppriv.antenna_tx == ANTENNA_B) {
		if (padapter->mppriv.antenna_rx == ANTENNA_A || padapter->mppriv.antenna_rx == ANTENNA_B) {
			padapter->mppriv.antenna_rx = ANTENNA_AB;
			pHalData->AntennaRxPath = ANTENNA_AB;
			RTW_INFO("%s:8822C Tx-B Rx Ant to AB\n", __func__);
		}
	}
	SetAntenna(padapter);

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


int rtw_mp_ant_rx(struct net_device *dev,
		  struct iw_request_info *info,
		  struct iw_point *wrqu, char *extra)
{
	u8 i;
	u16 antenna = 0;
	u8 input[RTW_IWD_MAX_LEN];
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	/*RTW_INFO("%s: input=%s\n", __func__, input);*/
	_rtw_memset(extra, 0, wrqu->length);

	sprintf(extra, "switch Rx antenna to %s", input);

	for (i = 0; i < strlen(input); i++) {
		switch (input[i]) {
		case 'a':
			antenna |= ANTENNA_A;
			break;
		case 'b':
			antenna |= ANTENNA_B;
			break;
		case 'c':
			antenna |= ANTENNA_C;
			break;
		case 'd':
			antenna |= ANTENNA_D;
			break;
		}
	}

	RTW_INFO("%s: antenna=0x%x\n", __func__, antenna);

	padapter->mppriv.antenna_rx = antenna;
	pHalData->AntennaRxPath = antenna;
	/*RTW_INFO("%s:mppriv.antenna_rx=%d\n", __func__, padapter->mppriv.antenna_rx);*/
	SetAntenna(padapter);
	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


int rtw_set_ctx_destAddr(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_point *wrqu, char *extra)
{
	int jj, kk = 0;

	struct pkt_attrib *pattrib;
	struct mp_priv *pmp_priv;
	PADAPTER padapter = rtw_netdev_priv(dev);

	pmp_priv = &padapter->mppriv;
	pattrib = &pmp_priv->tx.attrib;

	if (strlen(extra) < 5)
		return _FAIL;

	RTW_INFO("%s: in=%s\n", __func__, extra);
	for (jj = 0, kk = 0; jj < ETH_ALEN; jj++, kk += 3)
		pattrib->dst[jj] = key_2char2num(extra[kk], extra[kk + 1]);

	RTW_INFO("pattrib->dst:%x %x %x %x %x %x\n", pattrib->dst[0], pattrib->dst[1], pattrib->dst[2], pattrib->dst[3], pattrib->dst[4], pattrib->dst[5]);
	return 0;
}



int rtw_mp_ctx(struct net_device *dev,
	       struct iw_request_info *info,
	       struct iw_point *wrqu, char *extra)
{
	u32 pkTx = 1;
	int countPkTx = 1, cotuTx = 1, CarrSprTx = 1, scTx = 1, sgleTx = 1, stop = 1, payload = 1;
	u32 bStartTest = 1;
	u32 count = 0, pktinterval = 0, pktlen = 0;
	u8 status;
	struct mp_priv *pmp_priv;
	struct pkt_attrib *pattrib;
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	pmp_priv = &padapter->mppriv;
	pattrib = &pmp_priv->tx.attrib;

	if (padapter->registrypriv.mp_mode != 1 ) {
		sprintf(extra, "Error: can't tx ,not in MP mode. \n");
		wrqu->length = strlen(extra);
		return 0;
	}

	if (copy_from_user(extra, wrqu->pointer, wrqu->length))
		return -EFAULT;

	*(extra + wrqu->length) = '\0';
	RTW_INFO("%s: in=%s\n", __func__, extra);
#ifdef CONFIG_CONCURRENT_MODE
	if (!is_primary_adapter(padapter)) {
		sprintf(extra, "Error: MP mode can't support Virtual Adapter, Please to use main Adapter.\n");
		wrqu->length = strlen(extra);
		return 0;
	}
#endif
	countPkTx = strncmp(extra, "count=", 5); /* strncmp TRUE is 0*/
	cotuTx = strncmp(extra, "background", 20);
	CarrSprTx = strncmp(extra, "background,cs", 20);
	scTx = strncmp(extra, "background,sc", 20);
	sgleTx = strncmp(extra, "background,stone", 20);
	pkTx = strncmp(extra, "background,pkt", 20);
	stop = strncmp(extra, "stop", 4);
	payload = strncmp(extra, "payload=", 8);

	if (sscanf(extra, "count=%d,pkt", &count) > 0)
		RTW_INFO("count= %d\n", count);
	if (sscanf(extra, "pktinterval=%d", &pktinterval) > 0)
		RTW_INFO("pktinterval= %d\n", pktinterval);
	if (sscanf(extra, "pktlen=%d", &pktlen) > 0)
		RTW_INFO("pktlen= %d\n", pktlen);

	if (payload == 0) {
			payload = MP_TX_Payload_default_random;
			if (strncmp(extra, "payload=prbs9", 14) == 0) {
				payload = MP_TX_Payload_prbs9;
				sprintf(extra, "config payload PRBS9\n");
			} else {
				if (sscanf(extra, "payload=%x", &payload) > 0){
					RTW_INFO("payload= %x\n", payload);
					sprintf(extra, "config payload setting = %x\n"
									"1. input payload=[]:\n		"
									"[0]: 00, [1]: A5, [2]: 5A, [3]: FF, [4]: PRBS-9, [5]: Random\n"
									"2. specified a hex payload: payload=0xee\n", payload);
				 }
			}
			pmp_priv->tx.payload = payload;
			wrqu->length = strlen(extra);
			return 0;
	}

	if (_rtw_memcmp(extra, "destmac=", 8)) {
		wrqu->length -= 8;
		rtw_set_ctx_destAddr(dev, info, wrqu, &extra[8]);
		sprintf(extra, "Set dest mac OK !\n");
		return 0;
	}
	/*RTW_INFO("%s: count=%d countPkTx=%d cotuTx=%d CarrSprTx=%d scTx=%d sgleTx=%d pkTx=%d stop=%d\n", __func__, count, countPkTx, cotuTx, CarrSprTx, pkTx, sgleTx, scTx, stop);*/
	_rtw_memset(extra, '\0', strlen(extra));

	if (pktinterval != 0) {
		sprintf(extra, "Pkt Interval = %d", pktinterval);
		padapter->mppriv.pktInterval = pktinterval;
		wrqu->length = strlen(extra);
		return 0;

	} else if (pktlen != 0) {
		sprintf(extra, "Pkt len = %d", pktlen);
		pattrib->pktlen = pktlen;
		wrqu->length = strlen(extra);
		return 0;

	} else if (stop == 0) {
		struct xmit_priv	*pxmitpriv = &(padapter->xmitpriv);
		_queue *pfree_xmitbuf_queue = &pxmitpriv->free_xmitbuf_queue;
		_queue *pfree_xmit_queue = &pxmitpriv->free_xmit_queue;

		u32 i = 0;
		bStartTest = 0; /* To set Stop*/
		pmp_priv->tx.stop = 1;
		sprintf(extra, "Stop continuous Tx");
		odm_write_dig(&pHalData->odmpriv, 0x20);
		do {
			if (pxmitpriv->free_xmitframe_cnt == NR_XMITFRAME && pxmitpriv->free_xmitbuf_cnt == NR_XMITBUFF)
				break;
			else {
				i++;
				RTW_INFO("%s:wait queue_empty %d!!\n", __func__, i);
				rtw_msleep_os(10);
			}
		} while (i < 1000);
	} else {
		bStartTest = 1;
		odm_write_dig(&pHalData->odmpriv, 0x3f);
		if (IS_HARDWARE_TYPE_8822C(padapter) && pmp_priv->antenna_tx == ANTENNA_B) {
			if (pmp_priv->antenna_rx == ANTENNA_A || pmp_priv->antenna_rx == ANTENNA_B) {
				pmp_priv->antenna_rx = ANTENNA_AB;
				pHalData->AntennaRxPath = ANTENNA_AB;
				RTW_INFO("%s:8822C Tx-B Rx Ant to AB\n", __func__);
				SetAntenna(padapter);
			}
		}
		if (pmp_priv->mode != MP_ON) {
			if (pmp_priv->tx.stop != 1) {
				RTW_INFO("%s:Error MP_MODE %d != ON\n", __func__, pmp_priv->mode);
				return	-EFAULT;
			}
		}
	}

	pmp_priv->tx.count = count;

	if (pkTx == 0 || countPkTx == 0)
		pmp_priv->mode = MP_PACKET_TX;
	if (sgleTx == 0)
		pmp_priv->mode = MP_SINGLE_TONE_TX;
	if (cotuTx == 0)
		pmp_priv->mode = MP_CONTINUOUS_TX;
	if (CarrSprTx == 0)
		pmp_priv->mode = MP_CARRIER_SUPPRISSION_TX;
	if (scTx == 0)
		pmp_priv->mode = MP_SINGLE_CARRIER_TX;

	status = rtw_mp_pretx_proc(padapter, bStartTest, extra);

	if (stop == 0)
		pmp_priv->mode = MP_ON;

	wrqu->length = strlen(extra);
	return status;
}



int rtw_mp_disable_bt_coexist(struct net_device *dev,
			      struct iw_request_info *info,
			      union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = (PADAPTER)rtw_netdev_priv(dev);

	u8 input[RTW_IWD_MAX_LEN];
	u32 bt_coexist;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->data.length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->data.pointer, wrqu->data.length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->data.length] = '\0';

	bt_coexist = rtw_atoi(input);

	if (bt_coexist == 0) {
		RTW_INFO("Set OID_RT_SET_DISABLE_BT_COEXIST: disable BT_COEXIST\n");
		rtw_btcoex_HaltNotify(padapter);
		rtw_btcoex_SetManualControl(padapter, _TRUE);
		/* Force to switch Antenna to WiFi*/
		rtw_write16(padapter, 0x870, 0x300);
		rtw_write16(padapter, 0x860, 0x110);
		/* CONFIG_BT_COEXIST */
	} else {
		rtw_btcoex_SetManualControl(padapter, _FALSE);
	}

exit:
	rtw_mfree(input, wrqu->data.length + 1);

	return ret;
}


int rtw_mp_arx(struct net_device *dev,
	       struct iw_request_info *info,
	       struct iw_point *wrqu, char *extra)
{
	int bStartRx = 0, bStopRx = 0, bQueryPhy = 0, bQueryMac = 0, bSetBssid = 0, bSetRxframe = 0;
	int bmac_filter = 0, bmon = 0, bSmpCfg = 0;
	u8 input[RTW_IWD_MAX_LEN];
	char *pch, *token, *tmp[2] = {0x00, 0x00};
	u32 i = 0, jj = 0, kk = 0, cnts = 0;
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmppriv = &padapter->mppriv;
	struct dbg_rx_counter rx_counter;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	RTW_INFO("%s: %s\n", __func__, input);
#ifdef CONFIG_CONCURRENT_MODE
	if (!is_primary_adapter(padapter)) {
		sprintf(extra, "Error: MP mode can't support Virtual Adapter, Please to use main Adapter.\n");
		wrqu->length = strlen(extra);
		goto exit;
	}
#endif
	bStartRx = (strncmp(input, "start", 5) == 0) ? 1 : 0; /* strncmp TRUE is 0*/
	bStopRx = (strncmp(input, "stop", 5) == 0) ? 1 : 0; /* strncmp TRUE is 0*/
	bQueryPhy = (strncmp(input, "phy", 3) == 0) ? 1 : 0; /* strncmp TRUE is 0*/
	bQueryMac = (strncmp(input, "mac", 3) == 0) ? 1 : 0; /* strncmp TRUE is 0*/
	bSetBssid = (strncmp(input, "setbssid=", 8) == 0) ? 1 : 0; /* strncmp TRUE is 0*/
	bSetRxframe = (strncmp(input, "frametype", 9) == 0) ? 1 : 0;
	/*bfilter_init = (strncmp(input, "filter_init",11)==0)?1:0;*/
	bmac_filter = (strncmp(input, "accept_mac", 10) == 0) ? 1 : 0;
	bmon = (strncmp(input, "mon=", 4) == 0) ? 1 : 0;
	bSmpCfg = (strncmp(input , "smpcfg=" , 7) == 0) ? 1 : 0;
	pmppriv->bloopback = (strncmp(input, "loopbk", 6) == 0) ? 1 : 0; /* strncmp TRUE is 0*/

	if (bSetBssid == 1) {
		pch = input;
		while ((token = strsep(&pch, "=")) != NULL) {
			if (i > 1)
				break;
			tmp[i] = token;
			i++;
		}
		if ((tmp[0] != NULL) && (tmp[1] != NULL)) {
			cnts = strlen(tmp[1]) / 2;
			if (cnts < 1)
				return -EFAULT;
			RTW_INFO("%s: cnts=%d\n", __func__, cnts);
			RTW_INFO("%s: data=%s\n", __func__, tmp[1]);
			for (jj = 0, kk = 0; jj < cnts ; jj++, kk += 2) {
				pmppriv->network_macaddr[jj] = key_2char2num(tmp[1][kk], tmp[1][kk + 1]);
				RTW_INFO("network_macaddr[%d]=%x\n", jj, pmppriv->network_macaddr[jj]);
			}
		} else {
			ret = -EFAULT;
			goto exit;
		}

		pmppriv->bSetRxBssid = _TRUE;
	}
	if (bSetRxframe) {
		if (strncmp(input, "frametype beacon", 16) == 0)
			pmppriv->brx_filter_beacon = _TRUE;
		else
			pmppriv->brx_filter_beacon = _FALSE;
	}

	if (bmac_filter) {
		pmppriv->bmac_filter = bmac_filter;
		pch = input;
		while ((token = strsep(&pch, "=")) != NULL) {
			if (i > 1)
				break;
			tmp[i] = token;
			i++;
		}
		if ((tmp[0] != NULL) && (tmp[1] != NULL)) {
			cnts = strlen(tmp[1]) / 2;
			if (cnts < 1) {
				ret = -EFAULT;
				goto exit;
			}
			RTW_INFO("%s: cnts=%d\n", __func__, cnts);
			RTW_INFO("%s: data=%s\n", __func__, tmp[1]);
			for (jj = 0, kk = 0; jj < cnts ; jj++, kk += 2) {
				pmppriv->mac_filter[jj] = key_2char2num(tmp[1][kk], tmp[1][kk + 1]);
				RTW_INFO("%s mac_filter[%d]=%x\n", __func__, jj, pmppriv->mac_filter[jj]);
			}
		} else {
			ret = -EFAULT;
			goto exit;
		}
	}

	if (bStartRx) {
		sprintf(extra, "start");
		SetPacketRx(padapter, bStartRx, _FALSE);
	} else if (bStopRx) {
		SetPacketRx(padapter, bStartRx, _FALSE);
		pmppriv->bmac_filter = _FALSE;
		pmppriv->bSetRxBssid = _FALSE;
		sprintf(extra, "Received packet OK:%d CRC error:%d ,Filter out:%d", padapter->mppriv.rx_pktcount, padapter->mppriv.rx_crcerrpktcount, padapter->mppriv.rx_pktcount_filter_out);
	} else if (bQueryPhy) {
		_rtw_memset(&rx_counter, 0, sizeof(struct dbg_rx_counter));
		rtw_dump_phy_rx_counters(padapter, &rx_counter);

		RTW_INFO("%s: OFDM_FA =%d\n", __func__, rx_counter.rx_ofdm_fa);
		RTW_INFO("%s: CCK_FA =%d\n", __func__, rx_counter.rx_cck_fa);
		sprintf(extra, "Phy Received packet OK:%d CRC error:%d FA Counter: %d", rx_counter.rx_pkt_ok, rx_counter.rx_pkt_crc_error, rx_counter.rx_cck_fa + rx_counter.rx_ofdm_fa);


	} else if (bQueryMac) {
		_rtw_memset(&rx_counter, 0, sizeof(struct dbg_rx_counter));
		rtw_dump_mac_rx_counters(padapter, &rx_counter);
		sprintf(extra, "Mac Received packet OK: %d , CRC error: %d , Drop Packets: %d\n",
			rx_counter.rx_pkt_ok, rx_counter.rx_pkt_crc_error, rx_counter.rx_pkt_drop);

	}

	if (bmon == 1) {
		sscanf(input, "mon=%d", &bmon);

		if (bmon == 1) {
			pmppriv->rx_bindicatePkt = _TRUE;
			sprintf(extra, "Indicating Receive Packet to network start\n");
		} else {
			pmppriv->rx_bindicatePkt = _FALSE;
			sprintf(extra, "Indicating Receive Packet to network Stop\n");
		}
	}
	if (bSmpCfg == 1) {
		sscanf(input, "smpcfg=%d", &bSmpCfg);

		if (bSmpCfg == 1) {
			pmppriv->bRTWSmbCfg = _TRUE;
			sprintf(extra , "Indicate By Simple Config Format\n");
			SetPacketRx(padapter, _TRUE, _TRUE);
		} else {
			pmppriv->bRTWSmbCfg = _FALSE;
			sprintf(extra , "Indicate By Normal Format\n");
			SetPacketRx(padapter, _TRUE, _FALSE);
		}
	}

	if (pmppriv->bloopback == _TRUE) {
		sprintf(extra , "Enter MAC LoopBack mode\n");
		rtw_write32(padapter, 0x100, 0x0B0106FF);
		RTW_INFO("0x100 :0x%x", rtw_read32(padapter, 0x100));
		rtw_write16(padapter, 0x608, 0x30c);
		RTW_INFO("0x608 :0x%x", rtw_read32(padapter, 0x608));
	}

	wrqu->length = strlen(extra) + 1;

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}


int rtw_mp_trx_query(struct net_device *dev,
		     struct iw_request_info *info,
		     struct iw_point *wrqu, char *extra)
{
	u32 txok, txfail, rxok, rxfail, rxfilterout;
	PADAPTER padapter = rtw_netdev_priv(dev);
	PMPT_CONTEXT	pMptCtx		=	&(padapter->mppriv.mpt_ctx);
	RT_PMAC_TX_INFO	PMacTxInfo	=	pMptCtx->PMacTxInfo;

	if (PMacTxInfo.bEnPMacTx == TRUE)
		txok = hal_mpt_query_phytxok(padapter);
	else
		txok = padapter->mppriv.tx.sended;

	txfail = 0;
	rxok = padapter->mppriv.rx_pktcount;
	rxfail = padapter->mppriv.rx_crcerrpktcount;
	rxfilterout = padapter->mppriv.rx_pktcount_filter_out;

	_rtw_memset(extra, '\0', 128);

	sprintf(extra, "Tx OK:%d, Tx Fail:%d, Rx OK:%d, CRC error:%d ,Rx Filter out:%d\n", txok, txfail, rxok, rxfail, rxfilterout);

	wrqu->length = strlen(extra) + 1;

	return 0;
}


int rtw_mp_pwrtrk(struct net_device *dev,
		  struct iw_request_info *info,
		  struct iw_point *wrqu, char *extra)
{
	u8 enable;
	u32 thermal;
	s32 res;
	PADAPTER padapter = rtw_netdev_priv(dev);
	u8 input[RTW_IWD_MAX_LEN];
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	_rtw_memset(extra, 0, wrqu->length);

	enable = 1;
	if (wrqu->length > 1) {
		/* not empty string*/
		if (strncmp(input, "stop", 4) == 0) {
			enable = 0;
			sprintf(extra, "mp tx power tracking stop");
		} else if (sscanf(input, "ther=%d", &thermal) == 1) {
			res = SetThermalMeter(padapter, (u8)thermal);
			if (res == _FAIL) {
				ret = -EPERM;
				goto exit;
			}
			sprintf(extra, "mp tx power tracking start,target value=%d ok", thermal);
		} else {
			ret = -EINVAL;
			goto exit;
		}
	}

	res = SetPowerTracking(padapter, enable);
	if (res == _FAIL) {
		ret = -EPERM;
		goto exit;
	}

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}



int rtw_mp_psd(struct net_device *dev,
	       struct iw_request_info *info,
	       struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	u8 input[RTW_IWD_MAX_LEN];
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, (wrqu->length + 1)))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	strcpy(extra, input);

	wrqu->length = mp_query_psd(padapter, extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}


int rtw_mp_thermal(struct net_device *dev,
		   struct iw_request_info *info,
		   struct iw_point *wrqu, char *extra)
{
	u8 val[4] = {0};
	u8 ret = 0;
	u16 ther_path_addr[4] = {0};
	u16 cnt = 1;
	PADAPTER padapter = rtw_netdev_priv(dev);
	int rfpath = RF_PATH_A;

#ifdef CONFIG_RTL8703B
	ther_path_addr[0] = EEPROM_THERMAL_METER_8703B;
#endif
	ther_path_addr[0] = EEPROM_THERMAL_METER_8822B;

	if (copy_from_user(extra, wrqu->pointer, wrqu->length))
		return -EFAULT;

	if ((strncmp(extra, "write", 6) == 0)) {
		int i;
		u16 raw_cursize = 0, raw_maxsize = 0;
		raw_maxsize = efuse_GetavailableSize(padapter);
		RTW_INFO("[eFuse available raw size]= %d bytes\n", raw_maxsize - raw_cursize);
		if (2 > raw_maxsize - raw_cursize) {
			RTW_INFO("no available efuse!\n");
			return -EFAULT;
		}

		for (i = 0; i < GET_HAL_RFPATH_NUM(padapter); i++) {
				GetThermalMeter(padapter, i , &val[i]);
				if (ther_path_addr[i] != 0 && val[i] != 0) {
					if (rtw_efuse_map_write(padapter, ther_path_addr[i], cnt, &val[i]) == _FAIL) {
						RTW_INFO("Error efuse write thermal addr 0x%x ,val = 0x%x\n", ther_path_addr[i], val[i]);
						return -EFAULT;
					}
				} else {
						RTW_INFO("Error efuse write thermal Null addr,val \n");
						return -EFAULT;
				}
		}
		_rtw_memset(extra, 0, wrqu->length);
		sprintf(extra, " efuse write ok :%d", val[0]);
	} else {
		ret = sscanf(extra, "%d", &rfpath);
		if (ret < 1) {
			rfpath = RF_PATH_A;
			RTW_INFO("default thermal of path(%d)\n", rfpath);
		}
		if (rfpath >= GET_HAL_RFPATH_NUM(padapter))
			return -EINVAL;

		RTW_INFO("read thermal of path(%d)\n", rfpath);
		GetThermalMeter(padapter, rfpath, &val[0]);

		_rtw_memset(extra, 0, wrqu->length);
		sprintf(extra, "%d", val[0]);
	}
	wrqu->length = strlen(extra);

	return 0;
}



int rtw_mp_reset_stats(struct net_device *dev,
		       struct iw_request_info *info,
		       struct iw_point *wrqu, char *extra)
{
	struct mp_priv *pmp_priv;
	PADAPTER padapter = rtw_netdev_priv(dev);

	pmp_priv = &padapter->mppriv;

	pmp_priv->tx.sended = 0;
	pmp_priv->tx_pktcount = 0;
	pmp_priv->rx_pktcount = 0;
	pmp_priv->rx_pktcount_filter_out = 0;
	pmp_priv->rx_crcerrpktcount = 0;

	rtw_reset_phy_rx_counters(padapter);
	rtw_reset_mac_rx_counters(padapter);

	_rtw_memset(extra, 0, wrqu->length);
	sprintf(extra, "mp_reset_stats ok\n");
	wrqu->length = strlen(extra);

	return 0;
}


int rtw_mp_dump(struct net_device *dev,
		struct iw_request_info *info,
		struct iw_point *wrqu, char *extra)
{
	struct mp_priv *pmp_priv;
	u8 input[RTW_IWD_MAX_LEN];
	PADAPTER padapter = rtw_netdev_priv(dev);
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	pmp_priv = &padapter->mppriv;

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	if (strncmp(input, "all", 4) == 0) {
		mac_reg_dump(RTW_DBGDUMP, padapter);
		bb_reg_dump(RTW_DBGDUMP, padapter);
		rf_reg_dump(RTW_DBGDUMP, padapter);
	}

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}


int rtw_mp_phypara(struct net_device *dev,
		   struct iw_request_info *info,
		   struct iw_point *wrqu, char *extra)
{

	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	char input[RTW_IWD_MAX_LEN];
	u32		invalxcap = 0, cnt = 0, bwrite_xcap = 0, hwxtaladdr = 0;
	u16		pgval;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	RTW_INFO("%s:priv in=%s\n", __func__, input);
	bwrite_xcap = (strncmp(input, "write_xcap=", 11) == 0) ? 1 : 0;

	if (bwrite_xcap == 1) {
		cnt = sscanf(input, "write_xcap=%d", &invalxcap);
		invalxcap = invalxcap & 0x7f; /* xtal bit 0 ~6 */
		RTW_INFO("get crystal_cap %d\n", invalxcap);

		if (IS_HARDWARE_TYPE_8822C(padapter) && cnt == 1) {
			hwxtaladdr = 0x110;
			pgval = invalxcap | 0x80; /* reserved default bit7 on */
			pgval = pgval | pgval << 8; /* xtal xi/xo efuse 0x110 0x111 */

			RTW_INFO("Get crystal_cap 0x%x\n", pgval);
			if (rtw_efuse_map_write(padapter, hwxtaladdr, 2, (u8*)&pgval) == _FAIL) {
					RTW_INFO("%s: rtw_efuse_map_write xcap error!!\n", __func__);
					sprintf(extra, "write xcap pgdata fail");
					ret = -EFAULT;
			} else
					sprintf(extra, "write xcap pgdata ok");

		}
	} else {
		cnt = sscanf(input, "xcap=%d", &invalxcap);

		if (cnt == 1) {
			pHalData->crystal_cap = (u8)invalxcap;
			RTW_INFO("%s:crystal_cap=%d\n", __func__, pHalData->crystal_cap);

			if (rtw_phydm_set_crystal_cap(padapter, pHalData->crystal_cap) == _FALSE) {
				RTW_ERR("set crystal_cap failed\n");
				rtw_warn_on(1);
			}
			sprintf(extra, "Set xcap=%d", invalxcap);
		}
	}

	wrqu->length = strlen(extra) + 1;

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}


int rtw_mp_SetRFPath(struct net_device *dev,
		     struct iw_request_info *info,
		     struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	char input[RTW_IWD_MAX_LEN];
	int		bMain = 1, bTurnoff = 1;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}
	RTW_INFO("%s:iwpriv in=%s\n", __func__, input);

	bMain = strncmp(input, "1", 2); /* strncmp TRUE is 0*/
	bTurnoff = strncmp(input, "0", 3); /* strncmp TRUE is 0*/

	_rtw_memset(extra, 0, wrqu->length);

	if (bMain == 0) {
		MP_PHY_SetRFPathSwitch(padapter, _TRUE);
		RTW_INFO("%s:PHY_SetRFPathSwitch=TRUE\n", __func__);
		sprintf(extra, "mp_setrfpath Main\n");

	} else if (bTurnoff == 0) {
		MP_PHY_SetRFPathSwitch(padapter, _FALSE);
		RTW_INFO("%s:PHY_SetRFPathSwitch=FALSE\n", __func__);
		sprintf(extra, "mp_setrfpath Aux\n");
	} else {
		bMain = MP_PHY_QueryRFPathSwitch(padapter);
		RTW_INFO("%s:PHY_SetRFPathSwitch = %s\n", __func__, (bMain ? "Main":"Aux"));
		sprintf(extra, "mp_setrfpath %s\n" , (bMain ? "Main":"Aux"));
	}

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}


int rtw_mp_switch_rf_path(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmp_priv;
	char input[RTW_IWD_MAX_LEN];
	int		bwlg = 1, bwla = 1, btg = 1, bbt=1;
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length + 1))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	pmp_priv = &padapter->mppriv;

	RTW_INFO("%s: in=%s\n", __func__, input);

	bwlg = strncmp(input, "WLG", 3); /* strncmp TRUE is 0*/
	bwla = strncmp(input, "WLA", 3); /* strncmp TRUE is 0*/
	btg = strncmp(input, "BTG", 3); /* strncmp TRUE is 0*/
	bbt = strncmp(input, "BT", 3); /* strncmp TRUE is 0*/

	_rtw_memset(extra, 0, wrqu->length);

	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length + 1);

	return ret;
}
int rtw_mp_QueryDrv(struct net_device *dev,
		    struct iw_request_info *info,
		    union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	char input[RTW_IWD_MAX_LEN];
	int	qAutoLoad = 1;
	int ret = 0;

	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->data.length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->data.pointer, wrqu->data.length)) {
		ret = -EFAULT;
		goto exit;
	}
	RTW_INFO("%s:iwpriv in=%s\n", __func__, input);

	qAutoLoad = strncmp(input, "autoload", 8); /* strncmp TRUE is 0*/

	if (qAutoLoad == 0) {
		RTW_INFO("%s:qAutoLoad\n", __func__);

		if (pHalData->bautoload_fail_flag)
			sprintf(extra, "fail");
		else
			sprintf(extra, "ok");
	}
	wrqu->data.length = strlen(extra) + 1;

exit:
	rtw_mfree(input, wrqu->data.length);

	return ret;
}


int rtw_mp_PwrCtlDM(struct net_device *dev,
		    struct iw_request_info *info,
		    struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	u8 input[RTW_IWD_MAX_LEN];
	u8		pwrtrk_state = 0;
	u8		pwtk_type[5][25] = {"Thermal tracking off","Thermal tracking on",
					"TSSI tracking off","TSSI tracking on","TSSI calibration"};
	int ret = 0;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	input[wrqu->length] = '\0';
	RTW_INFO("%s: in=%s\n", __func__, input);

	if (wrqu->length == 2) {
		if(input[0] >= '0' && input[0] <= '4') {
			pwrtrk_state = rtw_atoi(input);
			MPT_PwrCtlDM(padapter, pwrtrk_state);
			sprintf(extra, "PwrCtlDM start %s\n" , pwtk_type[pwrtrk_state]);
		} else {
			sprintf(extra, "Error unknown number ! Please check your input number\n"
				" 0 : Thermal tracking off\n 1 : Thermal tracking on\n 2 : TSSI tracking off\n"
				" 3 : TSSI tracking on\n 4 : TSSI calibration\n");
		}
		wrqu->length = strlen(extra);

		goto exit;
	}
	if (strncmp(input, "start", 5) == 0 || strncmp(input, "thertrk on", 10) == 0) {/* strncmp TRUE is 0*/
		pwrtrk_state = 1;
		sprintf(extra, "PwrCtlDM start %s\n" , pwtk_type[pwrtrk_state]);
	} else if (strncmp(input, "thertrk off", 11) == 0 || strncmp(input, "stop", 5) == 0) {
		pwrtrk_state = 0;
		sprintf(extra, "PwrCtlDM stop %s\n" , pwtk_type[pwrtrk_state]);
	} else if (strncmp(input, "tssitrk off", 11) == 0){
		pwrtrk_state = 2;
		sprintf(extra, "PwrCtlDM stop %s\n" , pwtk_type[pwrtrk_state]);
	} else if (strncmp(input, "tssitrk on", 10) == 0){
		pwrtrk_state = 3;
		sprintf(extra, "PwrCtlDM start %s\n" , pwtk_type[pwrtrk_state]);
	} else if (strncmp(input, "tssik", 5) == 0){
		pwrtrk_state = 4;
		sprintf(extra, "PwrCtlDM start %s\n" , pwtk_type[pwrtrk_state]);
	} else {
		pwrtrk_state = 0;
		sprintf(extra, "Error input, default PwrCtlDM stop\n"
			" thertrk off : Thermal tracking off\n thertrk on : Thermal tracking on\n"
			" tssitrk off : TSSI tracking off\n tssitrk on : TSSI tracking on\n tssik : TSSI calibration\n\n"
			" 0 : Thermal tracking off\n 1 : Thermal tracking on\n 2 : TSSI tracking off\n"
			" 3 : TSSI tracking on\n 4 : TSSI calibration\n");
	}

	MPT_PwrCtlDM(padapter, pwrtrk_state);
	wrqu->length = strlen(extra);

exit:
	rtw_mfree(input, wrqu->length);

	return ret;
}

int rtw_mp_iqk(struct net_device *dev,
		 struct iw_request_info *info,
		 struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);

	rtw_mp_trigger_iqk(padapter);

	return 0;
}

int rtw_mp_lck(struct net_device *dev,
		 struct iw_request_info *info,
		 struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);

	rtw_mp_trigger_lck(padapter);

	return 0;
}

int rtw_mp_dpk(struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;
	struct pwrctrl_priv *pwrctrlpriv = adapter_to_pwrctl(padapter);

	u8 ips_mode = IPS_NUM; /* init invalid value */
	u8 lps_mode = PS_MODE_NUM; /* init invalid value */

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	*(extra + wrqu->data.length) = '\0';

	if (strncmp(extra, "off", 3) == 0 && strlen(extra) < 4) {
			pDM_Odm->dpk_info.is_dpk_enable = 0;
			halrf_dpk_enable_disable(pDM_Odm);
			sprintf(extra, "set dpk off\n");

	} else if (strncmp(extra, "on", 2) == 0 && strlen(extra) < 3) {
			pDM_Odm->dpk_info.is_dpk_enable = 1;
			halrf_dpk_enable_disable(pDM_Odm);
			sprintf(extra, "set dpk on\n");
	} else	{
			lps_mode = pwrctrlpriv->power_mgnt;/* keep org value */
			rtw_pm_set_lps(padapter, PS_MODE_ACTIVE);
			ips_mode = pwrctrlpriv->ips_mode;/* keep org value */
			rtw_pm_set_ips(padapter, IPS_NONE);
			rtw_mp_trigger_dpk(padapter);
	if (padapter->registrypriv.mp_mode == 0) {
			rtw_pm_set_ips(padapter, ips_mode);

			rtw_pm_set_lps(padapter, lps_mode);
	}
			sprintf(extra, "set dpk trigger\n");
	}

	wrqu->data.length = strlen(extra);

	return 0;
}

int rtw_mp_getver(struct net_device *dev,
		  struct iw_request_info *info,
		  union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmp_priv;

	pmp_priv = &padapter->mppriv;

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	sprintf(extra, "rtwpriv=%d\n", RTWPRIV_VER_INFO);
	wrqu->data.length = strlen(extra);
	return 0;
}


int rtw_mp_mon(struct net_device *dev,
	       struct iw_request_info *info,
	       union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmp_priv = &padapter->mppriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct hal_ops *pHalFunc = &padapter->hal_func;
	NDIS_802_11_NETWORK_INFRASTRUCTURE networkType;
	int bstart = 1, bstop = 1;

	networkType = Ndis802_11Infrastructure;
	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	*(extra + wrqu->data.length) = '\0';
	rtw_pm_set_ips(padapter, IPS_NONE);
	LeaveAllPowerSaveMode(padapter);

	if (init_mp_priv(padapter) == _FAIL)
		RTW_INFO("%s: initialize MP private data Fail!\n", __func__);
	padapter->mppriv.channel = 6;

	bstart = strncmp(extra, "start", 5); /* strncmp TRUE is 0*/
	bstop = strncmp(extra, "stop", 4); /* strncmp TRUE is 0*/
	if (bstart == 0) {
		mp_join(padapter, WIFI_FW_ADHOC_STATE);
		SetPacketRx(padapter, _TRUE, _FALSE);
		SetChannel(padapter);
		pmp_priv->rx_bindicatePkt = _TRUE;
		pmp_priv->bRTWSmbCfg = _TRUE;
		sprintf(extra, "monitor mode start\n");
	} else if (bstop == 0) {
		SetPacketRx(padapter, _FALSE, _FALSE);
		pmp_priv->rx_bindicatePkt = _FALSE;
		pmp_priv->bRTWSmbCfg = _FALSE;
		padapter->registrypriv.mp_mode = 1;
		pHalFunc->hal_deinit(padapter);
		padapter->registrypriv.mp_mode = 0;
		pHalFunc->hal_init(padapter);
		/*rtw_disassoc_cmd(padapter, 0, 0);*/
		if (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE) {
			rtw_disassoc_cmd(padapter, 500, 0);
			rtw_indicate_disconnect(padapter, 0, _FALSE);
			/*rtw_free_assoc_resources_cmd(padapter, _TRUE, 0);*/
		}
		rtw_pm_set_ips(padapter, IPS_NORMAL);
		sprintf(extra, "monitor mode Stop\n");
	}
	wrqu->data.length = strlen(extra);
	return 0;
}

int rtw_mp_pretx_proc(PADAPTER padapter, u8 bStartTest, char *extra)
{
	struct mp_priv *pmp_priv = &padapter->mppriv;
	char *pextra = extra;

	switch (pmp_priv->mode) {

	case MP_PACKET_TX:
		if (bStartTest == 0) {
			pmp_priv->tx.stop = 1;
			pmp_priv->mode = MP_ON;
			rtw_write8(padapter, 0x838, 0x61);
			sprintf(extra, "Stop continuous Tx");
		} else if (pmp_priv->tx.stop == 1) {
			pextra = extra + strlen(extra);
			pextra += sprintf(pextra, "\nStart continuous DA=ffffffffffff len=1500 count=%u\n", pmp_priv->tx.count);
			pmp_priv->tx.stop = 0;
			rtw_write8(padapter, 0x838, 0x6d);
			SetPacketTx(padapter);
		} else
			return -EFAULT;
		return 0;
	case MP_SINGLE_TONE_TX:
		if (bStartTest != 0)
			strcat(extra, "\nStart continuous DA=ffffffffffff len=1500\n infinite=yes.");
		SetSingleToneTx(padapter, (u8)bStartTest);
		break;
	case MP_CONTINUOUS_TX:
		if (bStartTest != 0)
			strcat(extra, "\nStart continuous DA=ffffffffffff len=1500\n infinite=yes.");
		SetContinuousTx(padapter, (u8)bStartTest);
		break;
	case MP_CARRIER_SUPPRISSION_TX:
		if (bStartTest != 0) {
			if (HwRateToMPTRate(pmp_priv->rateidx) <= MPT_RATE_11M)
				strcat(extra, "\nStart continuous DA=ffffffffffff len=1500\n infinite=yes.");
			else
				strcat(extra, "\nSpecify carrier suppression but not CCK rate");
		}
		SetCarrierSuppressionTx(padapter, (u8)bStartTest);
		break;
	case MP_SINGLE_CARRIER_TX:
		if (bStartTest != 0)
			strcat(extra, "\nStart continuous DA=ffffffffffff len=1500\n infinite=yes.");
		SetSingleCarrierTx(padapter, (u8)bStartTest);
		break;

	default:
		sprintf(extra, "Error! Continuous-Tx is not on-going.");
		return -EFAULT;
	}

	if (bStartTest == 1 && pmp_priv->mode != MP_ON) {
		struct mp_priv *pmp_priv = &padapter->mppriv;

		if (pmp_priv->tx.stop == 0) {
			pmp_priv->tx.stop = 1;
			rtw_msleep_os(5);
		}
		if(padapter->registrypriv.ht_enable &&
			is_supported_ht(padapter->registrypriv.wireless_mode))
			pmp_priv->tx.attrib.ht_en = 1;
		pmp_priv->tx.stop = 0;
		pmp_priv->tx.count = 1;
		SetPacketTx(padapter);
	} else
		pmp_priv->mode = MP_ON;


	return 0;
}


int rtw_mp_tx(struct net_device *dev,
	      struct iw_request_info *info,
	      union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	struct mp_priv *pmp_priv = &padapter->mppriv;
	PMPT_CONTEXT		pMptCtx = &(padapter->mppriv.mpt_ctx);
	char *pextra = extra;
	u32 bandwidth = 0, sg = 0, channel = 6, txpower = 40, rate = 108, ant = 0, txmode = 1, count = 0;
	u8 bStartTest = 1, status = 0;
	u16 antenna = 0;

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;
	RTW_INFO("extra = %s\n", extra);
#ifdef CONFIG_CONCURRENT_MODE
	if (!is_primary_adapter(padapter)) {
		sprintf(extra, "Error: MP mode can't support Virtual Adapter, Please to use main Adapter.\n");
		wrqu->data.length = strlen(extra);
		return 0;
	}
#endif

	if (strncmp(extra, "stop", 3) == 0) {
		bStartTest = 0; /* To set Stop*/
		pmp_priv->tx.stop = 1;
		sprintf(extra, "Stop continuous Tx");
		status = rtw_mp_pretx_proc(padapter, bStartTest, extra);
		wrqu->data.length = strlen(extra);
		return status;
	} else if (strncmp(extra, "count", 5) == 0) {
		if (sscanf(extra, "count=%d", &count) < 1)
			RTW_INFO("Got Count=%d]\n", count);
		pmp_priv->tx.count = count;
		return 0;
	} else if (strncmp(extra, "setting", 7) == 0) {
		_rtw_memset(extra, 0, wrqu->data.length);
		pextra += sprintf(pextra, "Current Setting :\n Channel:%d", pmp_priv->channel);
		pextra += sprintf(pextra, "\n Bandwidth:%d", pmp_priv->bandwidth);
		pextra += sprintf(pextra, "\n Rate index:%d", pmp_priv->rateidx);
		pextra += sprintf(pextra, "\n TxPower index:%d", pmp_priv->txpoweridx);
		pextra += sprintf(pextra, "\n Antenna TxPath:%d", pmp_priv->antenna_tx);
		pextra += sprintf(pextra, "\n Antenna RxPath:%d", pmp_priv->antenna_rx);
		pextra += sprintf(pextra, "\n MP Mode:%d", pmp_priv->mode);
		wrqu->data.length = strlen(extra);
		return 0;
	} else {

		if (sscanf(extra, "ch=%d,bw=%d,rate=%d,pwr=%d,ant=%d,tx=%d", &channel, &bandwidth, &rate, &txpower, &ant, &txmode) < 6) {
			RTW_INFO("Invalid format [ch=%d,bw=%d,rate=%d,pwr=%d,ant=%d,tx=%d]\n", channel, bandwidth, rate, txpower, ant, txmode);
			_rtw_memset(extra, 0, wrqu->data.length);
			pextra += sprintf(pextra, "\n Please input correct format as bleow:\n");
			pextra += sprintf(pextra, "\t ch=%d,bw=%d,rate=%d,pwr=%d,ant=%d,tx=%d\n", channel, bandwidth, rate, txpower, ant, txmode);
			pextra += sprintf(pextra, "\n [ ch : BGN = <1~14> , A or AC = <36~165> ]");
			pextra += sprintf(pextra, "\n [ bw : Bandwidth: 0 = 20M, 1 = 40M, 2 = 80M ]");
			pextra += sprintf(pextra, "\n [ rate :	CCK: 1 2 5.5 11M X 2 = < 2 4 11 22 >]");
			pextra += sprintf(pextra, "\n [		OFDM: 6 9 12 18 24 36 48 54M X 2 = < 12 18 24 36 48 72 96 108>");
			pextra += sprintf(pextra, "\n [		HT 1S2SS MCS0 ~ MCS15 : < [MCS0]=128 ~ [MCS7]=135 ~ [MCS15]=143 >");
			pextra += sprintf(pextra, "\n [		HT 3SS MCS16 ~ MCS32 : < [MCS16]=144 ~ [MCS23]=151 ~ [MCS32]=159 >");
			pextra += sprintf(pextra, "\n [		VHT 1SS MCS0 ~ MCS9 : < [MCS0]=160 ~ [MCS9]=169 >");
			pextra += sprintf(pextra, "\n [ txpower : 1~63 power index");
			pextra += sprintf(pextra, "\n [ ant : <A = 1, B = 2, C = 4, D = 8> ,2T ex: AB=3 BC=6 CD=12");
			pextra += sprintf(pextra, "\n [ txmode : < 0 = CONTINUOUS_TX, 1 = PACKET_TX, 2 = SINGLE_TONE_TX, 3 = CARRIER_SUPPRISSION_TX, 4 = SINGLE_CARRIER_TX>\n");
			wrqu->data.length = strlen(extra);
			return status;

		} else {
			char *pextra = extra;
			RTW_INFO("Got format [ch=%d,bw=%d,rate=%d,pwr=%d,ant=%d,tx=%d]\n", channel, bandwidth, rate, txpower, ant, txmode);
			_rtw_memset(extra, 0, wrqu->data.length);
			sprintf(extra, "Change Current channel %d to channel %d", padapter->mppriv.channel , channel);
			padapter->mppriv.channel = channel;
			SetChannel(padapter);
			pHalData->current_channel = channel;

			if (bandwidth == 1)
				bandwidth = CHANNEL_WIDTH_40;
			else if (bandwidth == 2)
				bandwidth = CHANNEL_WIDTH_80;
			pextra = extra + strlen(pextra);
			pextra += sprintf(pextra, "\nChange Current Bandwidth %d to Bandwidth %d", padapter->mppriv.bandwidth, bandwidth);
			padapter->mppriv.bandwidth = (u8)bandwidth;
			padapter->mppriv.preamble = sg;
			SetBandwidth(padapter);
			pHalData->current_channel_bw = bandwidth;

			pextra += sprintf(pextra, "\nSet power level :%d", txpower);
			padapter->mppriv.txpoweridx = (u8)txpower;
			pMptCtx->TxPwrLevel[RF_PATH_A] = (u8)txpower;
			pMptCtx->TxPwrLevel[RF_PATH_B] = (u8)txpower;
			pMptCtx->TxPwrLevel[RF_PATH_C] = (u8)txpower;
			pMptCtx->TxPwrLevel[RF_PATH_D]  = (u8)txpower;
			SetTxPower(padapter);

			RTW_INFO("%s: bw=%d sg=%d\n", __func__, bandwidth, sg);

			if (rate <= 0x7f)
				rate = wifirate2_ratetbl_inx((u8)rate);
			else if (rate < 0xC8)
				rate = (rate - 0x80 + MPT_RATE_MCS0);
			/*HT  rate 0x80(MCS0)  ~ 0x8F(MCS15) ~ 0x9F(MCS31) 128~159
			VHT1SS~2SS rate 0xA0 (VHT1SS_MCS0 44) ~ 0xB3 (VHT2SS_MCS9 #63) 160~179
			VHT rate 0xB4 (VHT3SS_MCS0 64) ~ 0xC7 (VHT2SS_MCS9 #83) 180~199
			else
			VHT rate 0x90(VHT1SS_MCS0) ~ 0x99(VHT1SS_MCS9) 144~153
			rate =(rate - MPT_RATE_VHT1SS_MCS0);
			*/
			RTW_INFO("%s: rate index=%d\n", __func__, rate);
			if (rate >= MPT_RATE_LAST)
				return -EINVAL;
			pextra += sprintf(pextra, "\nSet data rate to %d index %d", padapter->mppriv.rateidx, rate);

			padapter->mppriv.rateidx = rate;
			pMptCtx->mpt_rate_index = rate;
			SetDataRate(padapter);

			pextra += sprintf(pextra, "\nSet Antenna Path :%d", ant);
			switch (ant) {
			case 1:
				antenna = ANTENNA_A;
				break;
			case 2:
				antenna = ANTENNA_B;
				break;
			case 4:
				antenna = ANTENNA_C;
				break;
			case 8:
				antenna = ANTENNA_D;
				break;
			case 3:
				antenna = ANTENNA_AB;
				break;
			case 5:
				antenna = ANTENNA_AC;
				break;
			case 9:
				antenna = ANTENNA_AD;
				break;
			case 6:
				antenna = ANTENNA_BC;
				break;
			case 10:
				antenna = ANTENNA_BD;
				break;
			case 12:
				antenna = ANTENNA_CD;
				break;
			case 7:
				antenna = ANTENNA_ABC;
				break;
			case 14:
				antenna = ANTENNA_BCD;
				break;
			case 11:
				antenna = ANTENNA_ABD;
				break;
			case 15:
				antenna = ANTENNA_ABCD;
				break;
			}
			RTW_INFO("%s: antenna=0x%x\n", __func__, antenna);
			padapter->mppriv.antenna_tx = antenna;
			padapter->mppriv.antenna_rx = antenna;
			pHalData->antenna_tx_path = antenna;
			SetAntenna(padapter);

			if (txmode == 0)
				pmp_priv->mode = MP_CONTINUOUS_TX;
			else if (txmode == 1) {
				pmp_priv->mode = MP_PACKET_TX;
				pmp_priv->tx.count = count;
			} else if (txmode == 2)
				pmp_priv->mode = MP_SINGLE_TONE_TX;
			else if (txmode == 3)
				pmp_priv->mode = MP_CARRIER_SUPPRISSION_TX;
			else if (txmode == 4)
				pmp_priv->mode = MP_SINGLE_CARRIER_TX;

			status = rtw_mp_pretx_proc(padapter, bStartTest, extra);
		}

	}

	wrqu->data.length = strlen(extra);
	return status;
}


int rtw_mp_rx(struct net_device *dev,
	      struct iw_request_info *info,
	      union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	struct mp_priv *pmp_priv = &padapter->mppriv;
	char *pextra = extra;
	u32 bandwidth = 0, sg = 0, channel = 6, ant = 0;
	u16 antenna = 0;
	u8 bStartRx = 0;

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

#ifdef CONFIG_CONCURRENT_MODE
	if (!is_primary_adapter(padapter)) {
		sprintf(extra, "Error: MP mode can't support Virtual Adapter, Please to use main Adapter.\n");
		wrqu->data.length = strlen(extra);
		return 0;
	}
#endif

	if (strncmp(extra, "stop", 4) == 0) {
		_rtw_memset(extra, 0, wrqu->data.length);
		SetPacketRx(padapter, bStartRx, _FALSE);
		pmp_priv->bmac_filter = _FALSE;
		sprintf(extra, "Received packet OK:%d CRC error:%d ,Filter out:%d", padapter->mppriv.rx_pktcount, padapter->mppriv.rx_crcerrpktcount, padapter->mppriv.rx_pktcount_filter_out);
		wrqu->data.length = strlen(extra);
		return 0;

	} else if (sscanf(extra, "ch=%d,bw=%d,ant=%d", &channel, &bandwidth, &ant) < 3) {
		RTW_INFO("Invalid format [ch=%d,bw=%d,ant=%d]\n", channel, bandwidth, ant);
		_rtw_memset(extra, 0, wrqu->data.length);
		pextra += sprintf(pextra, "\n Please input correct format as bleow:\n");
		pextra += sprintf(pextra, "\t ch=%d,bw=%d,ant=%d\n", channel, bandwidth, ant);
		pextra += sprintf(pextra, "\n [ ch : BGN = <1~14> , A or AC = <36~165> ]");
		pextra += sprintf(pextra, "\n [ bw : Bandwidth: 0 = 20M, 1 = 40M, 2 = 80M ]");
		pextra += sprintf(pextra, "\n [ ant : <A = 1, B = 2, C = 4, D = 8> ,2T ex: AB=3 BC=6 CD=12");
		wrqu->data.length = strlen(extra);
		return 0;

	} else {
		char *pextra = extra;
		bStartRx = 1;
		RTW_INFO("Got format [ch=%d,bw=%d,ant=%d]\n", channel, bandwidth, ant);
		_rtw_memset(extra, 0, wrqu->data.length);
		sprintf(extra, "Change Current channel %d to channel %d", padapter->mppriv.channel , channel);
		padapter->mppriv.channel = channel;
		SetChannel(padapter);
		pHalData->current_channel = channel;

		if (bandwidth == 1)
			bandwidth = CHANNEL_WIDTH_40;
		else if (bandwidth == 2)
			bandwidth = CHANNEL_WIDTH_80;
		pextra = extra + strlen(extra);
		pextra += sprintf(pextra, "\nChange Current Bandwidth %d to Bandwidth %d", padapter->mppriv.bandwidth, bandwidth);
		padapter->mppriv.bandwidth = (u8)bandwidth;
		padapter->mppriv.preamble = sg;
		SetBandwidth(padapter);
		pHalData->current_channel_bw = bandwidth;

		pextra += sprintf(pextra, "\nSet Antenna Path :%d", ant);
		switch (ant) {
		case 1:
			antenna = ANTENNA_A;
			break;
		case 2:
			antenna = ANTENNA_B;
			break;
		case 4:
			antenna = ANTENNA_C;
			break;
		case 8:
			antenna = ANTENNA_D;
			break;
		case 3:
			antenna = ANTENNA_AB;
			break;
		case 5:
			antenna = ANTENNA_AC;
			break;
		case 9:
			antenna = ANTENNA_AD;
			break;
		case 6:
			antenna = ANTENNA_BC;
			break;
		case 10:
			antenna = ANTENNA_BD;
			break;
		case 12:
			antenna = ANTENNA_CD;
			break;
		case 7:
			antenna = ANTENNA_ABC;
			break;
		case 14:
			antenna = ANTENNA_BCD;
			break;
		case 11:
			antenna = ANTENNA_ABD;
			break;
		case 15:
			antenna = ANTENNA_ABCD;
			break;
		}
		RTW_INFO("%s: antenna=0x%x\n", __func__, antenna);
		padapter->mppriv.antenna_tx = antenna;
		padapter->mppriv.antenna_rx = antenna;
		pHalData->antenna_tx_path = antenna;
		SetAntenna(padapter);

		strcat(extra, "\nstart Rx");
		SetPacketRx(padapter, bStartRx, _FALSE);
	}
	wrqu->data.length = strlen(extra);
	return 0;
}


int rtw_mp_hwtx(struct net_device *dev,
		struct iw_request_info *info,
		union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmp_priv = &padapter->mppriv;
	PMPT_CONTEXT		pMptCtx = &(padapter->mppriv.mpt_ctx);

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;
	*(extra + wrqu->data.length) = '\0';

	_rtw_memset(&pMptCtx->PMacTxInfo, 0, sizeof(RT_PMAC_TX_INFO));
	_rtw_memcpy((void *)&pMptCtx->PMacTxInfo, (void *)extra, sizeof(RT_PMAC_TX_INFO));
	_rtw_memset(extra, 0, wrqu->data.length);

	if (pMptCtx->PMacTxInfo.bEnPMacTx == 1 && pmp_priv->mode != MP_ON) {
		sprintf(extra, "MP Tx Running, Please Set PMac Tx Mode Stop\n");
		RTW_INFO("Error !!! MP Tx Running, Please Set PMac Tx Mode Stop\n");
	} else {
		RTW_INFO("To set MAC Tx mode\n");
		mpt_ProSetPMacTx(padapter);
		sprintf(extra, "Set PMac Tx Mode OK\n");
	}
	wrqu->data.length = strlen(extra);
	return 0;

}

int rtw_mp_pwrlmt(struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct registry_priv  *registry_par = &padapter->registrypriv;
	u8 pwrlimtstat = 0;

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	*(extra + wrqu->data.length) = '\0';
#if CONFIG_TXPWR_LIMIT
	pwrlimtstat = registry_par->RegEnableTxPowerLimit;
	if (strncmp(extra, "off", 3) == 0 && strlen(extra) < 4) {
		padapter->registrypriv.RegEnableTxPowerLimit = 0;
		sprintf(extra, "Turn off Power Limit\n");

	} else if (strncmp(extra, "on", 2) == 0 && strlen(extra) < 3) {
		padapter->registrypriv.RegEnableTxPowerLimit = 1;
		sprintf(extra, "Turn on Power Limit\n");

	} else
#endif
		sprintf(extra, "Get Power Limit Status:%s\n", (pwrlimtstat == 1) ? "ON" : "OFF");


	wrqu->data.length = strlen(extra);
	return 0;
}

int rtw_mp_pwrbyrate(struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	*(extra + wrqu->data.length) = '\0';
	if (strncmp(extra, "off", 3) == 0 && strlen(extra) < 4) {
		padapter->registrypriv.RegEnableTxPowerByRate = 0;
		sprintf(extra, "Turn off Tx Power by Rate\n");

	} else if (strncmp(extra, "on", 2) == 0 && strlen(extra) < 3) {
		padapter->registrypriv.RegEnableTxPowerByRate = 1;
		sprintf(extra, "Turn On Tx Power by Rate\n");

	} else {
		sprintf(extra, "Get Power by Rate Status:%s\n", (padapter->registrypriv.RegEnableTxPowerByRate == 1) ? "ON" : "OFF");
	}

	wrqu->data.length = strlen(extra);
	return 0;
}


int rtw_mp_dpk_track(struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_struct		*pDM_Odm = &pHalData->odmpriv;


	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	*(extra + wrqu->data.length) = '\0';

	if (strncmp(extra, "off", 3) == 0 && strlen(extra) < 4) {
		halrf_set_dpk_track(pDM_Odm, FALSE);
		sprintf(extra, "set dpk track off\n");

	} else if (strncmp(extra, "on", 2) == 0 && strlen(extra) < 3) {
		halrf_set_dpk_track(pDM_Odm, TRUE);
		sprintf(extra, "set dpk track on\n");
	}

	wrqu->data.length = strlen(extra);
	return 0;
}


int rtw_bt_efuse_mask_file(struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	char *rtw_efuse_mask_file_path;
	u8 Status;
	PADAPTER padapter = rtw_netdev_priv(dev);

	_rtw_memset(btmaskfileBuffer, 0x00, sizeof(btmaskfileBuffer));

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	*(extra + wrqu->data.length) = '\0';

	if (strncmp(extra, "data,", 5) == 0) {
		u8	*pch;
		char	*ptmp, tmp;
		u8	count = 0;
		u8	i = 0;

		ptmp = extra;
		pch = strsep(&ptmp, ",");

		if ((pch == NULL) || (strlen(pch) == 0)) {
			RTW_INFO("%s: parameter error(no cmd)!\n", __func__);
			return -EFAULT;
		}

		do {
			pch = strsep(&ptmp, ":");
			if ((pch == NULL) || (strlen(pch) == 0))
				break;
			if (strlen(pch) != 2
				|| IsHexDigit(*pch) == _FALSE
				|| IsHexDigit(*(pch + 1)) == _FALSE
				|| sscanf(pch, "%hhx", &tmp) != 1
			) {
				RTW_INFO("%s: invalid 8-bit hex! input format: data,01:23:45:67:89:ab:cd:ef...\n", __func__);
				return -EFAULT;
			}
			btmaskfileBuffer[count++] = tmp;

		 } while (count < 64);

		_rtw_memset(extra, '\0' , strlen(extra));

		for (i = 0; i < count; i++)
			ptmp += sprintf(ptmp, "%02x:", btmaskfileBuffer[i]);

		padapter->registrypriv.bBTFileMaskEfuse = _TRUE;

		sprintf(ptmp, "\nLoad BT Efuse Mask data %d hex ok\n", count);
		wrqu->data.length = strlen(extra);
		return 0;
	}
	rtw_efuse_mask_file_path = extra;

	if (rtw_is_file_readable(rtw_efuse_mask_file_path) == _TRUE) {
		RTW_INFO("%s do rtw_is_file_readable = %s! ,sizeof BT maskfileBuffer %zu\n", __func__, rtw_efuse_mask_file_path, sizeof(btmaskfileBuffer));
		Status = rtw_efuse_file_read(padapter, rtw_efuse_mask_file_path, btmaskfileBuffer, sizeof(btmaskfileBuffer));
		if (Status == _TRUE) {
			padapter->registrypriv.bBTFileMaskEfuse = _TRUE;
			sprintf(extra, "BT efuse mask file read OK\n");
		} else {
			padapter->registrypriv.bBTFileMaskEfuse = _FALSE;
			sprintf(extra, "read BT efuse mask file FAIL\n");
			RTW_INFO("%s rtw_efuse_file_read BT mask fail!\n", __func__);
		}
	} else {
		padapter->registrypriv.bBTFileMaskEfuse = _FALSE;
		sprintf(extra, "BT efuse mask file readable FAIL\n");
		RTW_INFO("%s rtw_is_file_readable BT Mask file fail!\n", __func__);
	}
	wrqu->data.length = strlen(extra);
	return 0;
}


int rtw_efuse_mask_file(struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	char *rtw_efuse_mask_file_path;
	u8 Status;
	PADAPTER padapter = rtw_netdev_priv(dev);

	_rtw_memset(maskfileBuffer, 0x00, sizeof(maskfileBuffer));

	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	*(extra + wrqu->data.length) = '\0';
	if (strncmp(extra, "off", 3) == 0 && strlen(extra) < 4) {
		padapter->registrypriv.boffefusemask = 1;
		sprintf(extra, "Turn off Efuse Mask\n");
		wrqu->data.length = strlen(extra);
		return 0;
	}
	if (strncmp(extra, "on", 2) == 0 && strlen(extra) < 3) {
		padapter->registrypriv.boffefusemask = 0;
		sprintf(extra, "Turn on Efuse Mask\n");
		wrqu->data.length = strlen(extra);
		return 0;
	}
	if (strncmp(extra, "data,", 5) == 0) {
		u8	*pch;
		char	*ptmp, tmp;
		u8	count = 0;
		u8	i = 0;

		ptmp = extra;
		pch = strsep(&ptmp, ",");

		if ((pch == NULL) || (strlen(pch) == 0)) {
			RTW_INFO("%s: parameter error(no cmd)!\n", __func__);
			return -EFAULT;
		}

		do {
			pch = strsep(&ptmp, ":");
			if ((pch == NULL) || (strlen(pch) == 0))
				break;
			if (strlen(pch) != 2
				|| IsHexDigit(*pch) == _FALSE
				|| IsHexDigit(*(pch + 1)) == _FALSE
				|| sscanf(pch, "%hhx", &tmp) != 1
			) {
				RTW_INFO("%s: invalid 8-bit hex! input format: data,01:23:45:67:89:ab:cd:ef...\n", __func__);
				return -EFAULT;
			}
			maskfileBuffer[count++] = tmp;

		} while (count < 64);

		_rtw_memset(extra, '\0' , strlen(extra));

		for (i = 0; i < count; i++)
			ptmp += sprintf(ptmp, "%02x:", maskfileBuffer[i]);

		padapter->registrypriv.bFileMaskEfuse = _TRUE;

		sprintf(ptmp, "\nLoad Efuse Mask data %d hex ok\n", count);
		wrqu->data.length = strlen(extra);
		return 0;
	}
	rtw_efuse_mask_file_path = extra;

	if (rtw_is_file_readable(rtw_efuse_mask_file_path) == _TRUE) {
		RTW_INFO("%s do rtw_efuse_mask_file_read = %s! ,sizeof maskfileBuffer %zu\n", __func__, rtw_efuse_mask_file_path, sizeof(maskfileBuffer));
		Status = rtw_efuse_file_read(padapter, rtw_efuse_mask_file_path, maskfileBuffer, sizeof(maskfileBuffer));
		if (Status == _TRUE) {
			padapter->registrypriv.bFileMaskEfuse = _TRUE;
			sprintf(extra, "efuse mask file read OK\n");
		} else {
			padapter->registrypriv.bFileMaskEfuse = _FALSE;
			sprintf(extra, "read efuse mask file FAIL\n");
			RTW_INFO("%s rtw_efuse_file_read mask fail!\n", __func__);
		}
	} else {
		padapter->registrypriv.bFileMaskEfuse = _FALSE;
		sprintf(extra, "efuse mask file readable FAIL\n");
		RTW_INFO("%s rtw_is_file_readable fail!\n", __func__);
	}
	wrqu->data.length = strlen(extra);
	return 0;
}


int rtw_efuse_file_map(struct net_device *dev,
		       struct iw_request_info *info,
		       union iwreq_data *wrqu, char *extra)
{
	char *rtw_efuse_file_map_path;
	u8 Status;
	PEFUSE_HAL pEfuseHal;
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct mp_priv *pmp_priv = &padapter->mppriv;

	pEfuseHal = &pHalData->EfuseHal;
	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	rtw_efuse_file_map_path = extra;

	_rtw_memset(pEfuseHal->fakeEfuseModifiedMap, 0xFF, EFUSE_MAX_MAP_LEN);

	if (rtw_is_file_readable(rtw_efuse_file_map_path) == _TRUE) {
		RTW_INFO("%s do rtw_efuse_mask_file_read = %s!\n", __func__, rtw_efuse_file_map_path);
		Status = rtw_efuse_file_read(padapter, rtw_efuse_file_map_path, pEfuseHal->fakeEfuseModifiedMap, sizeof(pEfuseHal->fakeEfuseModifiedMap));
		if (Status == _TRUE) {
			pmp_priv->bloadefusemap = _TRUE;
			sprintf(extra, "efuse file file_read OK\n");
		} else {
			pmp_priv->bloadefusemap = _FALSE;
			sprintf(extra, "efuse file file_read FAIL\n");
		}
	} else {
		sprintf(extra, "efuse file readable FAIL\n");
		RTW_INFO("%s rtw_is_file_readable fail!\n", __func__);
	}
	wrqu->data.length = strlen(extra);
	return 0;
}

int rtw_bt_efuse_file_map(struct net_device *dev,
				struct iw_request_info *info,
				union iwreq_data *wrqu, char *extra)
{
	char *rtw_efuse_file_map_path;
	u8 Status;
	PEFUSE_HAL pEfuseHal;
	PADAPTER padapter = rtw_netdev_priv(dev);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct mp_priv *pmp_priv = &padapter->mppriv;

	pEfuseHal = &pHalData->EfuseHal;
	if (copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))
		return -EFAULT;

	rtw_efuse_file_map_path = extra;

	_rtw_memset(pEfuseHal->fakeBTEfuseModifiedMap, 0xFF, EFUSE_BT_MAX_MAP_LEN);

	if (rtw_is_file_readable(rtw_efuse_file_map_path) == _TRUE) {
		RTW_INFO("%s do rtw_efuse_mask_file_read = %s!\n", __func__, rtw_efuse_file_map_path);
		Status = rtw_efuse_file_read(padapter, rtw_efuse_file_map_path, pEfuseHal->fakeBTEfuseModifiedMap, sizeof(pEfuseHal->fakeBTEfuseModifiedMap));
		if (Status == _TRUE) {
			pmp_priv->bloadBTefusemap = _TRUE;
			sprintf(extra, "BT efuse file file_read OK\n");
		} else {
			pmp_priv->bloadBTefusemap = _FALSE;
			sprintf(extra, "BT efuse file file_read FAIL\n");
		}
	} else {
		sprintf(extra, "BT efuse file readable FAIL\n");
		RTW_INFO("%s rtw_is_file_readable fail!\n", __func__);
	}
	wrqu->data.length = strlen(extra);
	return 0;
}


static inline void dump_buf(u8 *buf, u32 len)
{
	u32 i;

	RTW_INFO("-----------------Len %d----------------\n", len);
	for (i = 0; i < len; i++)
		RTW_INFO("%2.2x-", *(buf + i));
	RTW_INFO("\n");
}

int rtw_mp_link(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_point *wrqu, char *extra)
{
	PADAPTER padapter = rtw_netdev_priv(dev);
	struct mp_priv *pmp_priv;
	char input[RTW_IWD_MAX_LEN];
	int		bgetrxdata = 0, btxdata = 0, bsetbt = 0;
	u32 i = 0, datalen = 0,jj, kk, waittime = 0;
	u16 val = 0x00, res = 0;
	char *pextra = NULL;
	u8 *setdata = NULL;
	char *pch, *ptmp, *token, *tmp[4] = {0x00, 0x00, 0x00};
	int ret = 0;

	pmp_priv = &padapter->mppriv;

	if (rtw_do_mp_iwdata_len_chk(__func__, wrqu->length))
		return -EFAULT;

	_rtw_memset(input, 0, sizeof(input));

	if (copy_from_user(input, wrqu->pointer, wrqu->length)) {
		ret = -EFAULT;
		goto exit;
	}

	_rtw_memset(extra, 0, wrqu->length);

	RTW_INFO("%s: in=%s\n", __func__, input);

	bgetrxdata =  (strncmp(input, "rxdata", 6) == 0) ? 1 : 0; /* strncmp TRUE is 0*/
	btxdata =  (strncmp(input, "txdata", 6) == 0) ? 1 : 0; /* strncmp TRUE is 0*/
	bsetbt =  (strncmp(input, "setbt", 5) == 0) ? 1 : 0; /* strncmp TRUE is 0*/

	if (bgetrxdata) {
		RTW_INFO("%s: in= 1 \n", __func__);
		if (pmp_priv->mplink_brx == _TRUE) {

				while (waittime < 100 && pmp_priv->mplink_brx == _FALSE) {
						if (pmp_priv->mplink_brx == _FALSE)
							rtw_msleep_os(10);
						else
							break;
						waittime++;
				}
				if (pmp_priv->mplink_brx == _TRUE) {
					sprintf(extra, "\n");
					pextra = extra + strlen(extra);
					for (i = 0; i < pmp_priv->mplink_rx_len; i ++) {
						pextra += sprintf(pextra, "%02x:", pmp_priv->mplink_buf[i]);
					}
					_rtw_memset(pmp_priv->mplink_buf, '\0' , sizeof(pmp_priv->mplink_buf));
					pmp_priv->mplink_brx = _FALSE;
				}
		}
	} else if (btxdata) {
		struct pkt_attrib *pattrib;

		pch = input;
		setdata = rtw_zmalloc(1024);
		if (setdata == NULL) {
			ret = -ENOMEM;
			goto exit;
		}

		i = 0;
		while ((token = strsep(&pch, ",")) != NULL) {
			if (i > 2)
				break;
			tmp[i] = token;
			i++;
		}

		/* tmp[0],[1],[2] */
		/* txdata,00e04c871200........... */
		if (strcmp(tmp[0], "txdata") == 0) {
			if (tmp[1] == NULL) {
				ret = -EINVAL;
				goto exit;
			}
		}

		datalen = strlen(tmp[1]);
		if (datalen % 2) {
			ret = -EINVAL;
			goto exit;
		}
		datalen /= 2;
		if (datalen == 0) {
			ret = -EINVAL;
			goto exit;
		}

		RTW_INFO("%s: data len=%d\n", __FUNCTION__, datalen);
		RTW_INFO("%s: tx data=%s\n", __FUNCTION__, tmp[1]);

		for (jj = 0, kk = 0; jj < datalen; jj++, kk += 2)
			setdata[jj] = key_2char2num(tmp[1][kk], tmp[1][kk + 1]);

		dump_buf(setdata, datalen);
		_rtw_memset(pmp_priv->mplink_buf, '\0' , sizeof(pmp_priv->mplink_buf));
		_rtw_memcpy(pmp_priv->mplink_buf, setdata, datalen);

		pattrib = &pmp_priv->tx.attrib;
		pattrib->pktlen = datalen;
		pmp_priv->tx.count = 1;
		pmp_priv->tx.stop = 0;
		pmp_priv->mplink_btx = _TRUE;
		SetPacketTx(padapter);
		pmp_priv->mode = MP_PACKET_TX;

	} else if (bsetbt) {

		pch = input;
		i = 0;

		while ((token = strsep(&pch, ",")) != NULL) {
			if (i > 3)
				break;
			tmp[i] = token;
			i++;
		}

		if (tmp[1] == NULL) {
			ret = -EINVAL;
			goto exit;
		}

		if (strcmp(tmp[1], "scbd") == 0) {
			u16 org_val = 0x8002, pre_val, read_score_board_val;
			u8 state;

			pre_val = (rtw_read16(padapter,(0xaa))) & 0x7fff;

			if (tmp[2] != NULL) {
				state = simple_strtoul(tmp[2], &ptmp, 10);

				if (state)
						org_val = org_val | BIT6;
				else
						org_val = org_val & (~BIT6);

				if (org_val != pre_val) {
					pre_val = org_val;
					rtw_write16(padapter, 0xaa, org_val);
					RTW_INFO("%s,setbt scbd write org_val = 0x%x , pre_val = 0x%x\n", __func__, org_val, pre_val);
				} else {
					RTW_INFO("%s,setbt scbd org_val = 0x%x ,pre_val = 0x%x\n", __func__, org_val, pre_val);
				}
			} else {
					read_score_board_val = (rtw_read16(padapter,(0xaa))) & 0x7fff;
					RTW_INFO("%s,read_score_board_val = 0x%x\n", __func__, read_score_board_val);
			}
			goto exit;

		} else if (strcmp(tmp[1], "testmode") == 0) {

			if (tmp[2] == NULL) {
				ret = -EINVAL;
				goto exit;
			}

			val = simple_strtoul(tmp[2], &ptmp, 16);
			RTW_INFO("get tmp, type  %s, val =0x%x!\n", tmp[1], val);

			if (tmp[2] != NULL) {
				_rtw_memset(extra, 0, wrqu->length);
				res = rtw_btcoex_btset_testmode(padapter, val);
				if (!CHECK_STATUS_CODE_FROM_BT_MP_OPER_RET(res, BT_STATUS_BT_OP_SUCCESS)) {
					RTW_INFO("%s: BT_OP fail = 0x%x!\n", __FUNCTION__, val);
					sprintf(extra, "BT_OP fail  0x%x!\n", val);
				} else
					sprintf(extra, "Set BT_OP 0x%x done!\n", val);
			}

		}
	}

exit:
	if (setdata)
		rtw_mfree(setdata, 1024);

	wrqu->length = strlen(extra);

	rtw_mfree(input, wrqu->length);

	return ret;
}


