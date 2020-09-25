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
/*

The purpose of rtw_io.c

a. provides the API

b. provides the protocol engine

c. provides the software interface between caller and the hardware interface


Compiler Flag Option:

1. CONFIG_SDIO_HCI:
    a. USE_SYNC_IRP:  Only sync operations are provided.
    b. USE_ASYNC_IRP:Both sync/async operations are provided.

2. CONFIG_USB_HCI:
   a. USE_ASYNC_IRP: Both sync/async operations are provided.

3. CONFIG_CFIO_HCI:
   b. USE_SYNC_IRP: Only sync operations are provided.


Only sync read/rtw_write_mem operations are provided.

jackson@realtek.com.tw

*/

#define _RTW_IO_C_

#include <drv_types.h>
#include <hal_data.h>

	#define rtw_le16_to_cpu(val)		le16_to_cpu(val)
	#define rtw_le32_to_cpu(val)		le32_to_cpu(val)
	#define rtw_cpu_to_le16(val)		cpu_to_le16(val)
	#define rtw_cpu_to_le32(val)		cpu_to_le32(val)


u8 _rtw_read8(_adapter *adapter, u32 addr)
{
	u8 r_val;
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	u8(*_read8)(struct intf_hdl *pintfhdl, u32 addr);
	_read8 = pintfhdl->io_ops._read8;

	r_val = _read8(pintfhdl, addr);
	return r_val;
}

u16 _rtw_read16(_adapter *adapter, u32 addr)
{
	u16 r_val;
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	u16(*_read16)(struct intf_hdl *pintfhdl, u32 addr);
	_read16 = pintfhdl->io_ops._read16;

	r_val = _read16(pintfhdl, addr);
	return rtw_le16_to_cpu(r_val);
}

u32 _rtw_read32(_adapter *adapter, u32 addr)
{
	u32 r_val;
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	u32(*_read32)(struct intf_hdl *pintfhdl, u32 addr);
	_read32 = pintfhdl->io_ops._read32;

	r_val = _read32(pintfhdl, addr);
	return rtw_le32_to_cpu(r_val);

}

int _rtw_write8(_adapter *adapter, u32 addr, u8 val)
{
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	int (*_write8)(struct intf_hdl *pintfhdl, u32 addr, u8 val);
	int ret;
	_write8 = pintfhdl->io_ops._write8;

	ret = _write8(pintfhdl, addr, val);

	return RTW_STATUS_CODE(ret);
}
int _rtw_write16(_adapter *adapter, u32 addr, u16 val)
{
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	int (*_write16)(struct intf_hdl *pintfhdl, u32 addr, u16 val);
	int ret;
	_write16 = pintfhdl->io_ops._write16;

	val = rtw_cpu_to_le16(val);
	ret = _write16(pintfhdl, addr, val);

	return RTW_STATUS_CODE(ret);
}
int _rtw_write32(_adapter *adapter, u32 addr, u32 val)
{
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	int (*_write32)(struct intf_hdl *pintfhdl, u32 addr, u32 val);
	int ret;
	_write32 = pintfhdl->io_ops._write32;

	val = rtw_cpu_to_le32(val);
	ret = _write32(pintfhdl, addr, val);

	return RTW_STATUS_CODE(ret);
}

int _rtw_writeN(_adapter *adapter, u32 addr , u32 length , u8 *pdata)
{
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl	*pintfhdl = (struct intf_hdl *)(&(pio_priv->intf));
	int (*_writeN)(struct intf_hdl *pintfhdl, u32 addr, u32 length, u8 *pdata);
	int ret;
	_writeN = pintfhdl->io_ops._writeN;

	ret = _writeN(pintfhdl, addr, length, pdata);

	return RTW_STATUS_CODE(ret);
}


int _rtw_write8_async(_adapter *adapter, u32 addr, u8 val)
{
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	int (*_write8_async)(struct intf_hdl *pintfhdl, u32 addr, u8 val);
	int ret;
	_write8_async = pintfhdl->io_ops._write8_async;

	ret = _write8_async(pintfhdl, addr, val);

	return RTW_STATUS_CODE(ret);
}
int _rtw_write16_async(_adapter *adapter, u32 addr, u16 val)
{
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	int (*_write16_async)(struct intf_hdl *pintfhdl, u32 addr, u16 val);
	int ret;
	_write16_async = pintfhdl->io_ops._write16_async;
	val = rtw_cpu_to_le16(val);
	ret = _write16_async(pintfhdl, addr, val);

	return RTW_STATUS_CODE(ret);
}
int _rtw_write32_async(_adapter *adapter, u32 addr, u32 val)
{
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	int (*_write32_async)(struct intf_hdl *pintfhdl, u32 addr, u32 val);
	int ret;
	_write32_async = pintfhdl->io_ops._write32_async;
	val = rtw_cpu_to_le32(val);
	ret = _write32_async(pintfhdl, addr, val);

	return RTW_STATUS_CODE(ret);
}

void _rtw_read_mem(_adapter *adapter, u32 addr, u32 cnt, u8 *pmem)
{
	void (*_read_mem)(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pmem);
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);


	if (RTW_CANNOT_RUN(adapter)) {
		return;
	}

	_read_mem = pintfhdl->io_ops._read_mem;

	_read_mem(pintfhdl, addr, cnt, pmem);


}

void _rtw_write_mem(_adapter *adapter, u32 addr, u32 cnt, u8 *pmem)
{
	void (*_write_mem)(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pmem);
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);


	_write_mem = pintfhdl->io_ops._write_mem;

	_write_mem(pintfhdl, addr, cnt, pmem);


}

void _rtw_read_port(_adapter *adapter, u32 addr, u32 cnt, u8 *pmem)
{
	u32(*_read_port)(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pmem);
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);


	if (RTW_CANNOT_RUN(adapter)) {
		return;
	}

	_read_port = pintfhdl->io_ops._read_port;

	_read_port(pintfhdl, addr, cnt, pmem);


}

void _rtw_read_port_cancel(_adapter *adapter)
{
	void (*_read_port_cancel)(struct intf_hdl *pintfhdl);
	struct io_priv *pio_priv = &adapter->iopriv;
	struct intf_hdl *pintfhdl = &(pio_priv->intf);

	_read_port_cancel = pintfhdl->io_ops._read_port_cancel;

	RTW_DISABLE_FUNC(adapter, DF_RX_BIT);

	if (_read_port_cancel)
		_read_port_cancel(pintfhdl);
}

u32 _rtw_write_port(_adapter *adapter, u32 addr, u32 cnt, u8 *pmem)
{
	u32(*_write_port)(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pmem);
	/* struct	io_queue  	*pio_queue = (struct io_queue *)adapter->pio_queue; */
	struct io_priv *pio_priv = &adapter->iopriv;
	struct	intf_hdl		*pintfhdl = &(pio_priv->intf);
	u32 ret = _SUCCESS;


	_write_port = pintfhdl->io_ops._write_port;

	ret = _write_port(pintfhdl, addr, cnt, pmem);


	return ret;
}

u32 _rtw_write_port_and_wait(_adapter *adapter, u32 addr, u32 cnt, u8 *pmem, int timeout_ms)
{
	int ret = _SUCCESS;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)pmem;
	struct submit_ctx sctx;

	rtw_sctx_init(&sctx, timeout_ms);
	pxmitbuf->sctx = &sctx;

	ret = _rtw_write_port(adapter, addr, cnt, pmem);

	if (ret == _SUCCESS) {
		ret = rtw_sctx_wait(&sctx, __func__);

		if (ret != _SUCCESS)
			pxmitbuf->sctx = NULL;
	}

	return ret;
}

void _rtw_write_port_cancel(_adapter *adapter)
{
	void (*_write_port_cancel)(struct intf_hdl *pintfhdl);
	struct io_priv *pio_priv = &adapter->iopriv;
	struct intf_hdl *pintfhdl = &(pio_priv->intf);

	_write_port_cancel = pintfhdl->io_ops._write_port_cancel;

	RTW_DISABLE_FUNC(adapter, DF_TX_BIT);

	if (_write_port_cancel)
		_write_port_cancel(pintfhdl);
}
int rtw_init_io_priv(_adapter *padapter, void (*set_intf_ops)(_adapter *padapter, struct _io_ops *pops))
{
	struct io_priv	*piopriv = &padapter->iopriv;
	struct intf_hdl *pintf = &piopriv->intf;

	if (set_intf_ops == NULL)
		return _FAIL;

	piopriv->padapter = padapter;
	pintf->padapter = padapter;
	pintf->pintf_dev = adapter_to_dvobj(padapter);

	set_intf_ops(padapter, &pintf->io_ops);

	return _SUCCESS;
}

/*
* Increase and check if the continual_io_error of this @param dvobjprive is larger than MAX_CONTINUAL_IO_ERR
* @return _TRUE:
* @return _FALSE:
*/
int rtw_inc_and_chk_continual_io_error(struct dvobj_priv *dvobj)
{
	int ret = _FALSE;
	int value;

	value = ATOMIC_INC_RETURN(&dvobj->continual_io_error);
	if (value > MAX_CONTINUAL_IO_ERR) {
		RTW_INFO("[dvobj:%p][ERROR] continual_io_error:%d > %d\n", dvobj, value, MAX_CONTINUAL_IO_ERR);
		ret = _TRUE;
	} else {
		/* RTW_INFO("[dvobj:%p] continual_io_error:%d\n", dvobj, value); */
	}
	return ret;
}

/*
* Set the continual_io_error of this @param dvobjprive to 0
*/
void rtw_reset_continual_io_error(struct dvobj_priv *dvobj)
{
	ATOMIC_SET(&dvobj->continual_io_error, 0);
}

