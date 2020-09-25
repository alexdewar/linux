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
#ifndef __RTL8188F_RECV_H__
#define __RTL8188F_RECV_H__

	#ifndef MAX_RECVBUF_SZ

				#define MAX_RECVBUF_SZ (32768) /* 32k */
			/* #define MAX_RECVBUF_SZ (20480) */ /* 20K */
			/* #define MAX_RECVBUF_SZ (10240)  */ /* 10K */
			/* #define MAX_RECVBUF_SZ (16384) */ /* 16k - 92E RX BUF :16K */
			/* #define MAX_RECVBUF_SZ (8192+1024) */ /* 8K+1k		 */
	#endif /* !MAX_RECVBUF_SZ */

/* Rx smooth factor */
#define	Rx_Smooth_Factor (20)


int rtl8188fu_init_recv_priv(_adapter *padapter);
void rtl8188fu_free_recv_priv(_adapter *padapter);
void rtl8188fu_init_recvbuf(_adapter *padapter, struct recv_buf *precvbuf);


void rtl8188f_query_rx_desc_status(union recv_frame *precvframe, u8 *pdesc);

#endif /* __RTL8188F_RECV_H__ */
