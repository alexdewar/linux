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
#ifndef __RTL8723B_RECV_H__
#define __RTL8723B_RECV_H__

#define RECV_BLK_SZ 512
#define RECV_BLK_CNT 16
#define RECV_BLK_TH RECV_BLK_CNT


	#ifndef MAX_RECVBUF_SZ
			/* #define MAX_RECVBUF_SZ (32768) */ /* 32k */
			/* #define MAX_RECVBUF_SZ (16384) */ /* 16K */
			/* #define MAX_RECVBUF_SZ (10240) */ /* 10K */
				#define MAX_RECVBUF_SZ (15360) /* 15k < 16k */
			/* #define MAX_RECVBUF_SZ (8192+1024) */ /* 8K+1k */
	#endif /* !MAX_RECVBUF_SZ */


/* Rx smooth factor */
#define	Rx_Smooth_Factor (20)



	int rtl8723bu_init_recv_priv(_adapter *padapter);
	void rtl8723bu_free_recv_priv(_adapter *padapter);
	void rtl8723bu_init_recvbuf(_adapter *padapter, struct recv_buf *precvbuf);


void rtl8723b_query_rx_desc_status(union recv_frame *precvframe, u8 *pdesc);

#endif /* __RTL8723B_RECV_H__ */
