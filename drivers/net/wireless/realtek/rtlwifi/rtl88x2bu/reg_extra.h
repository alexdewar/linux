#define REG_SYS_CLKR					0x0008
#define REG_EFUSE_TEST					0x0034
#define PWC_EV12V                            BIT(15)
#define FEN_ELDR                             BIT(12)
#define LOADER_CLK_EN                        BIT(5)
#define		HWSET_MAX_SIZE			1024
#define   EFUSE_MAX_SECTION                 64
#define EFUSE_REAL_CONTENT_LEN 512
#define     IMR_BCNINT                              BIT(13)
#define TRP_VAUX_EN				BIT(23)	/* RTL ID */

/* 2 RCR */
#define AAP						BIT(0)
#define APM						BIT(1)
#define AM						BIT(2)
#define AB						BIT(3)
#define ADD3						BIT(4)
#define APWRMGT				BIT(5)
#define CBSSID					BIT(6)
#define CBSSID_DATA				BIT(6)
#define CBSSID_BCN				BIT(7)
#define ACRC32					BIT(8)
#define AICV						BIT(9)
#define ADF						BIT(11)
#define ACF						BIT(12)
#define AMF						BIT(13)
#define HTC_LOC_CTRL			BIT(14)
#define UC_DATA_EN				BIT(16)
#define BM_DATA_EN				BIT(17)
#define MFBEN					BIT(22)
#define LSIGEN					BIT(23)
#define EnMBID					BIT(24)
#define FORCEACK				BIT(26)
#define APP_BASSN				BIT(27)
#define APP_PHYSTS				BIT(28)
#define APP_ICV					BIT(29)
#define APP_MIC					BIT(30)
#define APP_FCS					BIT(31)

/* ----------------------------------------------------------------------------
 * CAM Config Setting (offset 0x680, 1 byte)
 * ----------------------------------------------------------------------------			 */
#define CAM_VALID				BIT(15)
#define CAM_NOTVALID			0x0000
#define CAM_USEDK				BIT(5)

#define CAM_CONTENT_COUNT	8

#define CAM_NONE				0x0
#define CAM_WEP40				0x01
#define CAM_TKIP				0x02
#define CAM_AES					0x04
#define CAM_WEP104				0x05
#define CAM_SMS4				0x6

#define TOTAL_CAM_ENTRY		32
#define HALF_CAM_ENTRY			16

#define CAM_CONFIG_USEDK		_TRUE
#define CAM_CONFIG_NO_USEDK	_FALSE

#define CAM_WRITE				BIT(16)
#define CAM_READ				0x00000000
#define CAM_POLLINIG			BIT(31)

/*
 * 12. Host Interrupt Status Registers
 *
 * ----------------------------------------------------------------------------
 * 8190 IMR/ISR bits
 * ---------------------------------------------------------------------------- */
#define IMR8190_DISABLED		0x0
#define IMR_DISABLED			0x0
/* IMR DW0 Bit 0-31 */
#define IMR_BCNDMAINT6			BIT(31)		/* Beacon DMA Interrupt 6 */
#define IMR_BCNDMAINT5			BIT(30)		/* Beacon DMA Interrupt 5 */
#define IMR_BCNDMAINT4			BIT(29)		/* Beacon DMA Interrupt 4 */
#define IMR_BCNDMAINT3			BIT(28)		/* Beacon DMA Interrupt 3 */
#define IMR_BCNDMAINT2			BIT(27)		/* Beacon DMA Interrupt 2 */
#define IMR_BCNDMAINT1			BIT(26)		/* Beacon DMA Interrupt 1 */
#define IMR_BCNDOK8				BIT(25)		/* Beacon Queue DMA OK Interrupt 8 */
#define IMR_BCNDOK7				BIT(24)		/* Beacon Queue DMA OK Interrupt 7 */
#define IMR_BCNDOK6				BIT(23)		/* Beacon Queue DMA OK Interrupt 6 */
#define IMR_BCNDOK5				BIT(22)		/* Beacon Queue DMA OK Interrupt 5 */
#define IMR_BCNDOK4				BIT(21)		/* Beacon Queue DMA OK Interrupt 4 */
#define IMR_BCNDOK3				BIT(20)		/* Beacon Queue DMA OK Interrupt 3 */
#define IMR_BCNDOK2				BIT(19)		/* Beacon Queue DMA OK Interrupt 2 */
#define IMR_BCNDOK1				BIT(18)		/* Beacon Queue DMA OK Interrupt 1 */
#define IMR_TIMEOUT2			BIT(17)		/* Timeout interrupt 2 */
#define IMR_TIMEOUT1			BIT(16)		/* Timeout interrupt 1 */
#define IMR_TXFOVW				BIT(15)		/* Transmit FIFO Overflow */
#define IMR_PSTIMEOUT			BIT(14)		/* Power save time out interrupt */
#define IMR_BcnInt				BIT(13)		/* Beacon DMA Interrupt 0 */
#define IMR_RXFOVW				BIT(12)		/* Receive FIFO Overflow */
#define IMR_RDU					BIT(11)		/* Receive Descriptor Unavailable */
#define IMR_ATIMEND				BIT(10)		/* For 92C, ATIM Window End Interrupt. For 8723 and later ICs, it also means P2P CTWin End interrupt. */
#define IMR_BDOK				BIT(9)		/* Beacon Queue DMA OK Interrupt */
#define IMR_HIGHDOK				BIT(8)		/* High Queue DMA OK Interrupt */
#define IMR_TBDOK				BIT(7)		/* Transmit Beacon OK interrupt */
#define IMR_MGNTDOK			BIT(6)		/* Management Queue DMA OK Interrupt */
#define IMR_TBDER				BIT(5)		/* For 92C, Transmit Beacon Error Interrupt */
#define IMR_BKDOK				BIT(4)		/* AC_BK DMA OK Interrupt */
#define IMR_BEDOK				BIT(3)		/* AC_BE DMA OK Interrupt */
#define IMR_VIDOK				BIT(2)		/* AC_VI DMA OK Interrupt */
#define IMR_VODOK				BIT(1)		/* AC_VO DMA Interrupt */
#define IMR_ROK					BIT(0)		/* Receive DMA OK Interrupt */

/* 13. Host Interrupt Status Extension Register	 (Offset: 0x012C-012Eh) */
#define IMR_TSF_BIT32_TOGGLE	BIT(15)
#define IMR_BcnInt_E				BIT(12)
#define IMR_TXERR				BIT(11)
#define IMR_RXERR				BIT(10)
#define IMR_C2HCMD				BIT(9)
#define IMR_CPWM				BIT(8)
/* RSVD [2-7] */
#define IMR_OCPINT				BIT(1)
#define IMR_WLANOFF			BIT(0)
