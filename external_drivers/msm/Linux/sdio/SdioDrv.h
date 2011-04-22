/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2008 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/

#ifndef __MSM_SDIODRV_API_H
#define __MSM_SDIODRV_API_H

#include <asm/types.h>
#include <linux/mmc/mmc.h>


//#define SDIO_1_BIT


#ifdef SDIO_1_BIT
#define TIWLAN_SDIO_BUSWIDE MMC_BUS_WIDTH_1
#else
#define TIWLAN_SDIO_BUSWIDE MMC_BUS_WIDTH_4
#endif

#define TIWLAN_SDIO_INITIAL_CLOCK (144 * 1000)
#define TIWLAN_SDIO_CLOCK (25 * 1000 * 1000)

/* Card Common Control Registers (CCCR) */

#define CCCR_SDIO_REVISION                  0x00
#define CCCR_SD_SPECIFICATION_REVISION      0x01
#define CCCR_IO_ENABLE                      0x02
#define CCCR_IO_READY                       0x03
#define CCCR_INT_ENABLE                     0x04
#define CCCR_INT_PENDING                    0x05
#define CCCR_IO_ABORT                       0x06
#define CCCR_BUS_INTERFACE_CONTOROL         0x07
#define CCCR_CARD_CAPABILITY	            0x08
#define CCCR_COMMON_CIS_POINTER             0x09 /*0x09-0x0B*/
#define CCCR_FNO_BLOCK_SIZE	                0x10 /*0x10-0x11*/
#define FN0_CCCR_REG_32	                    0x64


/* Pprotocol defined constants */

#define SD_IO_GO_IDLE_STATE		  		    0
#define SD_IO_SEND_RELATIVE_ADDR	  	    3
#define SDIO_CMD5			  			    5
#define SD_IO_SELECT_CARD		  		    7
#define SDIO_CMD52		 	 			    52
#define SDIO_CMD53		 	 			    53
#define SDIO_SHIFT(v,n)                     (v<<n)
#define SDIO_RWFLAG(v)                      (SDIO_SHIFT(v,31))
#define SDIO_FUNCN(v)                       (SDIO_SHIFT(v,28))
#define SDIO_RAWFLAG(v)                     (SDIO_SHIFT(v,27))
#define SDIO_BLKM(v)                        (SDIO_SHIFT(v,27))
#define SDIO_OPCODE(v)                      (SDIO_SHIFT(v,26))
#define SDIO_ADDRREG(v)                     (SDIO_SHIFT(v,9))


#define VDD_VOLTAGE_WINDOW                  0xffffc0
/********************************************************************/
/*	SDIO driver functions prototypes                                */
/********************************************************************/
int sdioDrv_ConnectBus     (void *       fCbFunc,
                            void *       hCbArg,
                            unsigned int uBlkSizeShift,
                            unsigned int uSdioThreadPriority);


int sdioDrv_DisconnectBus  (void);

int sdioDrv_ExecuteCmd     (unsigned int uCmd,
                            unsigned int uArg,
                            unsigned int uRespType,
                            void *       pResponse,
                            unsigned int uLen);

int sdioDrv_ReadSync           (unsigned int uFunc,
                            unsigned int uHwAddr,
                            void *       pData,
                            unsigned int uLen,
                            unsigned int bBlkMode,
                            unsigned int bIncAddr,
                            unsigned int bMore);

int sdioDrv_WriteSync          (unsigned int uFunc,
                            unsigned int uHwAddr,
                            void *       pData,
                            unsigned int uLen,
                            unsigned int bBlkMode,
                            unsigned int bIncAddr,
                            unsigned int bMore);

int sdioDrv_ReadDirectBytes(unsigned int  uFunc,
                            unsigned int  uHwAddr,
                            unsigned char *pData,
                            unsigned int  uLen,
                            unsigned int  bMore);

int sdioDrv_WriteDirectBytes(unsigned int  uFunc,
                             unsigned int  uHwAddr,
                             unsigned char *pData,
                             unsigned int  uLen,
                             unsigned int  bMore);

void sdioDrv_register_pm(int (*wlanDrvIf_Start)(void),
						int (*wlanDrvIf_Stop)(void));


int sdioDrv_set_clock(unsigned int clock);


#endif
