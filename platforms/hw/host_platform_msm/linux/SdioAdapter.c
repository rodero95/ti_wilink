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

/** \file   SdioAdapter.c
 *  \brief  The SDIO driver adapter. Platform dependent.
 *
 * An adaptation layer between the lower SDIO driver (in BSP) and the upper SdioBusDrv.
 * Used for issuing all SDIO transaction types towards the lower SDIO-driver.
 * Makes the decision whether to use Sync or Async transaction, and reflects it to the caller
 *     by the return value and calling its callback in case of Async.
 *
 *  \see    SdioAdapter.h, SdioDrv.c & h
 */
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "SdioDrvDbg.h"
#include "TxnDefs.h"
#include "SdioAdapter.h"
#include "SdioDrv.h"
#include <asm/mach/mmc.h>

static unsigned char *pDmaBufAddr = 0;

/************************************************************************
 * Defines
 ************************************************************************/
#ifdef SDIO_1_BIT /* see also in SdioDrv.c */
#define SDIO_BITS_CODE   0x80 /* 1 bits */
#else
#define SDIO_BITS_CODE   0x82 /* 4 bits */
#endif

#define MAX_RETRIES                 10

#define MAX_BUS_TXN_SIZE            8192  /* Max bus transaction size in bytes (for the DMA buffer allocation) */

/* For block mode configuration */
#define FN0_FBR2_REG_108                    0x210
#define FN0_FBR2_REG_108_BIT_MASK           0xFFF

#define CHECK_ERR(status) { if ((status)) { printk(KERN_INFO "ERROR DETECT: STOP!\n"); while (1); } }

int sdioAdapt_ConnectBus (void *        fCbFunc,
                          void *        hCbArg,
                          unsigned int  uBlkSizeShift,
                          unsigned int  uSdioThreadPriority,
                          unsigned char **pRxDmaBufAddr,
                          unsigned int  *pRxDmaBufLen,
                          unsigned char **pTxDmaBufAddr,
                          unsigned int  *pTxDmaBufLen)
{
    unsigned char  uByte;
    unsigned long  uLong;
    unsigned long  uCount = 0;
    unsigned int   uBlkSize = 1 << uBlkSizeShift;
    int            iStatus;

    /* Allocate a DMA-able buffer and provide it to the upper layer to be used for all read and write transactions */
    if (pDmaBufAddr == 0) /* allocate only once (in case this function is called multiple times) */
    {
        pDmaBufAddr = (char *) kmalloc (MAX_BUS_TXN_SIZE, GFP_ATOMIC | GFP_DMA);
        if (pDmaBufAddr == 0) { return -1; }
    }
    *pRxDmaBufAddr = *pTxDmaBufAddr = pDmaBufAddr;
    *pRxDmaBufLen  = *pTxDmaBufLen  = MAX_BUS_TXN_SIZE;

    /* Init SDIO driver and HW */
    iStatus = sdioDrv_ConnectBus (fCbFunc, hCbArg, uBlkSizeShift, uSdioThreadPriority);
	if (iStatus) { return iStatus; }

    sdioDrv_set_clock(TIWLAN_SDIO_INITIAL_CLOCK);

    /* Send commands sequence: 0, 5, 3, 7 */
    iStatus = sdioDrv_ExecuteCmd (SD_IO_GO_IDLE_STATE, 0, MMC_RSP_NONE, &uByte, sizeof(uByte));
    if (iStatus) {
       printk("%s %d command number: %d failed\n", __FUNCTION__, __LINE__, SD_IO_GO_IDLE_STATE);
       return iStatus;
    }
    iStatus = sdioDrv_ExecuteCmd (SDIO_CMD5, VDD_VOLTAGE_WINDOW, MMC_RSP_R4, &uByte, sizeof(uByte));
    if (iStatus) {
        printk("%s %d command number: %d failed\n", __FUNCTION__, __LINE__, SDIO_CMD5);
        return iStatus;
    }
    iStatus = sdioDrv_ExecuteCmd (SD_IO_SEND_RELATIVE_ADDR, 0, MMC_RSP_R6, &uLong, sizeof(uLong));
    if (iStatus) {
       printk("%s %d command number: %d failed\n", __FUNCTION__, __LINE__, SD_IO_SEND_RELATIVE_ADDR);
       return iStatus;
    }
    iStatus = sdioDrv_ExecuteCmd (SD_IO_SELECT_CARD, uLong, MMC_RSP_R6, &uByte, sizeof(uByte));
    if (iStatus) {
       printk("%s %d command number: %d failed\n", __FUNCTION__, __LINE__, SD_IO_SELECT_CARD);
       return iStatus;
    }

    /* NOTE:
     * =====
     * Each of the following loops is a workaround for a HW bug that will be solved in PG1.1 !!
     * Each write of CMD-52 to function-0 should use it as follows:
     * 1) Write the desired byte using CMD-52
     * 2) Read back the byte using CMD-52
     * 3) Write two dummy bytes to address 0xC8 using CMD-53
     * 4) If the byte read in step 2 is different than the written byte repeat the sequence
     */

    /* set device side bus width to 4 bit (for 1 bit write 0x80 instead of 0x82) */
    do
    {
        uByte = SDIO_BITS_CODE;
        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, CCCR_BUS_INTERFACE_CONTOROL, &uByte, 1, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_ReadDirectBytes (TXN_FUNC_ID_CTRL, CCCR_BUS_INTERFACE_CONTOROL, &uByte, 1, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, 0xC8, (unsigned char*)&uLong, 2, 1);
        if (iStatus) { return iStatus; }

        uCount++;

    } while ((uByte != SDIO_BITS_CODE) && (uCount < MAX_RETRIES));


    uCount = 0;

    /* allow function 2 */
    do
    {
        uByte = 4;
        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, CCCR_IO_ENABLE, &uByte, 1, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_ReadDirectBytes (TXN_FUNC_ID_CTRL, CCCR_IO_ENABLE, &uByte, 1, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, 0xC8, (unsigned char*)&uLong, 2, 1);
        if (iStatus) { return iStatus; }

        uCount++;

    } while ((uByte != 4) && (uCount < MAX_RETRIES));


#ifdef SDIO_IN_BAND_INTERRUPT

    uCount = 0;

    do
    {
        uByte = 3;
        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, CCCR_INT_ENABLE, &uByte, 1, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_ReadDirectBytes (TXN_FUNC_ID_CTRL, CCCR_INT_ENABLE, &uByte, 1, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, 0xC8, &uLong, 2, 1);
        if (iStatus) { return iStatus; }

        uCount++;

    } while ((uByte != 3) && (uCount < MAX_RETRIES));


#endif

    uCount = 0;

    /* set block size for SDIO block mode */
    do
    {
        uLong = uBlkSize;
        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, FN0_FBR2_REG_108, (unsigned char*)&uLong, 2, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_ReadDirectBytes (TXN_FUNC_ID_CTRL, FN0_FBR2_REG_108, (unsigned char*)&uLong, 2, 1);
        if (iStatus) { return iStatus; }

        iStatus = sdioDrv_WriteDirectBytes (TXN_FUNC_ID_CTRL, 0xC8, (unsigned char*)&uLong, 2, 1);
        if (iStatus) { return iStatus; }

        uCount++;

    } while (((uLong & FN0_FBR2_REG_108_BIT_MASK) != uBlkSize) && (uCount < MAX_RETRIES));


    if (uCount >= MAX_RETRIES)
    {
        /* Failed to write CMD52_WRITE to function 0 */
        return (int)uCount;
    }

    sdioDrv_set_clock(TIWLAN_SDIO_CLOCK);

    return iStatus;
}


int sdioAdapt_DisconnectBus (void)
{
    return sdioDrv_DisconnectBus();
}
extern int use52;
ETxnStatus sdioAdapt_Transact (unsigned int  uFuncId,
                               unsigned int  uHwAddr,
                               void *        pHostAddr,
                               unsigned int  uLength,
                               unsigned int  bDirection,
                               unsigned int  bBlkMode,
                               unsigned int  bFixedAddr,
                               unsigned int  bMore)
{
    int iStatus,i;


    if (bDirection) { /* Read */
		iStatus = sdioDrv_ReadSync (uFuncId, uHwAddr, pHostAddr, uLength, bBlkMode, bFixedAddr, bMore);
		if (iStatus)
		{
			return TXN_STATUS_ERROR;
		}
		return TXN_STATUS_COMPLETE;
    } else { /* Write */
	    if (use52)
	    {
	//    printk("use 52\n");jj = 0;
	        for (i = 0; i < uLength ; ++i)
	        {
	           // printk("uHwAddr = 0x%x *pHostValue = 0x%x\n",uHwAddr,(unsigned char)(*((unsigned char*)pHostAddr)));
	            sdioAdapt_TransactBytes(uFuncId,uHwAddr,pHostAddr,1,bDirection,bMore);
	            uHwAddr += 1;
	            pHostAddr = (void *)((unsigned int)pHostAddr+1);

	        }
	        return TXN_STATUS_COMPLETE;
	    }
	    else
	    {
			iStatus = sdioDrv_WriteSync (uFuncId, uHwAddr, pHostAddr, uLength, bBlkMode, bFixedAddr, bMore);
		}

	    if (iStatus)
		{
			return TXN_STATUS_ERROR;
		}
		return TXN_STATUS_COMPLETE;
    }
}

ETxnStatus sdioAdapt_TransactBytes (unsigned int  uFuncId,
                                    unsigned int  uHwAddr,
                                    void *        pHostAddr,
                                    unsigned int  uLength,
                                    unsigned int  bDirection,
                                    unsigned int  bMore)
{
    int iStatus;

    /* Call read or write bytes Sync method */
    if (bDirection)
    {
        iStatus = sdioDrv_ReadDirectBytes(uFuncId, uHwAddr, pHostAddr, uLength, bMore);
    }
    else
    {
        iStatus = sdioDrv_WriteDirectBytes(uFuncId, uHwAddr, pHostAddr, uLength, bMore);
    }

    // If failed return ERROR, if succeeded return COMPLETE
    if (iStatus)
    {
        return TXN_STATUS_ERROR;
    }
    return TXN_STATUS_COMPLETE;
}




