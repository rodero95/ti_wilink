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
**|     Copyright (c) 1998 - 2009 Texas Instruments Incorporated         |**
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

/*--------------------------------------------------------------------------
 Module:      host_platform_sdio.h

 Purpose:     This module defines unified interface to the host platform specific
              sources and services.

--------------------------------------------------------------------------*/

#ifndef __HOST_PLATFORM_SDIO__H__
#define __HOST_PLATFORM_SDIO__H__

#include <mach/hardware.h>



#define OMAP_HSMMC3_BASE			0x480AD000	//0x480b4000

#define CONTROL_PADCONF_CAM_D1			0x48002118   /* WLAN_EN */
#define CONTROL_PADCONF_MCBSP1_CLKX		0x48002198   /* WLAN_IRQ */

#define CONTROL_PADCONF_CAM_D1_MCS8			0x48002124   /* WLAN_EN in MCS8 expansion board */
#define CONTROL_PADCONF_MCBSP1_CLKX_MCS8	0x48002184   /* WLAN_IRQ in MCS8 expansion board */

#define CONTROL_PADCONF_MMC3_CLK	   	0x480025D8  /* mmc3_cmd */
#define CONTROL_PADCONF_MMC3_CMD	   	0x480021D0  /* mmc3_cmd */


#define CONTROL_PADCONF_MMC3_DAT0		0x480025E4    /* mmc3_dat0, mmc3_dat1 */
#define CONTROL_PADCONF_MMC3_DAT2		0x480025E8    /* mmc3_dat2 */
#define CONTROL_PADCONF_MMC3_DAT3		0x480025E0    /* mmc3_dat3 */



#define PMENA_GPIO                      101
#define IRQ_GPIO                        162

#define PMENA_GPIO_MCS8                 106 
#define IRQ_GPIO_MCS8                   153 


#define MUXMODE_3                       3
#define TIWLAN_IRQ_POLL_INTERVAL	    HZ/100
#define HZ_IN_MSEC						HZ/1000
#define TIWLAN_IRQ_POLL_INTERVAL_MS		TIWLAN_IRQ_POLL_INTERVAL/HZ_IN_MSEC

int 
hPlatform_initInterrupt(
	void* tnet_drv,
	void* handle_add
	);

void*
hPlatform_hwGetRegistersAddr(
    TI_HANDLE OsContext
    );

void*
hPlatform_hwGetMemoryAddr(
    TI_HANDLE OsContext
    );

void hPlatform_freeInterrupt(void);

int  hPlatform_hardResetTnetw(void);
int  hPlatform_Wlan_Hardware_Init(void);
void hPlatform_Wlan_Hardware_DeInit(void);
int  hPlatform_DevicePowerOff(void);
int  hPlatform_DevicePowerOffSetLongerDelay(void);
int  hPlatform_DevicePowerOn(void);
#endif /* __HOST_PLATFORM_SDIO__H__ */
