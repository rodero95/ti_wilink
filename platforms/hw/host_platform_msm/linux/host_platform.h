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

/*--------------------------------------------------------------------------
 Module:      host_platform_sdio.h

 Purpose:     This module defines unified interface to the host platform specific
              sources and services.

--------------------------------------------------------------------------*/

#ifndef __HOST_PLATFORM_SDIO__H__
#define __HOST_PLATFORM_SDIO__H__

#include <mach/hardware.h>
#include <mach/irqs.h>

/* TODO: Platform specific constant */
#define TNETW_IRQ MSM_GPIO_TO_INT(29)
#define TIWLAN_IRQ_POLL_INTERVAL	    (HZ/100)


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

void hPlatform_freeInterrupt(void *tnet_drv);

int  hPlatform_hardResetTnetw(void);
int  hPlatform_Wlan_Hardware_Init(void);
void hPlatform_Wlan_Hardware_DeInit(void);
int  hPlatform_DevicePowerOff(void);
int  hPlatform_DevicePowerOffSetLongerDelay(void);
int  hPlatform_DevicePowerOn(void);
#endif /* __HOST_PLATFORM_SDIO__H__ */
