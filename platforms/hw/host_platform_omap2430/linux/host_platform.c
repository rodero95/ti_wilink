/*
 * host_platform.c
 *
 * Copyright(c) 1998 - 2009 Texas Instruments. All rights reserved.      
 * All rights reserved.                                                  
 *                                                                       
 * Redistribution and use in source and binary forms, with or without    
 * modification, are permitted provided that the following conditions    
 * are met:                                                              
 *                                                                       
 *  * Redistributions of source code must retain the above copyright     
 *    notice, this list of conditions and the following disclaimer.      
 *  * Redistributions in binary form must reproduce the above copyright  
 *    notice, this list of conditions and the following disclaimer in    
 *    the documentation and/or other materials provided with the         
 *    distribution.                                                      
 *  * Neither the name Texas Instruments nor the names of its            
 *    contributors may be used to endorse or promote products derived    
 *    from this software without specific prior written permission.      
 *                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "tidef.h"
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/arch-omap/tc.h>
#include <linux/delay.h>

#include "host_platform.h"
#include "ioctl_init.h"
#include "WlanDrvIf.h"

#include "Device1273.h"


#define OS_API_MEM_ADRR  	0x0000000
#define OS_API_REG_ADRR  	0x300000

#define SDIO_ATTEMPT_LONGER_DELAY_LINUX  150

/*--------------------------------------------------------------------------------------*/

static int OMAP2430_pad_config(unsigned int reg_offset, unsigned char mux_mode, unsigned char pull)
{

	unsigned char reg = 0; 
	int rval = 0;

	if (reg_offset) 
	{
		/*   PAD config register: */
		/*    - - - - - - - -     */
		/*   |7|6|5|4|3|2|1|0|    */
		/*   |r|r|r|p|p|m|m|m|    */
		/*    - - - - - - - -     */
		/* m - mux_mode           */
		/* p - pull up/down       */
		/* r - reserved           */

		/* clearing MuxMode bits */
		reg |= ((mux_mode & 7) | (pull & 3) << 3);

		writeb(reg,OMAP24XX_VA_SYSTEM_CONTROL_BASE + reg_offset);
	}
	else
	{
		printk(KERN_ERR "%s:pad_config() reg_offset must be > 0\n",__FILE__);
		rval =  -EINVAL;
	}
		return rval;

} /* OMAP2430_pad_config() */

/*----------------------------------------------------------------------------*/

static int OMAP2430_TNETW_Power(int power_on)

{
  omap_set_gpio_direction(GPIO_10, GPIO_OUTPUT);

  if (power_on)
  {
    omap_set_gpio_dataout(GPIO_10, 1);
  }
  else
  {
    omap_set_gpio_dataout(GPIO_10, 0);
  }

  return 0;
    
} /* OMAP2430_TNETW_Power() */

/*-----------------------------------------------------------------------------

Routine Name:

        hPlatform_hardResetTnetw

Routine Description:

        set the GPIO to low after awaking the TNET from ELP.

Arguments:

        OsContext - our adapter context.


Return Value:

        None

-----------------------------------------------------------------------------*/
int
hPlatform_hardResetTnetw( void )
{
  int err;

    /* Turn power OFF*/
  if ((err = OMAP2430_TNETW_Power(0)) == 0)
  {
    mdelay(200);
    /* Turn power ON*/
    err = OMAP2430_TNETW_Power(1);
    mdelay(200);
  }

  return err;

} /* hPlatform_hardResetTnetw() */

/*--------------------------------------------------------------------------------------*/

/* Turn device power off */
int hPlatform_DevicePowerOff (void)
{
    int err;
    
    err = OMAP2430_TNETW_Power(0);
    
    mdelay(1);
    
    return err;
}

/*--------------------------------------------------------------------------------------*/

/* Turn device power off according to a given delay */
int hPlatform_DevicePowerOffSetLongerDelay(void)
{
    int err;
    
    err = OMAP2430_TNETW_Power(0);
    
    mdelay(SDIO_ATTEMPT_LONGER_DELAY_LINUX);
    
    return err;
}

/*--------------------------------------------------------------------------------------*/

/* Turn device power on */

int hPlatform_DevicePowerOn (void)
{
    int err;

    err = OMAP2430_TNETW_Power(1);

    /* Should not be changed, 50 msec cause failures */
    mdelay(20);

    return err;
}

/*--------------------------------------------------------------------------------------*/

void hPlatform_Wlan_Hardware_DeInit(void)
{
  omap_free_gpio(GPIO_10);

}

/*--------------------------------------------------------------------------------------*/

int hPlatform_Wlan_Hardware_Init(void)
{
  int err;

  do
  {
	if (omap_request_gpio(GPIO_10) != 0) 
	{
	  printk("hPlatform_Wlan_Hardware_Init: omap_request_gpio FAILED\n");
	  err = -EINVAL;
	  break;
	};
	err = OMAP2430_pad_config(CONTROL_PADCONF_UART1_TX_OFFSET, MUX_MODE_3, 0);
	if (err)
	{
	  printk ("hPlatform_Wlan_Hardware_Init: OMAP2430_pad_config(TESTDRV_CONTROL_PADCONF_UART1_TX_OFFSET) FAILD\n");
	  break;
	}
	err = OMAP2430_pad_config(CONTROL_PADCONF_UART1_RX_OFFSET, MUX_MODE_3, 3);
	if (err)
	{
	  printk ("hPlatform_Wlan_Hardware_Init: OMAP2430_pad_config(TESTDRV_CONTROL_PADCONF_UART1_RX_OFFSET) FAILD\n");
	  break;
	}
	/* 
	 * set pull up on all SDIO lines from the 2430 registers 
	 */
	err = OMAP2430_pad_config (CONTROL_MMC2_CMD_PAD_PA, MUX_MODE_0, 3);
	if (err)
	{
      printk ("hPlatform_Wlan_Hardware_Init: ERROR setting CONTROL_MMC2_CMD_PAD_PA\n");
	  break;
	}
	err = OMAP2430_pad_config (CONTROL_MMC2_DAT3_PAD_PA, MUX_MODE_0, 3);
	if (err)
	{
      printk ("hPlatform_Wlan_Hardware_Init: ERROR setting CONTROL_MMC2_DAT3_PAD_PA\n");
	  break;
	}
	err = OMAP2430_pad_config (CONTROL_MMC2_DAT2_PAD_PA, MUX_MODE_0, 3);
	if (err)
	{
      printk ("hPlatform_Wlan_Hardware_Init: ERROR setting CONTROL_MMC2_DAT2_PAD_PA\n");
	  break;
	}
	err = OMAP2430_pad_config (CONTROL_MMC2_DAT1_PAD_PA, MUX_MODE_0, 3);
	if (err)
	{
      printk ("hPlatform_Wlan_Hardware_Init: ERROR setting CONTROL_MMC2_DAT1_PAD_PA\n");
	  break;
	}
	err = OMAP2430_pad_config (CONTROL_MMC2_DAT0_PAD_PA, MUX_MODE_0, 3);
	if (err)
	{
      printk ("hPlatform_Wlan_Hardware_Init: ERROR setting CONTROL_MMC2_DAT0_PAD_PA\n");
	}
/*
	err = hPlatform_DevicePowerOn();
	if (err)
	{
      printk ("hPlatform_Wlan_Hardware_Init: hPlatform_DevicePowerOn() FAILED\n");
	}
*/
  } while(0);

  return err;

} /* hPlatform_Wlan_Hardware_Init() */

/*-----------------------------------------------------------------------------

Routine Name:

        InitInterrupt

Routine Description:

        this function init the interrupt to the Wlan ISR routine.

Arguments:

        tnet_drv - Golbal Tnet driver pointer.


Return Value:

        status

-----------------------------------------------------------------------------*/

int hPlatform_initInterrupt(void *tnet_drv, void* handle_add ) 
{
	TWlanDrvIfObj *drv = tnet_drv;
    int rc;

	if (drv->irq == 0 || handle_add == NULL)
	{
	  print_err("hPlatform_initInterrupt() bad param drv->irq=%d handle_add=0x%x !!!\n",drv->irq,(int)handle_add);
	  return -EINVAL;
	}
	if (omap_request_gpio(GPIO_9) != 0) 
    {
	    print_err("hPlatform_initInterrupt() omap_request_gpio() FAILED !!\n");
		return -EINVAL;
	}
	omap_set_gpio_direction(GPIO_9, OMAP24XX_DIR_INPUT);

	/* change working mode to Rising Edge */
    /* the opsions is: OMAP_GPIO_RISING_EDGE \ OMAP_GPIO_FALLING_EDGE */
#ifdef FPGA1273_STAGE_
    omap_set_gpio_edge_ctrl(GPIO_9, OMAP_GPIO_RISING_EDGE); 
#else
#ifndef USE_IRQ_ACTIVE_HIGH
    omap_set_gpio_edge_ctrl(GPIO_9, OMAP_GPIO_FALLING_EDGE); 
#else
    omap_set_gpio_edge_ctrl(GPIO_9, OMAP_GPIO_RISING_EDGE); 
#endif
#endif

	if ((rc = request_irq(drv->irq, handle_add, SA_SHIRQ, drv->netdev->name, drv)))
	{
	    print_err("TIWLAN: Failed to register interrupt handler\n");
	}

	return rc;

} /* hPlatform_initInterrupt() */

/*--------------------------------------------------------------------------------------*/

void hPlatform_freeInterrupt(void) 
{
  omap_free_gpio(GPIO_9);
}

/****************************************************************************************
 *                        hPlatform_hwGetRegistersAddr()                                 
 ****************************************************************************************
DESCRIPTION:	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
void*
hPlatform_hwGetRegistersAddr(
        TI_HANDLE OsContext
        )
{
	return (void*)OS_API_REG_ADRR;
}

/****************************************************************************************
 *                        hPlatform_hwGetMemoryAddr()                                 
 ****************************************************************************************
DESCRIPTION:	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
void*
hPlatform_hwGetMemoryAddr(
        TI_HANDLE OsContext
        )
{
	return (void*)OS_API_MEM_ADRR;
}


