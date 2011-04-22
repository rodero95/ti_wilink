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
#include "tidef.h"
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "host_platform.h"
#include "ioctl_init.h"

#include <asm/gpio.h>
#include <WlanDrvIf.h>
#include <asm/mach/mmc.h>

#include "proc_comm.h"

/* TODO: Platform specific */
#define WLAN_EN				(108)
#define LOW     0
#define HIGH    1

#define OS_API_MEM_ADRR  	0x0000000
#define OS_API_REG_ADRR  	0x300000

#ifdef MISSING_WIFI_POWER_SUPPORT
/* Not all platforms have code that sets the external GPIO lines and functionality for powering on/off wifi */
/* Forward declaration */
#warning TODO: Platform specific constant
#define WLAN_IRQ_GPIO		(27)
struct platform_device *sdioDrv_get_platform_device(void);

static uint32_t wifi_on_gpio_table[] = {
#ifdef GPIO_CFG
	GPIO_CFG(WLAN_IRQ_GPIO, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
	GPIO_CFG(WLAN_EN, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),  /* WLAN EN */
#endif
};

static uint32_t wifi_off_gpio_table[] = {
#ifdef GPIO_CFG
	GPIO_CFG(WLAN_IRQ_GPIO, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
	GPIO_CFG(WLAN_IRQ_GPIO, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
#endif
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned int id;

	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

int wifi_power(int on)
{
	struct platform_device *ppdev = 0;
	struct mmc_platform_data *pplat = 0;
	uint32_t res = 0;

	/* Setup the IRQ GPIO */
	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
	}

	/* Turn on/off power */
	ppdev = sdioDrv_get_platform_device();
	if (!ppdev)
		return -ENODEV;
	pplat = ppdev->dev.platform_data;
	if (!pplat)
		return -ENODEV;
	if (!pplat->translate_vdd)
		return -ENODEV;
	res = pplat->translate_vdd(&(ppdev->dev), on);

	mdelay(100);
	gpio_set_value(WLAN_EN, on);
	mdelay(500);

	return res;
}
#else

#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wifi_tiwlan.h>

static struct wifi_platform_data *wifi_control_data = NULL;

static int wifi_probe( struct platform_device *pdev )
{
	struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

	printk("%s\n", __FUNCTION__);
	if( wifi_ctrl )
		wifi_control_data = wifi_ctrl;
	else
		wifi_control_data = NULL;

	return 0;
}

static int wifi_remove( struct platform_device *pdev )
{
	printk("%s\n", __FUNCTION__);
	return 0;
}

static struct platform_driver wifi_device = {
	.probe          = wifi_probe,
	.remove         = wifi_remove,
	.suspend        = NULL,
	.resume         = NULL,
	.driver         = {
		.name   = "msm_wifi",
	},
};

static int wifi_add_dev( void )
{
	printk("%s\n", __FUNCTION__);
	return platform_driver_register( &wifi_device );
}

static void wifi_del_dev( void )
{
	printk("%s\n", __FUNCTION__);
	platform_driver_unregister( &wifi_device );
}

int wifi_power(int on)
{
	if( wifi_control_data && wifi_control_data->set_power ) {
		return wifi_control_data->set_power(on);
	} else {
		printk("%s: wifi control is not ready\n", __FUNCTION__);
	}
	return -1;
}
#else
#define wifi_power legend_wifi_power
int wifi_power(int on);
#endif

#endif /* MISSING_WIFI_POWER_SUPPORT */

/* set the GPIO to low after awaking the TNET from ELP */
int hPlatform_hardResetTnetw(void)
{
  int err;

  // Turn power OFF
  if ((err = wifi_power(0)) == 0)
  {
    mdelay(150);
    // Turn power ON
    err = wifi_power(1);
    mdelay(150);
  }
  return err;
}

int hPlatform_DevicePowerOff (void)
{

    int err = 0;

    err = wifi_power(0);
    //mdelay(150);

    return err;
}

int hPlatform_DevicePowerOffSetLongerDelay (void)
{

    int err = 0;

    err = wifi_power(0);

	mdelay(500);

    return err;
}

int hPlatform_DevicePowerOn (void)
{

    int err = 0;

    err = wifi_power(1);
    //mdelay(150);

    return err;
}

int hPlatform_Wlan_Hardware_Init(void)
{

#ifdef CONFIG_WIFI_CONTROL_FUNC
	printk("%s\n", __FUNCTION__);
	wifi_add_dev();
#endif

    return 0;
}

void hPlatform_Wlan_Hardware_DeInit(void)
{

#ifdef CONFIG_WIFI_CONTROL_FUNC
	printk("%s\n", __FUNCTION__);
	wifi_del_dev();
#endif

}

int hPlatform_initInterrupt(void *tnet_drv, void* handle_add )
{
	TWlanDrvIfObj *drv = (TWlanDrvIfObj*) tnet_drv;
    int rc;


	if (drv->irq == 0 || handle_add == NULL) {
	  print_err("hPlatform_initInterrupt() bad param drv->irq=%d handle_add=0x%x !!!\n",drv->irq,(int)handle_add);
	  return -EINVAL;
	}

	if ((rc = request_irq(drv->irq, handle_add, IRQF_TRIGGER_FALLING, drv->netdev->name, drv))) {
	    print_err("TIWLAN: Failed to register interrupt handler\n");
		return rc;
	}

	if (gpio_request(WLAN_EN,"TI 1271") != 0) {
		return -EINVAL;
	}
	gpio_direction_output(WLAN_EN,LOW);
	set_irq_wake(drv->irq, 1);

	return rc;
}

void hPlatform_freeInterrupt(void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;

	set_irq_wake(drv->irq, 0);
	free_irq(drv->irq, drv);
	gpio_free(WLAN_EN);
}

void* hPlatform_hwGetRegistersAddr(TI_HANDLE OsContext)
{
	return 0;
}

void* hPlatform_hwGetMemoryAddr(TI_HANDLE OsContext)
{
	return 0;
}
