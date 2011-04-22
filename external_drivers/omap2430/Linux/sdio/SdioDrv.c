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
 
/** \file   SdioDrv.c 
 *  \brief  The OMAP2430 SDIO driver (platform and OS dependent) 
 * 
 * The lower SDIO driver (BSP) for OMAP2430 on Linux OS.
 * Provides all SDIO commands and read/write operation methods.
 *  
 *  \see    SdioDrv.h
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/arch/io.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <asm/arch/hardware.h>
#include <linux/errno.h>
#include <asm/hardware/clock.h>
#include <asm/arch/clock.h>
#include <asm/arch/dma.h>
#include <asm/io.h>
#include "SdioDrvDbg.h"
#include "SdioDrv.h"

/************************************************************************
 * Defines
 ************************************************************************/

#define TIWLAN_MMC_CONTROLLER              2
#define TIWLAN_MMC_CONTROLLER_BASE_ADDR    0x480b4000
#define TIWLAN_MMC_CONTROLLER_BASE_SIZE    512
#define OMAP2430_SYSC1_GENERAL1_REGS_ADDR  0x49002270
#define OMAP2430_SYSC1_GENERAL1_REGS_SIZE  1023
#define CONTROL_MMC2_CMD_PAD_PA            (0x49002000 + 0x00FB)
#define CONTROL_MMC2_DAT3_PAD_PA           (0x49002000 + 0x00FA)
#define CONTROL_MMC2_DAT2_PAD_PA           (0x49002000 + 0x00FD)
#define CONTROL_MMC2_DAT1_PAD_PA           (0x49002000 + 0x00FE)
#define CONTROL_MMC2_DAT0_PAD_PA           (0x49002000 + 0x00FC)
#define CONTROL_MMC2_SIZE                  (CONTROL_MMC2_DAT1_PAD_PA - CONTROL_MMC2_DAT3_PAD_PA + 1)
#define CONFIG_MUX_MODE_0                  (0 << 0)
#define CONFIG_MUX_PULL_UP_ENABLED         (3 << 3)

#define OMAP_MMC_MASTER_CLOCK          96000000
/*
 *  HSMMC Host Controller Registers
 */
#define OMAP_HSMMC_SYSCONFIG           0x0010
#define OMAP_HSMMC_SYSSTATUS           0x0014
#define OMAP_HSMMC_CSRE                0x0024
#define OMAP_HSMMC_SYSTEST             0x0028
#define OMAP_HSMMC_CON                 0x002C
#define OMAP_HSMMC_BLK                 0x0104
#define OMAP_HSMMC_ARG                 0x0108
#define OMAP_HSMMC_CMD                 0x010C
#define OMAP_HSMMC_RSP10               0x0110
#define OMAP_HSMMC_RSP32               0x0114
#define OMAP_HSMMC_RSP54               0x0118
#define OMAP_HSMMC_RSP76               0x011C
#define OMAP_HSMMC_DATA                0x0120
#define OMAP_HSMMC_PSTATE              0x0124
#define OMAP_HSMMC_HCTL                0x0128
#define OMAP_HSMMC_SYSCTL              0x012C
#define OMAP_HSMMC_STAT                0x0130
#define OMAP_HSMMC_IE                  0x0134
#define OMAP_HSMMC_ISE                 0x0138
#define OMAP_HSMMC_AC12                0x013C
#define OMAP_HSMMC_CAPA                0x0140
#define OMAP_HSMMC_CUR_CAPA            0x0148
#define OMAP_HSMMC_REV                 0x01FC


#define VS18                           (1 << 26)
#define VS30                           (1 << 25)
#define SRA                            (1 << 24)
#define SDVS18                         (0x5 << 9)
#define SDVS30                         (0x6 << 9)
#define SDVSCLR                        0xFFFFF1FF
#define SDVSDET                        0x00000400  
#define SIDLE_MODE                     (0x2 << 3)   
#define AUTOIDLE                       0x1        
#define SDBP                           (1 << 8)     
#define DTO                            0xE        
#define ICE                            0x1        
#define ICS                            0x2        
#define CEN                            (1 << 2)     
#define CLKD_MASK                      0x0000FFC0 
#define IE_EN_MASK                     0x317F0137  
#define INIT_STREAM                    (1 << 1)     
#define DP_SELECT                      (1 << 21)    
#define DDIR                           (1 << 4)     
#define DMA_EN                         0x1        
#define MSBS                           (1 << 5)
#define BCE                            (1 << 1)     
#define ONE_BIT                        (~(0x2))
#define EIGHT_BIT                      (~(0x20))   
#define CC                             0x1        
#define TC                             0x02       
#define OD                             0x1        
#define BRW                            0x400      
#define BRR                            0x800      
#define BRE                            (1 << 11)    
#define BWE                            (1 << 10)    
#define SBGR                           (1 << 16)    
#define CT                             (1 << 17)    
#define SDIO_READ                      (1 << 31)    
#define SDIO_BLKMODE                   (1 << 27)    
#define OMAP_HSMMC_ERR                 (1 << 15)  /* Any error */
#define OMAP_HSMMC_CMD_TIMEOUT         (1 << 16)  /* Com mand response time-out */
#define OMAP_HSMMC_DATA_TIMEOUT        (1 << 20)  /* Data response time-out */
#define OMAP_HSMMC_CMD_CRC             (1 << 17)  /* Command CRC error */
#define OMAP_HSMMC_DATA_CRC            (1 << 21)  /* Date CRC error */
#define OMAP_HSMMC_CARD_ERR            (1 << 28)  /* Card ERR */
#define OMAP_HSMMC_STAT_CLEAR          0xFFFFFFFF 
#define INIT_STREAM_CMD                0x00000000 
#define INT_CLEAR                      0x00000000 
#define BLK_CLEAR                      0x00000000 

/* SCM CONTROL_DEVCONF1 MMC1 overwrite but */

#define MMC1_ACTIVE_OVERWRITE          (1 << 31)                      
                                                                    
#define sdio_blkmode_regaddr           0x2000                       
#define sdio_blkmode_mask              0xFF00                       
                                                                    
#define IO_RW_DIRECT_MASK              0xF000FF00                   
#define IO_RW_DIRECT_ARG_MASK          0x80001A00                   
                                                                    
#define RMASK                          (MMC_RSP_MASK | MMC_RSP_CRC)   
#define MMC_TIMEOUT_MS                 20                           
#define MMCA_VSN_4                     4                            
                                                                    
#define VMMC1_DEV_GRP                  0x27                         
#define P1_DEV_GRP                     0x20                         
#define VMMC1_DEDICATED                0x2A                         
#define VSEL_3V                        0x02                         
#define VSEL_18V                       0x00                         
#define PBIAS_3V                       0x03                         
#define PBIAS_18V                      0x02                         
#define PBIAS_LITE                     0x04A0                       
#define PBIAS_CLR                      0x00                         

/* Required for the temp hack required for MMC1 Pull up. */
#define TWL_GPIO_PUPDCTR1              0x13 
#define T2_GPIO_MODULE_ENABLE_MASK     0x04    
#define MMC2_GPIO_CARDDET_PU_PD_MASK   0xF3   
#define MMC2_GPIO_CARDDET_PU           0x08    
#define MMC2_GPIO_CARDDET_DEB          0x02    
#define MMC2_GPIO_CARDDET              301     

#define OMAP_MMC_REGS_BASE             IO_ADDRESS(OMAP_HSMMC2_BASE)
#define OMAP_MMC_IRQ                   INT_MMC2_IRQ
/* 
 * MMC Host controller read/write API's.
 */
#define OMAP_HSMMC_READ_OFFSET(offset) (__raw_readl((OMAP_MMC_REGS_BASE) + (offset)))
#define OMAP_HSMMC_READ(reg)           (__raw_readl((OMAP_MMC_REGS_BASE) + OMAP_HSMMC_##reg))
#define OMAP_HSMMC_WRITE(reg, val)     (__raw_writel((val), (OMAP_MMC_REGS_BASE) + OMAP_HSMMC_##reg))

#define OMAP_HSMMC_SEND_COMMAND(cmd, arg) do \
{ \
	OMAP_HSMMC_WRITE(ARG, arg); \
	OMAP_HSMMC_WRITE(CMD, cmd); \
} while (0)

#define OMAP_HSMMC_CMD52_WRITE     ((SD_IO_RW_DIRECT    << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16))
#define OMAP_HSMMC_CMD52_READ      (((SD_IO_RW_DIRECT   << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16)) | DDIR)
#define OMAP_HSMMC_CMD53_WRITE     (((SD_IO_RW_EXTENDED << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16)) | DP_SELECT)
#define OMAP_HSMMC_CMD53_READ      (((SD_IO_RW_EXTENDED << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16)) | DP_SELECT | DDIR)
#define OMAP_HSMMC_CMD53_READ_DMA  (OMAP_HSMMC_CMD53_READ  | DMA_EN)
#define OMAP_HSMMC_CMD53_WRITE_DMA (OMAP_HSMMC_CMD53_WRITE | DMA_EN)

/* Macros to build commands 52 and 53 in format according to SDIO spec */
#define SDIO_CMD52_READ(v1,v2,v3,v4)        (SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_RAWFLAG(v3)| SDIO_ADDRREG(v4))
#define SDIO_CMD52_WRITE(v1,v2,v3,v4,v5)    (SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_RAWFLAG(v3)| SDIO_ADDRREG(v4)|(v5))
#define SDIO_CMD53_READ(v1,v2,v3,v4,v5,v6)  (SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_BLKM(v3)| SDIO_OPCODE(v4)|SDIO_ADDRREG(v5)|(v6&0x1ff))
#define SDIO_CMD53_WRITE(v1,v2,v3,v4,v5,v6) (SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_BLKM(v3)| SDIO_OPCODE(v4)|SDIO_ADDRREG(v5)|(v6&0x1ff))

#define SDIODRV_MAX_LOOPS 50000

static int      initCount = 0;
static struct   workqueue_struct *pWorkQueue;   /* The sdio work queue */
#define         SDIO_DRV_WK_NAME "ti_sdio_drv"

/************************************************************************
 * Types
 ************************************************************************/
typedef struct OMAP2430_sdiodrv
{
  struct clk    *fclk, *iclk, *dbclk;
  int           dma_tx_channel;
  int           dma_rx_channel;
  void          (*BusTxnCB)(void* BusTxnHandle, int status);
  void*         BusTxnHandle;
  unsigned int  uBlkSize;
  unsigned int  uBlkSizeShift;
  int           async_status;
}  OMAP2430_sdiodrv_t;

/********************************************************************/
/*	SDIO driver parameters and structures					       */
/********************************************************************/

#define SDIO_DRIVER_NAME 			"TIWLAN_SDIO"


MODULE_PARM(g_sdio_debug_level, "i");
MODULE_PARM_DESC(g_sdio_debug_level, "debug level");
int g_sdio_debug_level = SDIO_DEBUGLEVEL_ERR;
EXPORT_SYMBOL( g_sdio_debug_level);

OMAP2430_sdiodrv_t g_drv;
void sdiodrv_task(void *unused);
struct work_struct sdiodrv_work;

int enable_mmc_power(int slot);
int disable_mmc_power(int slot);

void sdiodrv_task(void *unused)
{
	if (g_drv.BusTxnCB != NULL)
	{
	  g_drv.BusTxnCB(g_drv.BusTxnHandle, g_drv.async_status);
	}
} /* sdiodrv_task() */

/********************************************************************/
/*	SDIO driver interrupt handling                                  */
/********************************************************************/
irqreturn_t sdiodrv_irq(int irq, void *drv, struct pt_regs *cpu_regs)
{
    int status;

	PDEBUG("sdiodrv_irq()\n");
	status = OMAP_HSMMC_READ(STAT);
	OMAP_HSMMC_WRITE(ISE, 0);
	g_drv.async_status = status & (OMAP_HSMMC_ERR);
	if (g_drv.async_status)
	{
		PERR("sdiodrv_irq: ERROR in STAT = 0x%x\n", status);
	}

    status = queue_work (pWorkQueue, &sdiodrv_work);
    if (!status)
    {
        printk("\n***ERROR in sdiodrv_irq: queue_work Failes with status = %d \n", status);
    }

    return IRQ_HANDLED;
  
} /* sdiodrv_irq() */

/********************************************************************/
/*	SDIO driver internal functions                                  */
/********************************************************************/

/*--------------------------------------------------------------------------------------*/
/*==================================== DMA stuff =======================================*/
/*--------------------------------------------------------------------------------------*/

void sdiodrv_dma_cb(int lch, u16 ch_status, void *data)
{
    int status;
	PDEBUG("sdiodrv_dma_cb() channel=%d status=0x%x\n", lch, (int)ch_status);

	g_drv.async_status = ch_status & (1 << 7);
   
    status = queue_work (pWorkQueue, &sdiodrv_work);
    if (!status)
    {
        printk("\nERROR in sdiodrv_dma_cb: queue_work Failes with status = %d \n", status);
    }

} /* sdiodrv_dma_cb() */

/*--------------------------------------------------------------------------------------*/

int sdiodrv_dma_init(void)
{
  int rc;

  rc = omap_request_dma(OMAP_DMA_MMC2_TX, "SDIO WRITE", NULL, &g_drv, &g_drv.dma_tx_channel);
  if (rc != 0) 
  {
	PERR("sdiodrv_dma_init() omap_request_dma(OMAP_DMA_MMC2_TX) FAILED\n");
	return rc;
  }
  rc = omap_request_dma(OMAP_DMA_MMC2_RX, "SDIO READ", sdiodrv_dma_cb, &g_drv, &g_drv.dma_rx_channel);
  if (rc != 0) 
  {
	PERR("sdiodrv_dma_init() omap_request_dma(OMAP_DMA_MMC2_RX) FAILED\n");
	return rc;
  }
  omap_set_dma_src_params(g_drv.dma_rx_channel, OMAP_DMA_AMODE_CONSTANT,(OMAP_HSMMC2_BASE) + OMAP_HSMMC_DATA,0,0);
  omap_set_dma_dest_params(g_drv.dma_tx_channel, OMAP_DMA_AMODE_CONSTANT,(OMAP_HSMMC2_BASE) + OMAP_HSMMC_DATA, 0, 0);
  return rc;

} /* sdiodrv_dma_init() */

/*--------------------------------------------------------------------------------------*/

void sdiodrv_dma_shutdown(void)
{
  omap_free_dma(g_drv.dma_tx_channel);
  omap_free_dma(g_drv.dma_rx_channel);
} /* sdiodrv_dma_shutdown() */

/*--------------------------------------------------------------------------------------*/
/*================================= End of DMA stuff ===================================*/
/*--------------------------------------------------------------------------------------*/

static u32 sdiodrv_poll_status(u32 reg_offset, u32 stat, unsigned int msecs)
{
    u32 status=0, loops=0;
	/*	typeof(jiffies) timeout = jiffies + msecs_to_jiffies(msecs); */
    /*  Note: don't use the timer for now, because it may be blocked by the TXN critical section */

	do
    {
	    status=OMAP_HSMMC_READ_OFFSET(reg_offset);
	    if(( status & stat))
		{
		  break;
		}
	} while (loops++ < SDIODRV_MAX_LOOPS);
/*	} while (time_before(jiffies, timeout)); */

	return status;

} /* sdiodrv_poll_status */

/*--------------------------------------------------------------------------------------*/

static int sdiodrv_send_command(u32 cmdreg, u32 cmdarg)
{
    OMAP_HSMMC_WRITE(STAT, OMAP_HSMMC_STAT_CLEAR);
	OMAP_HSMMC_SEND_COMMAND(cmdreg, cmdarg);

	return sdiodrv_poll_status(OMAP_HSMMC_STAT, CC, MMC_TIMEOUT_MS);

} /* sdiodrv_send_command() */

/*--------------------------------------------------------------------------------------*/
/*
 *  Disable clock to the card
 */
static void OMAP2430_mmc_stop_clock(void)
{
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(SYSCTL) & CEN) != 0x0)
    {
		PERR("MMC clock not stoped, clock freq can not be altered\n");
    }
} /* OMAP2430_mmc_stop_clock */

/*--------------------------------------------------------------------------------------*/
/*
 *  Reset the SD system
 */
#define SRA (1 << 24)

int OMAP2430_mmc_reset(void)
{
    int status, loops=0;
/*	typeof(jiffies) timeout; */

    /*  Note: don't use the timer for now, because it may be blocked by the TXN critical section */
	
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) | SRA);
/*	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS); */
/*	while ((status = OMAP_HSMMC_READ(SYSCTL) &  SRA) && time_before(jiffies, timeout)); */
	while ((status = OMAP_HSMMC_READ(SYSCTL) &  SRA) && loops++ < SDIODRV_MAX_LOOPS);
	if (status & SRA)
	{
	    PERR("OMAP2430_mmc_reset() MMC reset FAILED!! status=0x%x\n",status);
	}

	return status;

} /* OMAP2430_mmc_reset */

/*--------------------------------------------------------------------------------------*/

static void OMAP2430_mmc_set_clock(unsigned int clock, OMAP2430_sdiodrv_t *host)
{
	u16           dsor = 0;
	unsigned long regVal;
	int           status;

	PDEBUG("OMAP2430_mmc_set_clock(%d)\n",clock);
	if (clock) 
    {
		/* Enable MMC_SD_CLK */
		dsor = OMAP_MMC_MASTER_CLOCK / clock;
		if (dsor < 1)
        {
            dsor = 1;
        }
		if (OMAP_MMC_MASTER_CLOCK / dsor > clock)
        {
			dsor++;
        }
		if (dsor > 250)
        {
			dsor = 250;
        }
	}
	OMAP2430_mmc_stop_clock();
	regVal = OMAP_HSMMC_READ(SYSCTL);
	regVal = regVal & ~(CLKD_MASK);
	regVal = regVal | (dsor << 6);
	regVal = regVal | (DTO << 16);
	OMAP_HSMMC_WRITE(SYSCTL, regVal);
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) | ICE);
	/* 
     * wait till the the clock is satble (ICS) bit is set
	 */
	status  = sdiodrv_poll_status(OMAP_HSMMC_SYSCTL, ICS, MMC_TIMEOUT_MS);
	if(!(status & ICS)) 
    {
	    PERR("OMAP2430_mmc_set_clock() clock not stable!! status=0x%x\n",status);
	}
	/* 
     * Enable clock to the card
	 */
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) | CEN);

} /* OMAP2430_mmc_set_clock() */




/*--------------------------------------------------------------------------------------*/

/********************************************************************/
/*	SDIO driver interface functions                                 */
/********************************************************************/

/*--------------------------------------------------------------------------------------*/

int sdioDrv_InitHw(void)
{
    int rc=0;
#ifdef SDIO_1_BIT /* see also in SdioAdapter.c */
	unsigned long clock_rate = 6000000;
#else
	unsigned long clock_rate = 24000000;
#endif
	
    enable_mmc_power(TIWLAN_MMC_CONTROLLER);

    g_drv.fclk  = clk_get(NULL, "mmchs2_fck");
    g_drv.iclk  = clk_get(NULL, "mmchs2_ick");
    g_drv.dbclk = clk_get(NULL, "mmchsdb2_fck");
	
    clk_use(g_drv.fclk);
    clk_use(g_drv.iclk);
    clk_use(g_drv.dbclk);

    clk_safe(g_drv.fclk);
    clk_safe(g_drv.iclk);
    clk_safe(g_drv.dbclk);

    OMAP2430_mmc_reset();

    /* 1.8V */
    OMAP_HSMMC_WRITE(CAPA,      OMAP_HSMMC_READ(CAPA) | VS18);
    OMAP_HSMMC_WRITE(HCTL,      OMAP_HSMMC_READ(HCTL) | SDVS18);
    /* clock gating */
    OMAP_HSMMC_WRITE(SYSCONFIG, OMAP_HSMMC_READ(SYSCONFIG) | AUTOIDLE);

    OMAP_HSMMC_WRITE(SYSCONFIG,0x308);

    /* bus power */
    OMAP_HSMMC_WRITE(HCTL,      OMAP_HSMMC_READ(HCTL) | SDBP);
    /* interrupts */
    OMAP_HSMMC_WRITE(ISE,       0);
    OMAP_HSMMC_WRITE(IE,        IE_EN_MASK);

	OMAP2430_mmc_set_clock(clock_rate, &g_drv);
	printk("SDIO clock Configuration is now set to %dMhz\n",(int)clock_rate/1000000);

	/* Bus width */
#ifdef SDIO_1_BIT /* see also in SdioAdapter.c */
	PDEBUG("%s() setting %d data lines\n",__FUNCTION__, 1);
	OMAP_HSMMC_WRITE(HCTL, OMAP_HSMMC_READ(HCTL) & (ONE_BIT));
#else
	PDEBUG("%s() setting %d data lines\n",__FUNCTION__, 4);
	OMAP_HSMMC_WRITE(HCTL, OMAP_HSMMC_READ(HCTL) | (1 << 1));
#endif
    /* send the init sequence. 80 clocks of synchronization in the SDIO */
    OMAP_HSMMC_WRITE( CON, OMAP_HSMMC_READ(CON) | INIT_STREAM);
    OMAP_HSMMC_SEND_COMMAND( 0, 0);
    sdiodrv_poll_status(OMAP_HSMMC_STAT, CC, MMC_TIMEOUT_MS);
    OMAP_HSMMC_WRITE( CON, OMAP_HSMMC_READ(CON) & ~INIT_STREAM);

    return rc;

} /* sdiodrv_init */

/*--------------------------------------------------------------------------------------*/


void sdiodrv_shutdown(void)
{
	PDEBUG("entering %s()\n" , __FUNCTION__ );
	clk_unuse(g_drv.fclk);
	clk_unuse(g_drv.iclk);
	clk_unuse(g_drv.dbclk);
	if (disable_mmc_power(TIWLAN_MMC_CONTROLLER) != 0)
	{
	    PERR("Unable to disable power to MMC2\n");
	}
    
	free_irq(OMAP_MMC_IRQ, &g_drv);
	sdiodrv_dma_shutdown();
	PDEBUG("exiting %s\n",__FUNCTION__);

    return;

} /* sdiodrv_shutdown() */

/*--------------------------------------------------------------------------------------*/

static int sdiodrv_send_data_xfer_commad(u32 cmd, u32 cmdarg, int length, u32 buffer_enable_status, unsigned int bBlkMode)
{
    int status;

	PDEBUG("%s() writing CMD 0x%x ARG 0x%x\n",__FUNCTION__, cmd, cmdarg);

    /* block mode */
    if(bBlkMode)
    {
        /* 
         * Bits 31:16 of BLK reg: NBLK Blocks count for current transfer.
         *                        in case of Block MOde the lenght is treated here as number of blocks 
         *                        (and not as a length).
         * Bits 11:0 of BLK reg: BLEN Transfer Block Size. in case of block mode set that field to block size. 
         */
        OMAP_HSMMC_WRITE(BLK, (length << 16) | (g_drv.uBlkSize << 0));

        /*
         * In CMD reg:
         * BCE: Block Count Enable
         * MSBS: Multi/Single block select
         */
        cmd |= MSBS | BCE ;
    }
    else
    {
        OMAP_HSMMC_WRITE(BLK, length);
    }

    status = sdiodrv_send_command(cmd, cmdarg);
	if(!(status & CC)) 
    {
	    PERR("sdiodrv_send_data_xfer_commad() SDIO Command error! STAT = 0x%x\n", status);
	    return 0;
	}
	PDEBUG("%s() length = %d(%dw) BLK = 0x%x\n",
		   __FUNCTION__, length,((length + 3) >> 2), OMAP_HSMMC_READ(BLK));

    return sdiodrv_poll_status(OMAP_HSMMC_PSTATE, buffer_enable_status, MMC_TIMEOUT_MS);

} /* sdiodrv_send_data_xfer_commad() */

/*--------------------------------------------------------------------------------------*/

int sdiodrv_data_xfer_sync(u32 cmd, u32 cmdarg, void *data, int length, u32 buffer_enable_status)
{
    u32 buf_start, buf_end, data32;
	int status;

    status = sdiodrv_send_data_xfer_commad(cmd, cmdarg, length, buffer_enable_status, 0);
	if(!(status & buffer_enable_status)) 
    {
	    PERR("sdiodrv_data_xfer_sync() buffer disabled! length = %d BLK = 0x%x PSTATE = 0x%x\n", 
			   length, OMAP_HSMMC_READ(BLK), status);
	    return -1;
	}
	buf_end = (u32)data+(u32)length;

	/*
	 * Read loop 
	 */
	if (buffer_enable_status == BRE)
	{
	  if (((u32)data & 3) == 0) /* 4 bytes aligned */
	  {
		for (buf_start = (u32)data; (u32)data < buf_end; data += sizeof(unsigned long))
		{
		  *((unsigned long*)(data)) = OMAP_HSMMC_READ(DATA);
		}
	  }
	  else                      /* 2 bytes aligned */
	  {
		for (buf_start = (u32)data; (u32)data < buf_end; data += sizeof(unsigned long))
		{
		  data32 = OMAP_HSMMC_READ(DATA);
		  *((unsigned short *)data)     = (unsigned short)data32;
		  *((unsigned short *)data + 1) = (unsigned short)(data32 >> 16);
		}
	  }
	}
	/*
	 * Write loop 
	 */
	else
	{
	  if (((u32)data & 3) == 0) /* 4 bytes aligned */
	  {
		for (buf_start = (u32)data; (u32)data < buf_end; data += sizeof(unsigned long))
		{
		  OMAP_HSMMC_WRITE(DATA,*((unsigned long*)(data)));
		}
	  }
	  else                      /* 2 bytes aligned */
	  {
		for (buf_start = (u32)data; (u32)data < buf_end; data += sizeof(unsigned long))
		{
		  OMAP_HSMMC_WRITE(DATA,*((unsigned short*)data) | *((unsigned short*)data+1) << 16 );
		}

	  }
	}
	status  = sdiodrv_poll_status(OMAP_HSMMC_STAT, TC, MMC_TIMEOUT_MS);
	if(!(status & TC)) 
    {
	    PERR("sdiodrv_data_xfer_sync() transfer error! STAT = 0x%x\n", status);
	    return -1;
	}

    return 0;

} /* sdiodrv_data_xfer_sync() */

/*--------------------------------------------------------------------------------------*/

/********************************************************************/
/*	SDIO driver interface functions                                 */
/********************************************************************/

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ConnectBus (void *        fCbFunc, 
                        void *        hCbArg, 
                        unsigned int  uBlkSizeShift, 
                        unsigned int  uSdioThreadPriority)
{
    g_drv.BusTxnCB      = fCbFunc;
    g_drv.BusTxnHandle  = hCbArg;
    g_drv.uBlkSizeShift = uBlkSizeShift;  
    g_drv.uBlkSize      = 1 << uBlkSizeShift;

    if (0 == initCount)
    {
        initCount = 1;
        pWorkQueue = create_singlethread_workqueue (SDIO_DRV_WK_NAME);
        INIT_WORK(&sdiodrv_work, sdiodrv_task, 0);
    }

    return sdioDrv_InitHw ();
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_DisconnectBus (void)
{
    return 0;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ExecuteCmd (unsigned int uCmd, 
                        unsigned int uArg, 
                        unsigned int uRespType, 
                        void *       pResponse, 
                        unsigned int uLen)
{
	unsigned int uCmdReg   = 0;
	unsigned int uStatus   = 0;
	unsigned int uResponse = 0;

	PDEBUG("sdioDrv_ExecuteCmd() starting cmd %02x arg %08x\n", (int)uCmd, (int)uArg);

	uCmdReg = (uCmd << 24) | (uRespType << 16) ;

	uStatus = sdiodrv_send_command(uCmdReg, uArg);

	if (!(uStatus & CC)) 
    {
	    PERR("sdioDrv_ExecuteCmd() SDIO Command error status = 0x%x\n", uStatus);
	    return -1;
	}
	if ((uLen > 0) && (uLen <= 4))
	{
	    uResponse = OMAP_HSMMC_READ(RSP10);
		memcpy (pResponse, (char *)&uResponse, uLen);
		PDEBUG("sdioDrv_ExecuteCmd() response = 0x%x\n", uResponse);
	}

    return 0;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ReadSync (unsigned int uFunc, 
                      unsigned int uHwAddr, 
                      void *       pData, 
                      unsigned int uLen,
                      unsigned int bIncAddr,
                      unsigned int bMore)
{
	unsigned int uCmdArg;
	int          iStatus;

	uCmdArg = SDIO_CMD53_READ(0, uFunc, 0, bIncAddr, uHwAddr, uLen);

	iStatus = sdiodrv_data_xfer_sync(OMAP_HSMMC_CMD53_READ, uCmdArg, pData, uLen, BRE);
	if (iStatus != 0)
	{
        PERR("sdioDrv_ReadSync() FAILED!!\n");
	}

	return iStatus;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ReadAsync (unsigned int uFunc, 
                       unsigned int uHwAddr, 
                       void *       pData, 
                       unsigned int uLen, 
                       unsigned int bBlkMode,
                       unsigned int bIncAddr,
                       unsigned int bMore)
{
	int          iStatus;
	unsigned int uCmdArg;
    unsigned int uNumBlks;
    unsigned int uDmaBlockCount;
    unsigned int uNumOfElem;
    
    if (bBlkMode)
    {
        /* For block mode use number of blocks instead of length in bytes */
        uNumBlks = uLen >> g_drv.uBlkSizeShift;
        uDmaBlockCount = uNumBlks;
        /* due to the DMA config to 32Bit per element (OMAP_DMA_DATA_TYPE_S32) the division is by 4 */ 
        uNumOfElem = g_drv.uBlkSize >> 2;
    }
    else
    {	
        uNumBlks = uLen;
        uDmaBlockCount = 1;
        uNumOfElem = (uLen + 3) >> 2;
    }

    uCmdArg = SDIO_CMD53_READ(0, uFunc, bBlkMode, bIncAddr, uHwAddr, uNumBlks);

    iStatus = sdiodrv_send_data_xfer_commad(OMAP_HSMMC_CMD53_READ_DMA, uCmdArg, uNumBlks, BRE, bBlkMode);
    if (!(iStatus & BRE)) 
    {
        PERR("sdioDrv_ReadAsync() buffer disabled! length = %d BLK = 0x%x PSTATE = 0x%x, BlkMode = %d\n", 
              uLen, OMAP_HSMMC_READ(BLK), iStatus, bBlkMode);
        return -1;
    }
	PDEBUG("sdiodrv_read_async() dma_ch=%d \n",g_drv.dma_rx_channel);
	consistent_sync(pData, uLen, DMA_FROM_DEVICE);
	omap_set_dma_dest_params    (g_drv.dma_rx_channel, OMAP_DMA_AMODE_POST_INC,(unsigned int)virt_to_phys(pData), 0, 0);
	omap_set_dma_transfer_params(g_drv.dma_rx_channel, OMAP_DMA_DATA_TYPE_S32, uNumOfElem , uDmaBlockCount , OMAP_DMA_SYNC_FRAME, OMAP_DMA_MMC2_RX, OMAP_DMA_SRC_SYNC);
	if ((iStatus = omap_start_dma(g_drv.dma_rx_channel)) != 0)
	{
	  PERR("sdiodrv_read_async() - omap_start_dma() FAILED!! rc=0x%x\n", iStatus);
	}

    /* Continued at sdiodrv_irq() after DMA transfer is finished */

	return iStatus;    

}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_WriteSync (unsigned int uFunc, 
                       unsigned int uHwAddr, 
                       void *       pData, 
                       unsigned int uLen,
                       unsigned int bIncAddr,
                       unsigned int bMore)
{
	unsigned int uCmdArg;
	int          iStatus;

    uCmdArg = SDIO_CMD53_WRITE(1, uFunc, 0, bIncAddr, uHwAddr, uLen);

	iStatus = sdiodrv_data_xfer_sync(OMAP_HSMMC_CMD53_WRITE, uCmdArg, pData, uLen, BWE);
	if (iStatus != 0)
	{
        PERR("sdioDrv_WriteSync() FAILED!!\n");
	}

	return iStatus;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_WriteAsync (unsigned int uFunc, 
                        unsigned int uHwAddr, 
                        void *       pData, 
                        unsigned int uLen, 
                        unsigned int bBlkMode,
                        unsigned int bIncAddr,
                        unsigned int bMore)
{
	int          iStatus;
	unsigned int uCmdArg;
    unsigned int uNumBlks;
    unsigned int uDmaBlockCount;
    unsigned int uNumOfElem;
    
    if (bBlkMode)
    {
        /* For block mode use number of blocks instead of length in bytes */
        uNumBlks = uLen >> g_drv.uBlkSizeShift;
        uDmaBlockCount = uNumBlks;
        /* due to the DMA config to 32Bit per element (OMAP_DMA_DATA_TYPE_S32) the division is by 4 */ 
        uNumOfElem = g_drv.uBlkSize >> 2;
    }
    else
    {	
        uNumBlks = uLen;
        uDmaBlockCount = 1;
        uNumOfElem = (uLen + 3) >> 2;
    }
    uCmdArg = SDIO_CMD53_WRITE(1, uFunc, bBlkMode, bIncAddr, uHwAddr, uNumBlks);

    iStatus = sdiodrv_send_data_xfer_commad(OMAP_HSMMC_CMD53_WRITE_DMA, uCmdArg, uNumBlks, BWE, bBlkMode);
    if (!(iStatus & BWE)) 
    {
        PERR("sdioDrv_WriteAsync() buffer disabled! length = %d, BLK = 0x%x, Status = 0x%x\n", 
             uLen, OMAP_HSMMC_READ(BLK), iStatus);
        return -1;
    }
	OMAP_HSMMC_WRITE(ISE, TC);
	consistent_sync(pData, uLen, DMA_TO_DEVICE);
	omap_set_dma_src_params     (g_drv.dma_tx_channel, OMAP_DMA_AMODE_POST_INC, (unsigned int)virt_to_phys(pData), 0, 0);
	omap_set_dma_transfer_params(g_drv.dma_tx_channel, OMAP_DMA_DATA_TYPE_S32, uNumOfElem, uDmaBlockCount, OMAP_DMA_SYNC_FRAME, OMAP_DMA_MMC2_TX, OMAP_DMA_DST_SYNC);
	if ((iStatus = omap_start_dma(g_drv.dma_tx_channel)) != 0)
	{
	  PERR("sdiodrv_write_async() - omap_start_dma() FAILED!! rc=0x%x\n", iStatus);
	}
    /* Continued at sdiodrv_irq() after DMA transfer is finished */

	return iStatus;    
    
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ReadSyncBytes (unsigned int  uFunc, 
                           unsigned int  uHwAddr, 
                           unsigned char *pData, 
                           unsigned int  uLen, 
                           unsigned int  bMore)
{
	unsigned int uCmdArg;
	unsigned int i;
	int          iStatus;

    for (i = 0; i < uLen; i++) 
    {
        uCmdArg = SDIO_CMD52_READ(0, uFunc, 0, uHwAddr);

        iStatus = sdiodrv_send_command(OMAP_HSMMC_CMD52_READ, uCmdArg);

        if (!(iStatus & CC)) 
        {
            PERR("sdioDrv_ReadSyncBytes() SDIO Command error status = 0x%x\n", iStatus);
            return -1;
        }
        else
        {
            *pData = (unsigned char)(OMAP_HSMMC_READ(RSP10));
        }

        pData++;
        uHwAddr++;
    }

    return 0;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_WriteSyncBytes (unsigned int  uFunc, 
                            unsigned int  uHwAddr, 
                            unsigned char *pData, 
                            unsigned int  uLen, 
                            unsigned int  bMore)
{
	unsigned int uCmdArg;
	unsigned int i;
	int          iStatus;

    for (i = 0; i < uLen; i++) 
    {
        uCmdArg = SDIO_CMD52_WRITE(1, uFunc, 0, uHwAddr, *pData);

        iStatus = sdiodrv_send_command(OMAP_HSMMC_CMD52_WRITE, uCmdArg);

        if (!(iStatus & CC)) 
        {
            PERR("sdioDrv_WriteSyncBytes() SDIO Command error status = 0x%x\n", iStatus);
            return -1;
        }

        pData++;
        uHwAddr++;
    }

    return 0;
}

/*--------------------------------------------------------------------------------------*/

static int __init sdioDrv_init(void)
{
    int rc;
	PDEBUG("entering %s()\n" , __FUNCTION__ );
	memset(&g_drv, 0, sizeof(g_drv));

    rc= request_irq(OMAP_MMC_IRQ, sdiodrv_irq, 0, SDIO_DRIVER_NAME, &g_drv);
    if (rc != 0)
    {
	  PERR("sdioDrv_init() -  request_irq FAILED!!\n");
	  return rc;
    }

    rc = sdiodrv_dma_init();
    if (rc != 0)
    {
        PERR("sdioDrv_init() - sdiodrv_dma_init FAILED!!\n");
    }
    return 0;
}

/*--------------------------------------------------------------------------------------*/

static void __exit sdioDrv_exit(void)
{
    sdiodrv_shutdown();
}

/*--------------------------------------------------------------------------------------*/

module_init(sdioDrv_init);
module_exit(sdioDrv_exit);

EXPORT_SYMBOL(sdioDrv_ConnectBus);
EXPORT_SYMBOL(sdioDrv_DisconnectBus);
EXPORT_SYMBOL(sdioDrv_ExecuteCmd);
EXPORT_SYMBOL(sdioDrv_ReadSync);
EXPORT_SYMBOL(sdioDrv_ReadAsync);
EXPORT_SYMBOL(sdioDrv_WriteSync);
EXPORT_SYMBOL(sdioDrv_WriteAsync);
EXPORT_SYMBOL(sdioDrv_ReadSyncBytes);
EXPORT_SYMBOL(sdioDrv_WriteSyncBytes);
MODULE_LICENSE("GPL");
