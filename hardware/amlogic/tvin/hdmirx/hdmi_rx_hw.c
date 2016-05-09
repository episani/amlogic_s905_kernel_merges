/*
 * Amlogic M6TV
 * HDMI RX
 * Copyright (C) 2010 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */


#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
//#include <linux/amports/canvas.h>
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <mach/clock.h>
#include <mach/register.h>
#include <mach/power_gate.h>

#include <linux/amlogic/tvin/tvin.h>
/* Local include */
#include "hdmirx_drv.h"
#include "hdmi_rx_reg.h"
#ifdef CEC_FUNC_ENABLE
#include "hdmirx_cec.h"
#endif


#define EDID_AUTO_CHECKSUM_ENABLE   0               // Checksum byte selection: 0=Use data stored in MEM; 1=Use checksum calculated by HW.
//#define EDID_CLK_DIVIDE_M1          2               // EDID I2C clock = sysclk / (1+EDID_CLK_DIVIDE_M1).
#define EDID_AUTO_CEC_ENABLE        0//1
#define ACR_MODE            0                       // Select which ACR scheme:
                                                    // 0=Analog PLL based ACR;
                                                    // 1=Digital ACR.

#define EDID_IN_FILTER_MODE         7               // 0=No in filter; 1=Filter, use every sample;
                                                    // 2=Filter, use 1 sample out of 2 ...; 7=Filter, use 1 sample out of 7.

#define EDID_CEC_ID_ADDR            0x00a100a0      // EDID address offsets for storing 2-byte of Physical Address
//G9-AUDIO FIFO hardware: 32bit i2s in,16bit i2s out
#define I2S_32BIT_128FS_OUTPUT	0
#define I2S_32BIT_256FS_OUTPUT	1
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
#define AUDIO_OUTPUT_SELECT I2S_32BIT_128FS_OUTPUT
#else
#define AUDIO_OUTPUT_SELECT I2S_32BIT_256FS_OUTPUT
#endif
#define Wr_reg_bits(reg, val, start, len) \
  WRITE_MPEG_REG(reg, (READ_MPEG_REG(reg) & ~(((1L<<(len))-1)<<(start)))|((unsigned int)(val) << (start)))

#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
#undef WRITE_APB_REG
#undef WRITE_APB_REG_BITS
#undef READ_APB_REG
#undef READ_APB_REG_BITS

#define WRITE_APB_REG(x,val)					W_APB_REG(x,val)
#define WRITE_APB_REG_BITS(x,val,start,length)	W_APB_REG_BITS(x,val,start,length)
#define READ_APB_REG(x)							R_APB_REG(x)
#define READ_APB_REG_BITS(x,start,length)		R_APB_REG_BITS(x,start,length)
#endif
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
#define P_HDMIRX_CTRL_PORT	0xc800e000//0xc800e008  // TOP CTRL_PORT: 0xc800e008; DWC CTRL_PORT: 0xc800e018
#else
#define P_HDMIRX_CTRL_PORT    0xc800e008  // TOP CTRL_PORT: 0xc800e008; DWC CTRL_PORT: 0xc800e018
#endif
#define HDMIRX_ADDR_PORT	(0xe000)  // TOP ADDR_PORT: 0xc800e000; DWC ADDR_PORT: 0xc800e010
#define HDMIRX_DATA_PORT	(0xe004)  // TOP DATA_PORT: 0xc800e004; DWC DATA_PORT: 0xc800e014
#define HDMIRX_CTRL_PORT	(0xe008)  // TOP CTRL_PORT: 0xc800e008; DWC CTRL_PORT: 0xc800e018

static DEFINE_SPINLOCK(reg_rw_lock);

static int auto_aclk_mute = 2;
MODULE_PARM_DESC(auto_aclk_mute, "\n auto_aclk_mute \n");
module_param(auto_aclk_mute, int, 0664);

static int aud_avmute_en = 1;
MODULE_PARM_DESC(aud_avmute_en, "\n aud_avmute_en \n");
module_param(aud_avmute_en, int, 0664);

static int aud_mute_sel = 0;
MODULE_PARM_DESC(aud_mute_sel, "\n aud_mute_sel \n");
module_param(aud_mute_sel, int, 0664);
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
static bool pwr_gpio_pull_down = false;//true;
#else
static bool pwr_gpio_pull_down = true;
#endif
MODULE_PARM_DESC(pwr_gpio_pull_down, "\n pwr_gpio_pull_down \n");
module_param(pwr_gpio_pull_down, bool, 0664);

static int edid_clock_divide = 9;
MODULE_PARM_DESC(edid_clock_divide, "\n edid_clock_divide \n");
module_param(edid_clock_divide, int, 0664);

//sel 0 SCDC control; sel 1\2 enable when detected scramble data; sel 3 force scramble
static int scramble_sel = 1;
MODULE_PARM_DESC(scramble_sel, "\n scramble_sel \n");
module_param(scramble_sel, int, 0664);

bool multi_port_edid_enable = true;
MODULE_PARM_DESC(multi_port_edid_enable, "\n multi_port_edid_enable \n");
module_param(multi_port_edid_enable, bool, 0664);

static int mpll_ctl_setting = 0x200;//0x302;
MODULE_PARM_DESC(mpll_ctl_setting, "\n mpll_ctl_setting \n");
module_param(mpll_ctl_setting, int, 0664);

static bool new_phy_config = false;
MODULE_PARM_DESC(new_phy_config, "\n new_phy_config \n");
module_param(new_phy_config, bool, 0664);

static bool hdcp_enable = 1;
MODULE_PARM_DESC(hdcp_enable, "\n hdcp_enable \n");
module_param(hdcp_enable, bool, 0664);

static bool use_audioresample_reset = false;
MODULE_PARM_DESC(use_audioresample_reset, "\n use_audioresample_reset \n");
module_param(use_audioresample_reset, bool, 0664);

static int hdmi_mode_hyst = 6;
MODULE_PARM_DESC(hdmi_mode_hyst, "\n hdmi_mode_hyst \n");
module_param(hdmi_mode_hyst, int, 0664);
int eq_setting_ch0 = 4;
int eq_setting_ch1 = 4;
int eq_setting_ch2 = 4;
int port0_eq_setting_ch0 = 4;
int port0_eq_setting_ch1 = 4;
int port0_eq_setting_ch2 = 4;
int port1_eq_setting_ch0 = 4;
int port1_eq_setting_ch1 = 4;
int port1_eq_setting_ch2 = 4;
int port2_eq_setting_ch0 = 4;
int port2_eq_setting_ch1 = 4;
int port2_eq_setting_ch2 = 4;


/**
 * Read data from HDMI RX CTRL
 * @param[in] addr register address
 * @return data read value
 */
uint32_t hdmirx_rd_dwc(uint16_t addr)
{
    ulong flags;
	unsigned long dev_offset = 0x10;    // TOP ADDR_PORT: 0xc800e000; DWC ADDR_PORT: 0xc800e010
	unsigned long data;
	spin_lock_irqsave(&reg_rw_lock, flags);
	WRITE_APB_REG((HDMIRX_ADDR_PORT | dev_offset), addr);
	data = READ_APB_REG((HDMIRX_DATA_PORT | dev_offset));
	spin_unlock_irqrestore(&reg_rw_lock, flags);
	return (data);
} /* hdmirx_rd_DWC */

/**
 * Write data to HDMI RX CTRL
 * @param[in] addr register address
 * @param[in] data new register value
 */
void hdmirx_wr_dwc(uint16_t addr, uint32_t data)
{
	/* log_info("%04X:%08X", addr, data); */
    ulong flags;
	unsigned long dev_offset = 0x10;    // TOP ADDR_PORT: 0xc800e000; DWC ADDR_PORT: 0xc800e010
    spin_lock_irqsave(&reg_rw_lock, flags);
	WRITE_APB_REG((HDMIRX_ADDR_PORT | dev_offset), addr);
	WRITE_APB_REG((HDMIRX_DATA_PORT | dev_offset), data);
    spin_unlock_irqrestore(&reg_rw_lock, flags);
	//printk("Write DWC Reg 0x%08x <= 0x%08x\n", addr, data);
} /* hdmirx_wr_only_DWC */



uint32_t hdmirx_rd_bits_dwc( uint16_t addr, uint32_t mask)
{
	return get(hdmirx_rd_dwc(addr), mask);
}


void hdmirx_wr_bits_dwc( uint16_t addr, uint32_t mask, uint32_t value)
{
	hdmirx_wr_dwc(addr, set(hdmirx_rd_dwc(addr), mask, value));
}

uint16_t hdmirx_rd_phy(uint8_t reg_address)
{
	int cnt = 0;

	//hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_SLAVE, 0x39);
	hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_ADDRESS, reg_address);
	hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_OPERATION, 0x02); /* read op */
	do{ //wait i2cmpdone
		if(hdmirx_rd_dwc(HDMIRX_DWC_HDMI_ISTS)&(1<<28)){
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_ICLR, 1<<28);
		break;
	}
	cnt++;
	if(cnt>10000){
		printk("[HDMIRX error]: %s(%x,%x) timeout\n", __func__, 0x39, reg_address);
		break;
	}
	}while(1);

	return (uint16_t)(hdmirx_rd_dwc(HDMIRX_DWC_I2CM_PHYG3_DATAI));
}


int hdmirx_wr_phy(uint8_t reg_address, uint16_t data)
{
	int error = 0;
	int cnt = 0;

	//hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_SLAVE, 0x39);
	hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_ADDRESS, reg_address);
	hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_DATAO, data);
	hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_OPERATION, 0x01); /* write op */

	do{ //wait i2cmpdone
		if(hdmirx_rd_dwc(HDMIRX_DWC_HDMI_ISTS)&(1<<28)){
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_ICLR, 1<<28);
		break;
	}
	cnt++;
	if(cnt>10000){
		printk("[HDMIRX error]: %s(%x,%x,%x) timeout\n", __func__, 0x39, reg_address, data);
		break;
	}
	}while(1);
	//printk("Write PHY Reg 0x%08x <= 0x%08x\n", reg_address, data);
	return error;
}



void hdmirx_wr_top (unsigned long addr, unsigned long data)
{
    ulong flags;
	unsigned long dev_offset = 0;       // TOP ADDR_PORT: 0xc800e000; DWC ADDR_PORT: 0xc800e010
    spin_lock_irqsave(&reg_rw_lock, flags);
	WRITE_APB_REG((HDMIRX_ADDR_PORT | dev_offset), addr);
	WRITE_APB_REG((HDMIRX_DATA_PORT | dev_offset), data);
    spin_unlock_irqrestore(&reg_rw_lock, flags);

	//printk("Write TOP Reg 0x%08lx <= 0x%08lx\n", addr, data);
} /* hdmirx_wr_only_TOP */

unsigned long hdmirx_rd_top (unsigned long addr)
{
    ulong flags;
	unsigned long dev_offset = 0;       // TOP ADDR_PORT: 0xc800e000; DWC ADDR_PORT: 0xc800e010
	unsigned long data;
    spin_lock_irqsave(&reg_rw_lock, flags);
	WRITE_APB_REG((HDMIRX_ADDR_PORT | dev_offset), addr);
	data = READ_APB_REG((HDMIRX_DATA_PORT | dev_offset));
    spin_unlock_irqrestore(&reg_rw_lock, flags);
	return (data);
} /* hdmirx_rd_TOP */


/**************************
    phy functions
***************************/
/** PHY GEN 3 I2C slave address (Testchip E205) */
//#define PHY_GEN3_I2C_SLAVE_ADDR			(0x39UL)
#define PHY_GEN3_I2C_SLAVE_ADDR			(0x69UL)
//#define PHY_GEN3_GLUE_I2C_SLAVE_ADDR	(0x48UL)
#if 0
static void phy_set_cfgclk_freq(int cfgclk)
{
	hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, MSK(2,4), cfgclk);
}
#endif
#if 0	//no use , init set 1
static void hdmirx_phy_svsretmode(int enable)
{
	hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, MSK(1,6), enable);
}
#endif
void hdmirx_phy_reset(bool enable)
{
	hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, MSK(1,0), enable);
}
void hdmirx_phy_pddq(int enable)
{
	hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, MSK(1,1), enable);
}
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
void hdmirx_phy_fast_switching( int enable)
{
	hdmirx_wr_phy(REG_HDMI_PHY_SYSTEM_CONFIG, hdmirx_rd_phy(REG_HDMI_PHY_SYSTEM_CONFIG) |
		((enable & 1) << 11) | rx.port | rx.phy.port_select_ovr_en<<2);

	hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, hdmirx_rd_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL) |
		rx.port << 2); // DEFAULT: {27'd0, 3'd0, 2'd1}
}
#endif
void hdmirx_phy_power_down(void)
{
	hdmirx_phy_pddq(1);
	//hdmirx_phy_svsretmode(1);		//always 1
	hdmirx_phy_reset(1);
}
void hdmirx_phy_configuration(void)
{
	hdmirx_phy_pddq(1);
	//hdmirx_phy_svsretmode(1);		//always 1
	hdmirx_phy_reset(0);
}
void hdmirx_phy_power_up(void)
{
	hdmirx_phy_pddq(0);
	//hdmirx_phy_svsretmode(1); 	//always 1
	hdmirx_phy_reset(0);
}
#if 0
static void hdmirx_phy_restart(void)
{
    hdmirx_phy_reset(true);
    mdelay(1);
    hdmirx_phy_reset(false);
    mdelay(1);
    hdmirx_phy_pddq(1);
    mdelay(1);
    phy_wrapper_svsretmode(1);
#if 1
    phy_set_cfgclk_freq(1);
#endif
    /* power up */
    hdmirx_phy_pddq(0);

}
#endif
/**
 * Close device
 * @param[in,out] ctx context information
 * @return error code
 */
#if 0
int hdmi_rx_phy_close(struct hdmi_rx_phy *ctx)
{
	int error = 0;

	if (ctx == 0)
	{
		return -EINVAL;
	}
	if (ctx->status == 0)
	{		/* is closed? */
		return -EPERM;
	}
	ctx->status = 0;
	ctx->bsp_reset(true);
	if(ctx->generation == 3)
	{
		ctx->bsp_pddq(1);
	}
	return error;
}
/**
 * Close device
 * @param[in,out] ctx context information
 * @return error code
 */
int hdmi_rx_phy_try_force_close(struct hdmi_rx_phy *ctx)
{
	int error = 0;

	if (ctx == 0)
	{
		return -EINVAL;
	}
	ctx->status = 0;
	ctx->bsp_reset(true);
	if(ctx->generation == 3)
	{
		ctx->bsp_pddq(1);
	}
	return error;
}
#endif

/**************************
    hw functions
***************************/
void hdmirx_phy_init_reset(int rx_port_sel, int dcm)
{
	unsigned int data32;
	printk("hdmirx_phy_init_reset\n");
	// PDDQ = 1'b1; PHY_RESET = 1'b1;
	data32	= 0;
	data32 |= 1 			<< 6;	// [6]		physvsretmodez
	data32 |= 1 			<< 4;	// [5:4]	cfgclkfreq
	data32 |= rx_port_sel	<< 2;	// [3:2]	portselect
	data32 |= 1 			<< 1;	// [1]		phypddq
	data32 |= 1 			<< 0;	// [0]		phyreset
	hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,	 data32); // DEFAULT: {27'd0, 3'd0, 2'd1}
	mdelay(1);

	// PDDQ = 1'b1; PHY_RESET = 1'b0;
	data32	= 0;
	data32 |= 1 			<< 6;	// [6]		physvsretmodez
	data32 |= 1 			<< 4;	// [5:4]	cfgclkfreq
	data32 |= rx_port_sel	<< 2;	// [3:2]	portselect
	data32 |= 1 			<< 1;	// [1]		phypddq
	data32 |= 0 			<< 0;	// [0]		phyreset
	hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,	 data32); // DEFAULT: {27'd0, 3'd0, 2'd1}
	if(rx_port_sel == 0) {
		printk("port0-0\n");
		hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, port0_eq_setting_ch0);
		hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, port0_eq_setting_ch1);
		hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, port0_eq_setting_ch2);
	} else if(rx_port_sel == 1) {
		printk("port1-1\n");
		hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, port1_eq_setting_ch0);
		hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, port1_eq_setting_ch1);
		hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, port1_eq_setting_ch2);
	} else {
		printk("port2-2\n");
		hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, port2_eq_setting_ch0);
		hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, port2_eq_setting_ch1);
		hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, port2_eq_setting_ch2);
	}
	hdmirx_wr_phy(HDMIRX_PHY_MAIN_FSM_OVERRIDE2, 0x40);

	data32	= 0;
	data32 |= 1 			<< 6;	// [6]		physvsretmodez
	data32 |= 1 			<< 4;	// [5:4]	cfgclkfreq
	data32 |= rx_port_sel	<< 2;	// [3:2]	portselect
	data32 |= 0 			<< 1;	// [1]		phypddq
	data32 |= 0 			<< 0;	// [0]		phyreset
	hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,	 data32); // DEFAULT: {27'd0, 3'd0, 2'd1}

}
void hdmirx_phy_init(int rx_port_sel, int dcm)
{
	unsigned int data32;
	printk("hdmirx_phy_init\n");
    // PDDQ = 1'b1; PHY_RESET = 1'b1;
    data32  = 0;
    data32 |= 1             << 6;   // [6]      physvsretmodez
    data32 |= 1             << 4;   // [5:4]    cfgclkfreq
    data32 |= rx_port_sel   << 2;   // [3:2]    portselect
    data32 |= 1             << 1;   // [1]      phypddq
    data32 |= 1             << 0;   // [0]      phyreset
    hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,    data32); // DEFAULT: {27'd0, 3'd0, 2'd1}
    mdelay(1);

    // PDDQ = 1'b1; PHY_RESET = 1'b0;
    data32  = 0;
    data32 |= 1             << 6;   // [6]      physvsretmodez
    data32 |= 1             << 4;   // [5:4]    cfgclkfreq
    data32 |= rx_port_sel   << 2;   // [3:2]    portselect
    data32 |= 1             << 1;   // [1]      phypddq
    data32 |= 0             << 0;   // [0]      phyreset
    hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,    data32); // DEFAULT: {27'd0, 3'd0, 2'd1}

    // Configuring PHY's MPLL
    hdmirx_wr_phy(MPLL_PARAMETERS2,    0x2d20);
    hdmirx_wr_phy(MPLL_PARAMETERS3,    0x3713);
    hdmirx_wr_phy(MPLL_PARAMETERS4,    0x24da);
    hdmirx_wr_phy(MPLL_PARAMETERS5,    0x5492);
    hdmirx_wr_phy(MPLL_PARAMETERS6,    0x4b0d);
    hdmirx_wr_phy(MPLL_PARAMETERS7,    0x4760);
    hdmirx_wr_phy(MPLL_PARAMETERS8,    0x008c);
	hdmirx_wr_phy(MPLL_PARAMETERS9,    0x0002);
    hdmirx_wr_phy(MPLL_PARAMETERS10,   0x2d20);
    hdmirx_wr_phy(MPLL_PARAMETERS11,    0x2e31);
    hdmirx_wr_phy(MPLL_PARAMETERS12,    0x4b64);
    hdmirx_wr_phy(MPLL_PARAMETERS13,    0x2493);
    hdmirx_wr_phy(MPLL_PARAMETERS14,    0x676d);
    hdmirx_wr_phy(MPLL_PARAMETERS15,    0x23e0);
	hdmirx_wr_phy(MPLL_PARAMETERS16,    0x001b);
    hdmirx_wr_phy(MPLL_PARAMETERS17,    0x2218);
    hdmirx_wr_phy(MPLL_PARAMETERS18,    0x1b25);
	hdmirx_wr_phy(MPLL_PARAMETERS19,    0x2492);
    hdmirx_wr_phy(MPLL_PARAMETERS20,    0x48ea);
    hdmirx_wr_phy(MPLL_PARAMETERS21,    0x0011);

	// Configuring I2C to work in fastmode
	hdmirx_wr_dwc(HDMIRX_DWC_I2CM_PHYG3_MODE,    0x1);

	/* write timebase override and override enable */
	hdmirx_wr_phy(OVL_PROT_CTRL, 0xa); //disable overload protect for Philips DVD ???


	data32  = 0;
	data32 |= 0                     << 15;  // [15]     mpll_short_power_up
	data32 |= 0                     << 13;  // [14:13]  mpll_mult
	data32 |= 0                     << 12;  // [12]     dis_off_lp
	//data32 |= rx.phy.fast_switching << 11;  // [11]     fast_switching
	data32 |= 0						<< 11;  // [11]     fast_switching
	data32 |= 0                     << 10;  // [10]     bypass_afe
	data32 |= rx.phy.fsm_enhancement<< 9;   // [9]      fsm_enhancement
	data32 |= 0                     << 8;   // [8]      low_freq_eq
	data32 |= 0                     << 7;   // [7]      bypass_aligner
	data32 |= dcm                   << 5;   // [6:5]    color_depth: 0=8-bit; 1=10-bit; 2=12-bit; 3=16-bit.
	data32 |= 0                     << 3;   // [4:3]    sel_tmdsclk: 0=Use chan0 clk; 1=Use chan1 clk; 2=Use chan2 clk; 3=Rsvd.
	data32 |= rx.phy.port_select_ovr_en   << 2;   // [2]      port_select_ovr_en
	data32 |= rx_port_sel           << 0;   // [1:0]    port_select_ovr
	hdmirx_wr_phy(REG_HDMI_PHY_SYSTEM_CONFIG,
	(rx.phy.phy_system_config_force_val != 0) ? rx.phy.phy_system_config_force_val : data32);

	hdmirx_wr_phy(REG_HDMI_PHY_CMU_CONFIG,
	(rx.phy.phy_cmu_config_force_val != 0) ? rx.phy.phy_cmu_config_force_val :
	((rx.phy.lock_thres << 10) | (1 << 9) | (((1 << 9) - 1) & ((rx.phy.cfg_clk * 4) / 1000))));

	if(new_phy_config){
		hdmirx_wr_phy(HDMIRX_PHY_VOLTAGE_LEVEL,0x010a);
		hdmirx_wr_phy(HDMIRX_PHY_MPLL_CTRL, mpll_ctl_setting);
	}

	//hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, eq_setting_ch0);
	//hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, eq_setting_ch1);
	//hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, eq_setting_ch2);
	if(rx_port_sel == 0) {
		printk("port0-0\n");
		hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, port0_eq_setting_ch0);
		hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, port0_eq_setting_ch1);
		hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, port0_eq_setting_ch2);
	} else if(rx_port_sel == 1) {
		printk("port1-1\n");
		hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, port1_eq_setting_ch0);
		hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, port1_eq_setting_ch1);
		hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, port1_eq_setting_ch2);
	} else {
		printk("port2-2\n");
		hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, port2_eq_setting_ch0);
		hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, port2_eq_setting_ch1);
		hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, port2_eq_setting_ch2);
	}
	hdmirx_wr_phy(HDMIRX_PHY_MAIN_FSM_OVERRIDE2, 0x40);

#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
	// Write PHY register 0x0e, MHL&HDMI select
    data32  = 0;
    data32 |= (0                << 10); // [10]     ctrl_eq_peak
    data32 |= (0                << 9);  // [9]      hdmi_mhl_mode[1]
    data32 |= (0	<< 8);  // [8]      hdmi_mhl_mode[0]
    data32 |= (8                << 0);  // [7:0]    cdr_ctrl_char_cnt
    hdmirx_wr_phy(HDMIRX_PHY_CDR_CTRL_CNT,    data32);
#endif

    // PDDQ = 1'b0; PHY_RESET = 1'b0;
    data32  = 0;
    data32 |= 1             << 6;   // [6]      physvsretmodez
    data32 |= 1             << 4;   // [5:4]    cfgclkfreq
    data32 |= rx_port_sel   << 2;   // [3:2]    portselect
    data32 |= 0             << 1;   // [1]      phypddq
    data32 |= 0             << 0;   // [0]      phyreset
    hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,    data32); // DEFAULT: {27'd0, 3'd0, 2'd1}

}

void hdmirx_phy_hw_reset(void)
{
    hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, MSK(2,0), 0x3);
    mdelay(1);
    hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, MSK(2,0), 0x2);
    mdelay(1);
    hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, MSK(2,0), 0x0);
}

int hdmirx_interrupts_cfg( bool enable)
{
	int error = 0;

	if (enable) {
		/* set enable */
		hdmirx_wr_dwc(HDMIRX_DWC_PDEC_IEN_SET, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_CLK_IEN_SET, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_IEN_SET, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_MD_IEN_SET, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_IEN_SET, ~0);

	} else {
		/* clear enable */
		hdmirx_wr_dwc(HDMIRX_DWC_PDEC_IEN_CLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_CLK_IEN_CLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_IEN_CLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_MD_IEN_CLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_IEN_CLR, ~0);
		/* clear status */
		hdmirx_wr_dwc(HDMIRX_DWC_PDEC_ICLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_CLK_ICLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_ICLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_MD_ICLR, ~0);
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_ICLR, ~0);
	}
	return error;
}

int hdmirx_interrupts_hpd( bool enable)
{
	int error = 0;

	if (enable) {
		/* set enable */
		//hdmirx_wr_dwc(HDMIRX_DWC_PDEC_IEN_SET, DVIDET|AIF_CKS_CHG|AVI_CKS_CHG|VSI_CKS_CHG|PD_FIFO_NEW_ENTRY|PD_FIFO_OVERFL);
		//hdmirx_wr_dwc(HDMIRX_DWC_PDEC_IEN_SET, DVIDET|AVI_CKS_CHG|VSI_CKS_CHG);

		hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_IEN_SET, AFIF_OVERFL|AFIF_UNDERFL);
		//hdmirx_wr_dwc(HDMIRX_DWC_MD_IEN_SET, VIDEO_MODE);

		//hdmirx_wr_dwc(HDMIRX_DWC_HDMI_IEN_SET, AKSV_RCV|DCM_CURRENT_MODE_CHG|CLK_CHANGE);
		//hdmirx_wr_dwc(HDMIRX_DWC_HDMI_IEN_SET, DCM_CURRENT_MODE_CHG|CLK_CHANGE);
	} else {
		/* clear enable */
		hdmirx_wr_dwc(HDMIRX_DWC_PDEC_IEN_CLR, DVIDET|AVI_CKS_CHG|VSI_CKS_CHG);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_IEN_CLR, AFIF_OVERFL|AFIF_UNDERFL);
		hdmirx_wr_dwc(HDMIRX_DWC_MD_IEN_CLR, VIDEO_MODE);
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_IEN_CLR, DCM_CURRENT_MODE_CHG|CLK_CHANGE);
		/* clear status */
		hdmirx_wr_dwc(HDMIRX_DWC_PDEC_ICLR, DVIDET|AVI_CKS_CHG|VSI_CKS_CHG);
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_ICLR, AFIF_OVERFL|AFIF_UNDERFL);
		hdmirx_wr_dwc(HDMIRX_DWC_MD_ICLR, VIDEO_MODE);
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_ICLR, DCM_CURRENT_MODE_CHG|CLK_CHANGE);
	}
	return error;
}

#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
void hdmirx_audiopll_control(bool enable)
{
	if(enable){
		hdmirx_wr_top(HDMIRX_TOP_ACR_CNTL_STAT,hdmirx_rd_top(HDMIRX_TOP_ACR_CNTL_STAT)|(1<<11));
		WRITE_CBUS_REG(HHI_HDMIRX_AUD_PLL_CNTL6,(hdmirx_rd_phy(REG_HDMI_PHY_MAINFSM_STATUS1)>>9 & 3)<<28);
		printk("REG_HDMI_PHY_MAINFSM_STATUS1---%x\n",hdmirx_rd_phy(REG_HDMI_PHY_MAINFSM_STATUS1));
		WRITE_CBUS_REG(HHI_HDMIRX_AUD_PLL_CNTL5,0x0000002e);
		WRITE_CBUS_REG(HHI_HDMIRX_AUD_PLL_CNTL4,0x30000000);
		WRITE_CBUS_REG(HHI_HDMIRX_AUD_PLL_CNTL3,0x00000000);
		WRITE_CBUS_REG(HHI_HDMIRX_AUD_PLL_CNTL,0x40000000);
		WRITE_CBUS_REG(HHI_ADC_PLL_CNTL4,0x805);
		hdmirx_wr_top(HDMIRX_TOP_ACR_CNTL_STAT,hdmirx_rd_top(HDMIRX_TOP_ACR_CNTL_STAT)|(1<<11));
		if(use_audioresample_reset) {
			WRITE_CBUS_REG(AUD_RESAMPLE_CTRL0,(READ_CBUS_REG(AUD_RESAMPLE_CTRL0) | 0x10000000) & 0x7fffffff);
		}
	}else{
		//disable pll, into reset mode
		WRITE_CBUS_REG(HHI_HDMIRX_AUD_PLL_CNTL,0x20000000);
		if(use_audioresample_reset) {
			//reset resample module
			WRITE_CBUS_REG(AUD_RESAMPLE_CTRL0,(READ_CBUS_REG(AUD_RESAMPLE_CTRL0) | 0x80000000) & 0xefffffff);
		}
	}
}
#endif

void hdmirx_audio_enable(bool en)
{
    unsigned int val = hdmirx_rd_dwc(HDMIRX_DWC_AUD_SAO_CTRL);

    if (en) {
        if (val != AUDIO_OUTPUT_SELECT)
            hdmirx_wr_dwc(HDMIRX_DWC_AUD_SAO_CTRL, AUDIO_OUTPUT_SELECT);
    } else {
        if (val != 0x7ff)//disable i2s output by bit[5:8]
            hdmirx_wr_dwc(HDMIRX_DWC_AUD_SAO_CTRL, 0x7ff);
    }
}

int hdmirx_iaudioclk_domain_reset(void)
{
	int error = 0;

	//hdmirx_wr_bits_dwc(HDMIRX_DWC_DMI_SW_RST, IAUDIOCLK_DOMAIN_RESET, 1);
	//hdmirx_wr_bits_dwc(HDMIRX_DWC_DMI_SW_RST, IAUDIOCLK_DOMAIN_RESET, 0);
	return error;
}
int hdmirx_audio_fifo_rst(void)
{
	int error = 0;

	hdmirx_wr_bits_dwc(HDMIRX_DWC_AUD_FIFO_CTRL, AFIF_INIT, 1);
	hdmirx_wr_bits_dwc(HDMIRX_DWC_AUD_FIFO_CTRL, AFIF_INIT, 0);
	return error;
}

int hdmirx_control_clk_range(unsigned long min, unsigned long max)
{
	int error = 0;
	unsigned evaltime = 0;
	unsigned long ref_clk;

	ref_clk = rx.ctrl.md_clk;
	evaltime = (ref_clk * 4095) / 158000;
	min = (min * evaltime) / ref_clk;
	max = (max * evaltime) / ref_clk;
	hdmirx_wr_bits_dwc(HDMIRX_DWC_HDMI_CKM_F, MINFREQ, min);
	hdmirx_wr_bits_dwc(HDMIRX_DWC_HDMI_CKM_F, CKM_MAXFREQ, max);
	return error;
}

static int packet_init(void)
{
	int error = 0;
	/*
    data32  = 0;
    data32 |= (1     << 31);    // [31]     pfifo_store_filter_en
    data32 |= (1     << 26);    // [26]     pfifo_store_mpegs_if
    data32 |= (1     << 25);    // [25]     pfifo_store_aud_if
    data32 |= (1     << 24);    // [24]     pfifo_store_spd_if
    data32 |= (1     << 23);    // [23]     pfifo_store_avi_if
    data32 |= (1     << 22);    // [22]     pfifo_store_vs_if
    data32 |= (1     << 21);    // [21]     pfifo_store_gmtp
    data32 |= (1     << 20);    // [20]     pfifo_store_isrc2
    data32 |= (1     << 19);    // [19]     pfifo_store_isrc1
    data32 |= (1     << 18);    // [18]     pfifo_store_acp
    data32 |= (0     << 17);    // [17]     pfifo_store_gcp
    data32 |= (0     << 16);    // [16]     pfifo_store_acr
    data32 |= (0     << 14);    // [14]     gcpforce_clravmute
    data32 |= (0     << 13);    // [13]     gcpforce_setavmute
    data32 |= (0     << 12);    // [12]     gcp_avmute_allsps
    data32 |= (0     << 8);     // [8]      pd_fifo_fill_info_clr
    data32 |= (0     << 6);     // [6]      pd_fifo_skip
    data32 |= (0     << 5);     // [5]      pd_fifo_clr
    data32 |= (1     << 4);     // [4]      pd_fifo_we
    data32 |= (1     << 0);     // [0]      pdec_bch_en
    hdmirx_wr_reg(base_offset, HDMIRX_DEV_ID_DWC, HDMIRX_DWC_PDEC_CTRL, data32);
	*/
	hdmirx_wr_dwc(HDMIRX_DWC_PDEC_CTRL, PFIFO_STORE_FILTER_EN|PD_FIFO_WE|PDEC_BCH_EN);
	hdmirx_wr_dwc(HDMIRX_DWC_PDEC_ASP_CTRL, AUTO_VMUTE|AUTO_SPFLAT_MUTE);
	return error;
}

int hdmirx_packet_fifo_rst(void)
{
	int error = 0;

	hdmirx_wr_bits_dwc(HDMIRX_DWC_PDEC_CTRL, PD_FIFO_FILL_INFO_CLR|PD_FIFO_CLR, ~0);
	hdmirx_wr_bits_dwc(HDMIRX_DWC_PDEC_CTRL, PD_FIFO_FILL_INFO_CLR|PD_FIFO_CLR,  0);
	return error;
}

static void control_init_more(void)
{
	#define VSYNC_POLARITY      1                       // TX VSYNC polarity: 0=low active; 1=high active.
	unsigned long   data32;

    data32  = 0;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
    data32 |= (EDID_IN_FILTER_MODE				<< 14); // [16:14]	i2c_filter_mode
	data32 |= (((EDID_IN_FILTER_MODE==0)?0:1)	<< 13); // [   13]	i2c_filter_enable
#else
    data32 |= EDID_AUTO_CHECKSUM_ENABLE << 12;  // [   12]  auto_checksum_enable
#endif
    data32 |= EDID_AUTO_CEC_ENABLE      << 11;  // [   11]  auto_cec_enable
    data32 |= 0                         << 10;  // [   10]  scl_stretch_trigger_config
    data32 |= 0                         << 9;   // [    9]  force_scl_stretch_trigger
	data32 |= 1                         << 8;   // [    8]  scl_stretch_enable
    data32 |= edid_clock_divide << 0;   // [ 7: 0]  clk_divide_m1
    hdmirx_wr_top(HDMIRX_TOP_EDID_GEN_CNTL,  data32);

#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
	hdmirx_wr_top(HDMIRX_TOP_EDID_ADDR_CEC,	EDID_CEC_ID_ADDR);
#endif

#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
	data32=0;
	data32 |= (0							<< 28); // [28] 	cntl_vs_timing: 0=Detect VS rising; 1=Detect HS falling.
	data32 |= (0							<< 27); // [27] 	cntl_hs_timing: 0=Detect HS rising; 1=Detect HS falling.
	data32 |= (0							<< 24); // [26:24]	vid_data_map. 0={vid2, vid1, vid0}->{vid2, vid1, vid0}

	data32 |= (0							<< 19); // [19] 	cntl_422_mode : 0=Insert a repeat of every pixel; 1=Insert an average of every 2-pixel.
	hdmirx_wr_top(HDMIRX_TOP_VID_CNTL,	data32);

	data32 |= (0						<< 20); // [20] 	last420l_rdwin_mode. Control when to start read from FIFO for outputting 420 even or last line.
												//			0=use measured the previous line's start position minus last420l_rdwin_auto.
												//			1=use last420l_rdwin_manual to start the read window.
	data32 |= (0						<< 8);	// [19:8]	last420l_rdwin_manual
	data32 |= (0						<< 0);	// [7:0]	last420l_rdwin_auto
	hdmirx_wr_top(HDMIRX_TOP_VID_CNTL2,  data32);
#endif

#if 0
    data32  = 0;
    data32 |= VSYNC_POLARITY    << 3;   // [4:3]    vs_pol_adj_mode:0=invert input VS; 1=no invert; 2=auto convert to high active; 3=no invert.
    data32 |= 2                 << 1;   // [2:1]    hs_pol_adj_mode:0=invert input VS; 1=no invert; 2=auto convert to high active; 3=no invert.
    hdmirx_wr_dwc( HDMIRX_DWC_HDMI_SYNC_CTRL,     data32); // DEFAULT: {27'd0, 2'd0, 2'd0, 1'b0}
#endif

    data32  = 0;
    data32 |= 0     << 20;  // [20]     rg_block_off:1=Enable HS/VS/CTRL filtering during active video
    data32 |= 1     << 19;  // [19]     block_off:1=Enable HS/VS/CTRL passing during active video
    data32 |= 5     << 16;  // [18:16]  valid_mode
    data32 |= 0     << 12;  // [13:12]  ctrl_filt_sens
    data32 |= 3     << 10;  // [11:10]  vs_filt_sens
    data32 |= 0     << 8;   // [9:8]    hs_filt_sens
    data32 |= 2     << 6;   // [7:6]    de_measure_mode
    data32 |= 0     << 5;   // [5]      de_regen
    data32 |= 3     << 3;   // [4:3]    de_filter_sens
    hdmirx_wr_dwc(HDMIRX_DWC_HDMI_ERROR_PROTECT, data32); // DEFAULT: {11'd0, 1'b0, 1'b0, 3'd0, 2'd0, 2'd0, 2'd0, 2'd0, 2'd0, 1'b0, 2'd0, 3'd0}

    data32  = 0;
    data32 |= 0     << 8;   // [10:8]   hact_pix_ith
    data32 |= 0     << 5;   // [5]      hact_pix_src
    data32 |= 1     << 4;   // [4]      htot_pix_src
    hdmirx_wr_dwc(HDMIRX_DWC_MD_HCTRL1,  data32); // DEFAULT: {21'd0, 3'd1, 2'd0, 1'b0, 1'b1, 4'd0}

    data32  = 0;
    data32 |= 1     << 12;  // [14:12]  hs_clk_ith
    data32 |= 7     << 8;   // [10:8]   htot32_clk_ith
    data32 |= 1     << 5;   // [5]      vs_act_time
    data32 |= 3     << 3;   // [4:3]    hs_act_time
    data32 |= 0     << 0;   // [1:0]    h_start_pos
    hdmirx_wr_dwc(HDMIRX_DWC_MD_HCTRL2,  data32); // DEFAULT: {17'd0, 3'd2, 1'b0, 3'd1, 2'd0, 1'b0, 2'd0, 1'b0, 2'd2}
	//HDMI2.0 ??
	#define interlace_mode 1
    data32  = 0;
    data32 |= 1                 << 4;   // [4]      v_offs_lin_mode
    data32 |= 1                 << 1;   // [1]      v_edge
    data32 |= interlace_mode    << 0;   // [0]      v_mode
    hdmirx_wr_dwc(HDMIRX_DWC_MD_VCTRL,   data32); // DEFAULT: {27'd0, 1'b0, 2'd0, 1'b1, 1'b0}

    data32  = 0;
    data32 |= 1 << 10;  // [11:10]  vofs_lin_ith
    data32 |= 3 << 8;   // [9:8]    vact_lin_ith
    data32 |= 0 << 6;   // [7:6]    vtot_lin_ith
    data32 |= 7 << 3;   // [5:3]    vs_clk_ith
    data32 |= 2 << 0;   // [2:0]    vtot_clk_ith
    hdmirx_wr_dwc(HDMIRX_DWC_MD_VTH,     data32); // DEFAULT: {20'd0, 2'd2, 2'd0, 2'd0, 3'd2, 3'd2}

    data32  = 0;
    data32 |= 1 << 2;   // [2]      fafielddet_en
    data32 |= 0 << 0;   // [1:0]    field_pol_mode
    hdmirx_wr_dwc(HDMIRX_DWC_MD_IL_POL,  data32); // DEFAULT: {29'd0, 1'b0, 2'd0}

    data32  = 0;
    data32 |= 0                 << 1;   // [4:1]    man_vid_derepeat
  	if(hdmirx_de_repeat_enable)
		data32 |= 1             << 0;   // [0]      auto_derepeat
	else
		data32 |= 0				<< 0;	// disable auto de repeat
    hdmirx_wr_dwc(HDMIRX_DWC_HDMI_RESMPL_CTRL,   data32); // DEFAULT: {27'd0, 4'd0, 1'b1}
	data32	= 0;
	data32 |= (hdmirx_rd_dwc(HDMIRX_DWC_HDMI_MODE_RECOVER) & 0xf8000000);	// [31:27]	preamble_cnt_limit
	data32 |= (0	<< 24); // [25:24]	mr_vs_pol_adj_mode
	data32 |= (0	<< 18); // [18] 	spike_filter_en
	data32 |= (8	<< 13); // [17:13]	dvi_mode_hyst
	data32 |= (hdmi_mode_hyst	<< 8);	// [12:8]	hdmi_mode_hyst
	data32 |= (0	<< 6);	// [7:6]	hdmi_mode: 0=automatic
	data32 |= (0	<< 4);	// [5:4]	gb_det
	data32 |= (0	<< 2);	// [3:2]	eess_oess
	data32 |= (0	<< 0);	// [1:0]	sel_ctl01
	hdmirx_wr_dwc(HDMIRX_DWC_HDMI_MODE_RECOVER, data32);
}

static int control_init(unsigned port)
{
	int err = 0;
	unsigned evaltime = 0;

  	evaltime = (rx.ctrl.md_clk * 4095) / 158000;
	hdmirx_wr_dwc(HDMIRX_DWC_HDMI_OVR_CTRL, ~0);	/* enable all */
	hdmirx_wr_bits_dwc(HDMIRX_DWC_HDMI_SYNC_CTRL, VS_POL_ADJ_MODE, VS_POL_ADJ_AUTO);
	hdmirx_wr_bits_dwc(HDMIRX_DWC_HDMI_SYNC_CTRL, HS_POL_ADJ_MODE, HS_POL_ADJ_AUTO);
	hdmirx_wr_bits_dwc(HDMIRX_DWC_HDMI_CKM_EVLTM, EVAL_TIME, evaltime);
	hdmirx_control_clk_range(TMDS_CLK_MIN, TMDS_CLK_MAX);
	/* bit field shared between phy and controller */
	hdmirx_wr_bits_dwc(HDMIRX_DWC_HDMI_PCB_CTRL, INPUT_SELECT, port);
	hdmirx_wr_bits_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL, ((1 << 2) - 1) << 2, port);

	control_init_more();
	return err;
}

#define HDCP_KEY_WR_TRIES		(5)
static void hdmi_rx_ctrl_hdcp_config( const struct hdmi_rx_ctrl_hdcp *hdcp)
{
	int error = 0;
	unsigned i = 0;
	unsigned k = 0;
	hdmirx_wr_bits_dwc( HDMIRX_DWC_HDCP_CTRL, HDCP_ENABLE, 0);
	//hdmirx_wr_bits_dwc(ctx, HDMIRX_DWC_HDCP_CTRL, KEY_DECRYPT_ENABLE, 1);
	hdmirx_wr_bits_dwc( HDMIRX_DWC_HDCP_CTRL, KEY_DECRYPT_ENABLE, 0);
	hdmirx_wr_dwc(HDMIRX_DWC_HDCP_SEED, hdcp->seed);
	for (i = 0; i < HDCP_KEYS_SIZE; i += 2) {

		for (k = 0; k < HDCP_KEY_WR_TRIES; k++) {
			if (hdmirx_rd_bits_dwc( HDMIRX_DWC_HDCP_STS, HDCP_KEY_WR_OK_STS) != 0) {
				break;
			}
		}
		if (k < HDCP_KEY_WR_TRIES) {
			hdmirx_wr_dwc(HDMIRX_DWC_HDCP_KEY1, hdcp->keys[i + 0]);
			hdmirx_wr_dwc(HDMIRX_DWC_HDCP_KEY0, hdcp->keys[i + 1]);
		} else {
			error = -EAGAIN;
			break;
		}
	}
	hdmirx_wr_dwc(HDMIRX_DWC_HDCP_BKSV1, hdcp->bksv[0]);
	hdmirx_wr_dwc(HDMIRX_DWC_HDCP_BKSV0, hdcp->bksv[1]);
	hdmirx_wr_bits_dwc( HDMIRX_DWC_HDCP_RPT_CTRL, REPEATER, hdcp->repeat? 1 : 0);
	hdmirx_wr_dwc(HDMIRX_DWC_HDCP_RPT_BSTATUS, 0);	/* nothing attached downstream */

	hdmirx_wr_bits_dwc( HDMIRX_DWC_HDCP_CTRL, HDCP_ENABLE, 1);
}

#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
/**
 * Configure KSV list
 * @param[in,out] ctx context information
 * @param[in] list KSV list, 0: high order, 1: low order
 * @param[in] count device count, if too big MAX_DEVS_EXCEEDED will be set
 * @param[in] depth cascade depth, if too big MAX_CASCADE_EXCEEDED will be set
 * @return error code
 */
#ifdef HDMI_REPEATER
int hdmi_rx_ctrl_hdcp_ksv_list(struct hdmi_rx_ctrl *ctx, const uint32_t *list, unsigned count, unsigned depth)
{
	int error = 0;
	unsigned i = 0;
	unsigned k = 0;

	if (ctx == 0 || list == 0) {
		return -EINVAL;
	}
	if (ctx->status <= 0) {		/* is configured? */
		return -EPERM;
	}
	if (io_read(ctx, HDMIRX_DWC_HDCP_RPT_CTRL, REPEATER) == 0) {
		error = -EPERM;			/* not configured in repeater mode */
	}
	if (io_read(ctx, HDMIRX_DWC_HDCP_RPT_CTRL, WAITING_KSV) == 0) {
		error = -EAGAIN;		/* not waiting for KSV list writing */
	}
	if (count <= MAX_DEVICES && count <= KSV_FIFO_SIZE) {
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, MAX_DEVS_EXCEEDED, 0);
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, DEVICE_COUNT, count);
	} else {
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, MAX_DEVS_EXCEEDED, 1);
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, DEVICE_COUNT, 0);
		count = 0;		/* don't write KSV list */
	}
	if (depth <= MAX_CASCADE) {
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, MAX_CASCADE_EXCEEDED, 0);
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, DEPTH, depth);
	} else {
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, MAX_CASCADE_EXCEEDED, 1);
		error |= io_write(ctx, HDMIRX_DWC_HDCP_RPT_BSTATUS, DEPTH, 0);
		count = 0;		/* don't write KSV list */
	}
	for (i = 0; i < count; i++) {
		for (k = 0; k < HDCP_KEY_WR_TRIES; k++) {
			if (io_read(ctx, HDMIRX_DWC_HDCP_RPT_CTRL, KSV_HOLD) != 0) {
				break;
			}
		}
		if (k < HDCP_KEY_WR_TRIES) {
			ctx->bsp_write(HDMIRX_DWC_HDCP_RPT_KSVFIFOCTRL, i);
			ctx->bsp_write(HDMIRX_DWC_HDCP_RPT_KSVFIFO1, list[i * 2 + 0]);
			ctx->bsp_write(HDMIRX_DWC_HDCP_RPT_KSVFIFO0, list[i * 2 + 1]);
		} else {
			error = -EAGAIN;
			break;
		}
	}
	if (error == 0) {
		error = io_write(ctx, HDMIRX_DWC_HDCP_RPT_CTRL, KSVLIST_READY, 1);
	}
	return error;
}
#endif
#endif
#if 0
static void hdmi_rx_ctrl_hpd(bool enable)
{
	hdmirx_wr_bits_dwc(HDMIRX_DWC_HDMI_SETUP_CTRL, HOT_PLUG_DETECT, enable? 1 : 0);
}
#endif

void hdmirx_set_hpd(int port, unsigned char val)
{
#if (MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV)
#ifdef USE_GPIO_FOR_HPD
    int bitpos = 1;
    switch(port){
        case 0:
            bitpos=1;
            break;
        case 1:
            bitpos=5;
            break;
        case 2:
            bitpos=9;
            break;
        case 3:
            bitpos=13;
            break;
    }
    if(val){
        WRITE_CBUS_REG(PREG_PAD_GPIO5_O, READ_CBUS_REG(PREG_PAD_GPIO5_O) & (~(1<<bitpos)));
    }
    else{
        WRITE_CBUS_REG(PREG_PAD_GPIO5_O, READ_CBUS_REG(PREG_PAD_GPIO5_O) | (1<<bitpos));
    }
#else
    if(val){
        hdmirx_wr_top( HDMIRX_TOP_HPD_PWR5V,  hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V)|(1<<rx.port));
    }
    else{
        hdmirx_wr_top( HDMIRX_TOP_HPD_PWR5V,  hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V)&(~(1<<rx.port)));
    }
#endif
#endif
//G9TV
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9TV)
#ifdef USE_GPIO_FOR_HPD
    int bitpos = 1;
    switch(port){
        case 0:
            bitpos=5;
            break;
        case 1:
            bitpos=9;
            break;
        case 2:
            bitpos=13;
            break;
        case 3:
            bitpos=17;
            break;
    }
    if(val){
        WRITE_CBUS_REG(PREG_PAD_GPIO0_O, READ_CBUS_REG(PREG_PAD_GPIO0_O) & (~(1<<bitpos)));
    }
    else{
        WRITE_CBUS_REG(PREG_PAD_GPIO0_O, READ_CBUS_REG(PREG_PAD_GPIO0_O) | (1<<bitpos));
    }
#else
    if(val){
        hdmirx_wr_top( HDMIRX_TOP_HPD_PWR5V,  hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V)&(~(1<<port)));
    }
    else{
		hdmirx_wr_top( HDMIRX_TOP_HPD_PWR5V,  hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V)|(1<<port));
    }
#endif
#endif

	//G9TV
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9BB)
#ifdef USE_GPIO_FOR_HPD
		int bitpos = 1;
		switch(port){
			case 0:
				bitpos=5;
				break;
			case 1:
				bitpos=9;
				break;
			case 2:
				bitpos=13;
				break;
			case 3:
				bitpos=17;
				break;
		}
		if(val){
			WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) | (1<<bitpos));
			//WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) & (~(1<<bitpos)));
		}
		else{
			WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) & (~(1<<bitpos)));
			//WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) | (1<<bitpos));
		}
#else
		if(val){
			hdmirx_wr_top( HDMIRX_TOP_HPD_PWR5V,  hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V)&(~(1<<port)));
		}
		else{
			hdmirx_wr_top( HDMIRX_TOP_HPD_PWR5V,  hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V)|(1<<port));
		}
#endif
#endif

    hdmirx_print("%s(%d,%d)\n", __func__, port, val);

}


static void control_reset(unsigned char seq)
{
    unsigned long   data32;

    if (seq == 0) {
        //DWC reset default to be active, until reg HDMIRX_TOP_SW_RESET[0] is set to 0.
        //hdmirx_rd_check_reg(HDMIRX_DEV_ID_TOP, HDMIRX_TOP_SW_RESET, 0x1, 0x0);
        // IP reset
        //hdmirx_wr_top( HDMIRX_TOP_SW_RESET, 0x3f);
        //mdelay(50);
        // Release IP reset
        hdmirx_wr_top(HDMIRX_TOP_SW_RESET, 0x0);

		/* add delay for audio problem */
        mdelay(2);

		// Enable functional modules
    	data32  = 0;
		#ifdef CEC_FUNC_ENABLE
    	data32 |= 1 << 5;   // [5]      cec_enable
		#else
		data32 |= 0 << 5;   // [5]      cec_enable
		#endif
    	data32 |= 1 << 4;   // [4]      aud_enable
		data32 |= 1 << 3;   // [3]      bus_enable
		data32 |= 1 << 2;   // [2]      hdmi_enable
		data32 |= 1 << 1;   // [1]      modet_enable
		data32 |= 1 << 0;   // [0]      cfg_enable
		hdmirx_wr_dwc(HDMIRX_DWC_DMI_DISABLE_IF, data32);    // DEFAULT: {31'd0, 1'b0}

    	mdelay(1);
    	// Reset functional modules
    	hdmirx_wr_dwc(HDMIRX_DWC_DMI_SW_RST,     0x0000007F);

    	// RX Reset
    }
    else{
    data32  = 0;
    data32 |= 0 << 5;   // [5]      cec_enable
    data32 |= 0 << 4;   // [4]      aud_enable
    data32 |= 0 << 3;   // [3]      bus_enable
    data32 |= 0 << 2;   // [2]      hdmi_enable
    data32 |= 0 << 1;   // [1]      modet_enable
    data32 |= 1 << 0;   // [0]      cfg_enable
    hdmirx_wr_dwc(HDMIRX_DWC_DMI_DISABLE_IF, data32);    // DEFAULT: {31'd0, 1'b0}

    mdelay(1);

    //--------------------------------------------------------------------------
    // Bring up RX
    //--------------------------------------------------------------------------
    data32  = 0;
#ifdef CEC_FUNC_ENABLE
	data32 |= 1 << 5;   // [5]      cec_enable
#else
    data32 |= 0 << 5;   // [5]      cec_enable
#endif
    data32 |= 1 << 4;   // [4]      aud_enable
    data32 |= 1 << 3;   // [3]      bus_enable
    data32 |= 1 << 2;   // [2]      hdmi_enable
    data32 |= 1 << 1;   // [1]      modet_enable
    data32 |= 1 << 0;   // [0]      cfg_enable
    hdmirx_wr_dwc(HDMIRX_DWC_DMI_DISABLE_IF, data32);    // DEFAULT: {31'd0, 1'b0}

    mdelay(1);
    }
}

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9TV)
void hdmirx_set_pinmux(void)
{
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_10 , READ_CBUS_REG(PERIPHS_PIN_MUX_10 )|
				((1 << 0)  |   // pm_gpioW_20_hdmirx_scl_D
				(1 << 0)   |   // pm_gpioW_19_hdmirx_sda_D
				(1 << 1)   |   // pm_gpioW_18_hdmirx_5v_D
				(1 << 2)   |   // pm_gpioW_17_hdmirx_hpd_D
				(1 << 3)   |   // pm_gpioW_16_hdmirx_scl_C
				(1 << 3)   |   // pm_gpioW_15_hdmirx_sda_C
				(1 << 4)   |   // pm_gpioW_14_hdmirx_5v_C
				(1 << 5)   |   // pm_gpioW_13_hdmirx_hpd_C
				(1 << 6)   |   // pm_gpioW_12_hdmirx_scl_B
				(1 << 6)   |   // pm_gpioW_11_hdmirx_sda_B
				(1 << 7)   |   // pm_gpioW_10_hdmirx_5v_B
				(1 << 8)   |   // pm_gpioW_9_hdmirx_hpd_B
				(1 << 9)   |   // pm_gpioW_8_hdmirx_scl_A
				(1 << 9)   |   // pm_gpioW_7_hdmirx_sda_A
				(1 << 10)   |   // pm_gpioW_6_hdmirx_5v_A
				(1 << 11)   |   // pm_gpioW_5_hdmirx_hpd_A
				(1 << 12)));     // pm_gpioW_4_hdmirx_cec
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_10, READ_CBUS_REG(PERIPHS_PIN_MUX_10 ) &
                (~(
                  (1<<29)
                  |(1<<28)
                  |(1<<27)
                  |(1<<26)
                  |(1<<25)
                  |(1<<24)
                  |(1<<23)
                  |(1<<22)
                  |(1<<13))));
    WRITE_CBUS_REG(PERIPHS_PIN_MUX_11, READ_CBUS_REG(PERIPHS_PIN_MUX_11 ) &
                (~(
                  (1<<24)
                  |(1<<23)
                  |(1<<22)
                  |(1<<21)
                  |(1<<20)
                  |(1<<19)
                  |(1<<12)
                  |(1<<11)
                  |(1<<10)
                  |(1<<9)
                  |(1<<8)
                  |(1<<7)
                  |(1<<6)
                  |(1<<5)
                  |(1<<4)
                  |(1<<3)
                  |(1<<2)
                  |(1<<1)
                  |(1<<0))));
    WRITE_CBUS_REG(PERIPHS_PIN_MUX_6, READ_CBUS_REG(PERIPHS_PIN_MUX_6 ) &
                (~((1<<21)
                  |(1<<20)
                  |(1<<19)
                  |(1<<18))));
#ifdef USE_GPIO_FOR_HPD
    if (pwr_gpio_pull_down)
        WRITE_CBUS_REG(PAD_PULL_UP_REG2, READ_CBUS_REG(PAD_PULL_UP_REG2) |
	            ((1<<0)|(1<<4)|(1<<8)|(1<<12)));

    WRITE_CBUS_REG(PERIPHS_PIN_MUX_10, READ_CBUS_REG(PERIPHS_PIN_MUX_10 ) &
                (~((1<<2)|(1<<5)|(1<<8)|(1<<11))));

    WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) &
                (~((1<<5)|(1<<9)|(1<<13)|(1<<17))));

    //WRITE_CBUS_REG(PREG_PAD_GPIO0_O, READ_CBUS_REG(PREG_PAD_GPIO0_O) &
                //(~((1<<5)|(1<<9)|(1<<13)|(1<<17))));
	WRITE_CBUS_REG(PREG_PAD_GPIO0_O, READ_CBUS_REG(PREG_PAD_GPIO0_O) |
                ((1<<17)|(1<<5)|(1<<9)|(1<<13)));
#endif

#if 0
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_1 , READ_CBUS_REG(PERIPHS_PIN_MUX_1 )|
				( (1 << 2)    |   // pm_gpioW_17_hdmirx_tmds_clk
				(1 << 1)    |   // pm_gpioW_18_hdmirx_pix_clk
				(1 << 0)));      // pm_gpioW_19_hdmirx_audmeas
#endif
}
#endif
#if(MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9BB)
void hdmirx_set_pinmux(void)
{
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_10 , READ_CBUS_REG(PERIPHS_PIN_MUX_10 )|
				((1 << 0)  |   // pm_gpioW_20_hdmirx_scl_D
				(1 << 0)   |   // pm_gpioW_19_hdmirx_sda_D
				(1 << 1)   |   // pm_gpioW_18_hdmirx_5v_D
				(1 << 2)   |   // pm_gpioW_17_hdmirx_hpd_D
				(1 << 3)   |   // pm_gpioW_16_hdmirx_scl_C
				(1 << 3)   |   // pm_gpioW_15_hdmirx_sda_C
				(1 << 4)   |   // pm_gpioW_14_hdmirx_5v_C
				(1 << 5)   |   // pm_gpioW_13_hdmirx_hpd_C
				(1 << 6)   |   // pm_gpioW_12_hdmirx_scl_B
				(1 << 6)   |   // pm_gpioW_11_hdmirx_sda_B
				(1 << 7)   |   // pm_gpioW_10_hdmirx_5v_B
				(1 << 8)   |   // pm_gpioW_9_hdmirx_hpd_B
				(1 << 9)   |   // pm_gpioW_8_hdmirx_scl_A
				(1 << 9)   |   // pm_gpioW_7_hdmirx_sda_A
				(1 << 10)   |   // pm_gpioW_6_hdmirx_5v_A
				(1 << 11)   |   // pm_gpioW_5_hdmirx_hpd_A
				(1 << 12)));     // pm_gpioW_4_hdmirx_cec
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_10, READ_CBUS_REG(PERIPHS_PIN_MUX_10 ) &
                (~(
                  (1<<29)
                  |(1<<28)
                  |(1<<27)
                  |(1<<26)
                  |(1<<25)
                  |(1<<24)
                  |(1<<23)
                  |(1<<22)
                  |(1<<13))));
    WRITE_CBUS_REG(PERIPHS_PIN_MUX_11, READ_CBUS_REG(PERIPHS_PIN_MUX_11 ) &
                (~(
                  //(1<<24)
                  (1<<23)
                  |(1<<22)
                  |(1<<21)
                 // |(1<<20)
                  |(1<<19)
                 // |(1<<12)
                 // |(1<<11)
                 // |(1<<10)
                 // |(1<<9)
                 // |(1<<8)
                  |(1<<7)
                  |(1<<6)
                  |(1<<5)
                  |(1<<4)
                  |(1<<3)
                  |(1<<2)
                  |(1<<1)
                  |(1<<0))));
    WRITE_CBUS_REG(PERIPHS_PIN_MUX_6, READ_CBUS_REG(PERIPHS_PIN_MUX_6 ) &
                (~((1<<21)
                  |(1<<20)
                  |(1<<19)
                  |(1<<18))));
#ifdef USE_GPIO_FOR_HPD
    if (pwr_gpio_pull_down)
        WRITE_CBUS_REG(PAD_PULL_UP_REG2, READ_CBUS_REG(PAD_PULL_UP_REG2) |
	            ((1<<0)|(1<<4)|(1<<8)|(1<<12)));

    WRITE_CBUS_REG(PERIPHS_PIN_MUX_10, READ_CBUS_REG(PERIPHS_PIN_MUX_10 ) &
                (~((1<<2)|(1<<5)|(1<<8)|(1<<11))));



    WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) &
                (~((1<<5)|(1<<9)|(1<<13)|(1<<17))));
    //WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) |
                //((1<<5)|(1<<9)|(1<<13)|(1<<17)));
#endif

#if 0
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_1 , READ_CBUS_REG(PERIPHS_PIN_MUX_1 )|
				( (1 << 2)    |   // pm_gpioW_17_hdmirx_tmds_clk
				(1 << 1)    |   // pm_gpioW_18_hdmirx_pix_clk
				(1 << 0)));      // pm_gpioW_19_hdmirx_audmeas
#endif

}
#endif
#if(MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV)
void hdmirx_set_pinmux(void)
{
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_0 , READ_CBUS_REG(PERIPHS_PIN_MUX_0 )|
				( (1 << 27)   |   // pm_gpioW_0_hdmirx_5V_A
				(1 << 26)   |   // pm_gpioW_1_hdmirx_HPD_A
				(1 << 25)   |   // pm_gpioW_2_hdmirx_scl_A
				(1 << 24)   |   // pm_gpioW_3_hdmirx_sda_A
				(1 << 23)   |   // pm_gpioW_4_hdmirx_5V_B
				(1 << 22)   |   // pm_gpioW_5_hdmirx_HPD_B
				(1 << 21)   |   // pm_gpioW_6_hdmirx_scl_B
				(1 << 20)   |   // pm_gpioW_7_hdmirx_sda_B
				(1 << 19)   |   // pm_gpioW_8_hdmirx_5V_C
				(1 << 18)   |   // pm_gpioW_9_hdmirx_HPD_C
				(1 << 17)   |   // pm_gpioW_10_hdmirx_scl_C
				(1 << 16)   |   // pm_gpioW_11_hdmirx_sda_C
				(1 << 15)   |   // pm_gpioW_12_hdmirx_5V_D
				(1 << 14)   |   // pm_gpioW_13_hdmirx_HPD_D
				(1 << 13)   |   // pm_gpioW_14_hdmirx_scl_D
				(1 << 12)   |   // pm_gpioW_15_hdmirx_sda_D
				(1 << 11)));     // pm_gpioW_16_hdmirx_cec
#ifdef USE_GPIO_FOR_HPD
    if (pwr_gpio_pull_down)
        WRITE_CBUS_REG(PAD_PULL_UP_REG2, READ_CBUS_REG(PAD_PULL_UP_REG2) |
	            ((1<<0)|(1<<4)|(1<<8)|(1<<12)));

    WRITE_CBUS_REG(PERIPHS_PIN_MUX_0, READ_CBUS_REG(PERIPHS_PIN_MUX_0 ) &
                (~((1<<26)|(1<<22)|(1<<18)|(1<<14))));

    WRITE_CBUS_REG(PREG_PAD_GPIO5_EN_N, READ_CBUS_REG(PREG_PAD_GPIO5_EN_N) &
                (~((1<<1)|(1<<5)|(1<<9)|(1<<13))));

    WRITE_CBUS_REG(PREG_PAD_GPIO5_O, READ_CBUS_REG(PREG_PAD_GPIO5_O) |
                ((1<<1)|(1<<5)|(1<<9)|(1<<13)));
#endif

#if 0
	WRITE_CBUS_REG(PERIPHS_PIN_MUX_1 , READ_CBUS_REG(PERIPHS_PIN_MUX_1 )|
				( (1 << 2)    |   // pm_gpioW_17_hdmirx_tmds_clk
				(1 << 1)    |   // pm_gpioW_18_hdmirx_pix_clk
				(1 << 0)));      // pm_gpioW_19_hdmirx_audmeas
#endif

}
#endif
void clk_off(void)
{

    WRITE_MPEG_REG(HHI_HDMIRX_AUD_CLK_CNTL, 0x0);
    WRITE_MPEG_REG(HHI_HDMIRX_CLK_CNTL,     0x0);
    Wr_reg_bits(HHI_GCLK_MPEG0, 0, 21, 1);  // Turn 0ff clk_hdmirx_pclk, also = sysclk
}

void clk_init(void)
{
    unsigned int data32;


    /* DWC clock enable */
    Wr_reg_bits(HHI_GCLK_MPEG0, 1, 21, 1);  // Turn on clk_hdmirx_pclk, also = sysclk
    // Enable APB3 fail on error
    *((volatile unsigned long *) P_HDMIRX_CTRL_PORT)          |= (1 << 15);   // APB3 to HDMIRX-TOP err_en
    *((volatile unsigned long *) (P_HDMIRX_CTRL_PORT+0x10))   |= (1 << 15);   // APB3 to HDMIRX-DWC err_en

    //turn on clocks: md, cfg...
	//G9 clk tree
	//fclk_div5 510M ----- mux sel = 3
	//fclk_div3 850M ----- mux sel = 2
	//fclk_div4 637M ----- mux sel = 1
	//XTAL 		24M  ----- mux sel = 0
    data32  = 0;
    data32 |= 0 << 25;  // [26:25] HDMIRX mode detection clock mux select: osc_clk
    data32 |= 1 << 24;  // [24]    HDMIRX mode detection clock enable
    data32 |= 0 << 16;  // [22:16] HDMIRX mode detection clock divider
    data32 |= 0 << 9;   // [10: 9] HDMIRX config clock mux select:
    data32 |= 1 << 8;   // [    8] HDMIRX config clock enable
    data32 |= 0 << 0;   // [ 6: 0] HDMIRX config clock divider:
    WRITE_MPEG_REG(HHI_HDMIRX_CLK_CNTL,     data32);

    data32  = 0;
    data32 |= 2             << 25;  // [26:25] HDMIRX ACR ref clock mux select: fclk_div5
    data32 |= rx.ctrl.acr_mode      << 24;  // [24]    HDMIRX ACR ref clock enable
    data32 |= 0             << 16;  // [22:16] HDMIRX ACR ref clock divider
    data32 |= 2             << 9;   // [10: 9] HDMIRX audmeas clock mux select: fclk_div5
    data32 |= 1             << 8;   // [    8] HDMIRX audmeas clock enable
    data32 |= 2             << 0;   // [ 6: 0] HDMIRX audmeas clock divider: 510/3 = 170MHz
    WRITE_MPEG_REG(HHI_HDMIRX_AUD_CLK_CNTL, data32);


    data32  = 0;
    data32 |= 1 << 17;  // [17]     audfifo_rd_en
    data32 |= 1 << 16;  // [16]     pktfifo_rd_en
    data32 |= 1 << 2;   // [2]      hdmirx_cecclk_en
    data32 |= 0 << 1;   // [1]      bus_clk_inv
    data32 |= 0 << 0;   // [0]      hdmi_clk_inv
    hdmirx_wr_top(HDMIRX_TOP_CLK_CNTL, data32);    // DEFAULT: {32'h0}

}

#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
void hdmirx_20_init(void)
{
	unsigned long data32;
    data32  = 0;
    data32 |= (1    << 12); // [12]     vid_data_checken
    data32 |= (1    << 11); // [11]     data_island_checken
    data32 |= (1    << 10); // [10]     gb_checken
    data32 |= (1    << 9);  // [9]      preamb_checken
    data32 |= (1    << 8);  // [8]      ctrl_checken
    data32 |= (1    << 4);  // [4]      scdc_enable
    data32 |= (scramble_sel    << 0);  // [1:0]    scramble_sel
    hdmirx_wr_dwc(HDMIRX_DWC_HDMI20_CONTROL,    data32);

    data32  = 0;
    data32 |= (1    << 24); // [25:24]  i2c_spike_suppr
    data32 |= (0    << 20); // [20]     i2c_timeout_en
    data32 |= (0    << 0);  // [19:0]   i2c_timeout_cnt
    hdmirx_wr_dwc(HDMIRX_DWC_SCDC_I2CCONFIG,    data32);

    data32  = 0;
    data32 |= (0    << 1);  // [1]      hpd_low
    data32 |= (1    << 0);  // [0]      power_provided
    hdmirx_wr_dwc(HDMIRX_DWC_SCDC_CONFIG,   data32);

    data32  = 0;
    data32 |= (0xabcdef << 8);  // [31:8]   manufacture_oui
    data32 |= (1        << 0);  // [7:0]    sink_version
    hdmirx_wr_dwc(HDMIRX_DWC_SCDC_WRDATA0,  data32);

    data32  = 0;
    data32 |= (10       << 20); // [29:20]  chlock_max_err
    data32 |= (24000    << 0);  // [15:0]   milisec_timer_limit
    hdmirx_wr_dwc(HDMIRX_DWC_CHLOCK_CONFIG, data32);
}
#endif
void hdmirx_hw_config(void)
{
	hdmirx_print("%s %d\n", __func__, rx.port);
	mdelay(10);
	//hdmirx_rd_check_TOP(HDMIRX_TOP_SW_RESET, 0x01, 0x00);
	hdmirx_wr_top(HDMIRX_TOP_MEM_PD,    0); // Release memories out of power down. // hdmi2.0 new
	clk_init();

	hdmirx_wr_top(HDMIRX_TOP_INTR_MASKN, 0); //disable top interrupt gate
	control_reset(0);
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
	if(multi_port_edid_enable)
		hdmirx_wr_top(HDMIRX_TOP_PORT_SEL, 0x10 | ((1<<rx.port))); //enable all 4 port available
	else
		hdmirx_wr_top(HDMIRX_TOP_PORT_SEL, ((1<<rx.port)));
#else
	hdmirx_wr_top(HDMIRX_TOP_PORT_SEL,   (1<<rx.port));  //G9 EDID multiplex mode
#endif
	hdmirx_interrupts_cfg(false); //disable dwc interrupt

	if(hdcp_enable){
		hdmi_rx_ctrl_hdcp_config(&rx.hdcp);
	} else {
		hdmirx_wr_bits_dwc(HDMIRX_DWC_HDCP_CTRL, HDCP_ENABLE, 0);
	}

	/*phy config*/
	//hdmirx_phy_restart();
	//hdmi_rx_phy_fast_switching(1);
	hdmirx_phy_init(rx.port, 0); //port, dcm
	/**/

	/* control config */
	control_init(rx.port);
	hdmirx_audio_init();//
	packet_init();
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
	hdmirx_20_init();
#endif
	hdmirx_audio_fifo_rst();
	hdmirx_packet_fifo_rst();
	/**/
	control_reset(1);

	/*enable irq */
	hdmirx_wr_top(HDMIRX_TOP_INTR_STAT_CLR, ~0);
	hdmirx_wr_top(HDMIRX_TOP_INTR_MASKN, 0x00001fff);
	hdmirx_interrupts_hpd(true);
	/**/

#ifndef USE_GPIO_FOR_HPD
	//hdmi_rx_ctrl_hpd(true);
	//hdmirx_wr_top( HDMIRX_TOP_HPD_PWR5V, (1<<5)|(1<<4)); //invert HDP output
#endif
#ifdef CEC_FUNC_ENABLE
	cec_init();
#endif
	/* wait at least 4 video frames (at 24Hz) : 167ms for the mode detection
	recover the video mode */
	mdelay(200);

	/* Check If HDCP engine is in Idle state, if not wait for authentication time.
	200ms is enough if no Ri errors */
	if (hdmirx_rd_dwc(0xe0) != 0)
	{
		printk("hdcp engine busy\n");
		mdelay(300);
	}
	hdmirx_print("%s  %d Done! \n", __func__, rx.port);
}

void hdmirx_hw_probe(void)
{
	hdmirx_wr_top(HDMIRX_TOP_MEM_PD,	0);//1
	hdmirx_wr_top(HDMIRX_TOP_SW_RESET,	0);//release IP from reset

    //Wr_reg_bits(HHI_GCLK_MPEG0, 1, 21, 1);  // Turn on clk_hdmirx_pclk, also = sysclk //1
    // Enable APB3 fail on error
    //*((volatile unsigned long *) P_HDMIRX_CTRL_PORT)          |= (1 << 15);   // APB3 to HDMIRX-TOP err_en
    //*((volatile unsigned long *) (P_HDMIRX_CTRL_PORT+0x10))   |= (1 << 15);   // APB3 to HDMIRX-DWC err_en

    //turn on clocks: md, cfg...
	//turn on cfg_clk
    WRITE_MPEG_REG(HHI_HDMIRX_CLK_CNTL,	0x100);

	//hdmirx_wr_dwc(HDMIRX_DWC_DMI_DISABLE_IF,0x29);//bit0,bit3,bit5
	hdmirx_wr_top(HDMIRX_TOP_EDID_GEN_CNTL,0x1e109);
	#ifdef CEC_FUNC_ENABLE
	cec_init();
	#endif
	hdmi_rx_ctrl_edid_update();

	hdmirx_wr_top(HDMIRX_TOP_PORT_SEL, 0x10);
	hdmirx_wr_top(HDMIRX_TOP_INTR_STAT_CLR, ~0);
	hdmirx_wr_top(HDMIRX_TOP_INTR_MASKN, 0x00001fff);
	hdmirx_print("%s Done! \n", __func__);
}


/***********************
   get infor and config:
hdmirx_packet_get_avi
hdmirx_get_video_info
************************/
int hdmirx_packet_get_avi(struct hdmi_rx_ctrl_video *params)
{
	int error = 0;

	params->video_format = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, VIDEO_FORMAT);
	params->active_valid = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, ACT_INFO_PRESENT);
	params->bar_valid = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, BAR_INFO_VALID);
	params->scan_info = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, SCAN_INFO);
	params->colorimetry = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, COLORIMETRY);
	params->picture_ratio = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, PIC_ASPECT_RATIO);
	params->active_ratio = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, ACT_ASPECT_RATIO);
	params->it_content = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, IT_CONTENT);
	params->ext_colorimetry = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, EXT_COLORIMETRY);
	params->rgb_quant_range = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, RGB_QUANT_RANGE);
	params->yuv_quant_range = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_HB, YUV_QUANT_RANGE);
	params->n_uniform_scale = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, NON_UNIF_SCALE);
	params->video_mode = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_PB, VID_IDENT_CODE);
	params->pixel_repetition = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_HB, PIX_REP_FACTOR);
	/** @note HW does not support AVI YQ1-0, YCC quantization range */
	/** @note HW does not support AVI CN1-0, IT content type */
	params->bar_end_top = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_TBB, LIN_END_TOP_BAR);
	params->bar_start_bottom = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_TBB, LIN_ST_BOT_BAR);
	params->bar_end_left = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_LRB, PIX_END_LEF_BAR);
	params->bar_start_right = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AVI_LRB, PIX_ST_RIG_BAR);
	return error;
}

int hdmirx_get_video_info(struct hdmi_rx_ctrl *ctx, struct hdmi_rx_ctrl_video *params)
{
	int error = 0;
	const unsigned factor = 100;
	unsigned divider = 0;
	uint32_t tmp = 0;

	/* DVI mode */
	params->dvi = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_STS, DVIDET) != 0;
	/* hdcp encrypted state */
	params->hdcp_enc_state = hdmirx_rd_bits_dwc(HDMIRX_DWC_HDCP_STS, ENCRYPTED_STATUS) != 0;
	/* AVI parameters */
	error |= hdmirx_packet_get_avi( params);
	if (error != 0) {
		goto exit;
	}
	/* pixel clock */
	params->pixel_clk = ctx->tmds_clk;
	/* refresh rate */
	//tmp = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_VTC, VTOT_CLK);
	//params->refresh_rate = (tmp == 0)? 0: (ctx->md_clk * 100000) / tmp;

	/* image parameters */
	params->interlaced = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_STS, ILACE) != 0;
	params->voffset = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_VOL, VOFS_LIN);
	params->vactive = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_VAL, VACT_LIN);
	params->vtotal = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_VTL, VTOT_LIN);
	if (params->interlaced)	{
		//params->voffset <<= 1;
		//params->vactive <<= 1;
		//params->vtotal <<= 1;
	}
	params->hoffset = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_HT1, HOFS_PIX);
	params->hactive = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_HACT_PX, HACT_PIX);
	params->htotal = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_HT1, HTOT_PIX);

		/* refresh rate */
	tmp = hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_VTC, VTOT_CLK);
	//tmp = (tmp == 0)? 0: (ctx->md_clk * 100000) / tmp;
	//if((params->vtotal == 0) || (params->htotal == 0))
	params->refresh_rate = (tmp == 0)? 0: (ctx->md_clk * 100000) / tmp;
	//else {
		//params->refresh_rate = (hdmirx_get_pixel_clock() / (params->vtotal * params->htotal / 100));
		//if(abs(tmp - params->refresh_rate) > 50)
			//printk("\n\n refresh_rate -%ld -- tmp-%d\n\n",params->refresh_rate,tmp);
	//}
	/* deep color mode */
	tmp = hdmirx_rd_bits_dwc( HDMIRX_DWC_HDMI_STS, DCM_CURRENT_MODE);
#if 0
	if (hdmirx_rd_bits_dwc( HDMIRX_DWC_PDEC_ISTS, DVIDET|AVI_CKS_CHG) != 0
		|| hdmirx_rd_bits_dwc( HDMIRX_DWC_MD_ISTS, VIDEO_MODE) != 0
		|| hdmirx_rd_bits_dwc( HDMIRX_DWC_HDMI_ISTS, DCM_CURRENT_MODE_CHG) != 0) {
		printk("%s error\n", __func__);
		error = -EAGAIN;
		goto exit;
	}
#endif
	switch (tmp) {
	case DCM_CURRENT_MODE_48b:
		params->deep_color_mode = 48;
		divider = 2.00 * factor;	/* divide by 2 */
		break;
	case DCM_CURRENT_MODE_36b:
		params->deep_color_mode = 36;
		divider = 1.50 * factor;	/* divide by 1.5 */
		break;
	case DCM_CURRENT_MODE_30b:
		params->deep_color_mode = 30;
		divider = 1.25 * factor;	/* divide by 1.25 */
		break;
	default:
		params->deep_color_mode = 24;
		divider = 1.00 * factor;
		break;
	}
	params->pixel_clk = (params->pixel_clk * factor) / divider;
	params->hoffset = (params->hoffset * factor) / divider;
	params->hactive	= (params->hactive * factor) / divider;
	params->htotal = (params->htotal  * factor) / divider;

exit:
	return error;
}


void hdmirx_config_video(struct hdmi_rx_ctrl_video *video_params)
{
	int data32=0;
#if ((MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9BB))

	if ((video_params->sw_vic >= HDMI_3840_2160p) && (video_params->sw_vic <= HDMI_4096_2160p)) {
	    data32 |= 1 << 23; //video_params.pixel_repetition << 23;  // [23]     hscale_half: 1=Horizontally scale down by half
	    data32 |= 1 << 29;  //clk_half  297-148.5
	} else {
	    data32 |= 0 << 23; //video_params.pixel_repetition << 23;  // [23]     hscale_half: 1=Horizontally scale down by half
	    data32 |= 0 << 29;  //clk_half  297-148.5
	}

    data32 |= 0                             << 22;  // [22]     force_vid_rate: 1=Force video output sample rate
    data32 |= 0                             << 19;  // [21:19]  force_vid_rate_chroma_cfg : 0=Bypass, not rate change. Applicable only if force_vid_rate=1
    data32 |= 0                             << 16;  // [18:16]  force_vid_rate_luma_cfg   : 0=Bypass, not rate change. Applicable only if force_vid_rate=1
    data32 |= 0x7fff                        << 0;   // [14: 0]  hsizem1
    hdmirx_wr_top( HDMIRX_TOP_VID_CNTL,   data32);
#endif
	if(video_params->video_format == 0) //rgb
	{
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_VM_CFG_CH2, 0);
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_VM_CFG_CH_0_1, 0);
	} else {
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_VM_CFG_CH2, 0x8000);
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_VM_CFG_CH_0_1, 0x8000);
	}

	if(video_params->interlaced == 1) {
		data32	= 0;
		data32 |= (hdmirx_rd_dwc(HDMIRX_DWC_HDMI_MODE_RECOVER) & 0xf8000000);	// [31:27]	preamble_cnt_limit
		data32 |= (0	<< 24); // [25:24]	mr_vs_pol_adj_mode
		data32 |= (0	<< 18); // [18] 	spike_filter_en
		data32 |= (8	<< 13); // [17:13]	dvi_mode_hyst
		data32 |= (8	<< 8);	// [12:8]	hdmi_mode_hyst
		data32 |= (0	<< 6);	// [7:6]	hdmi_mode: 0=automatic
		data32 |= (0	<< 4);	// [5:4]	gb_det
		data32 |= (0	<< 2);	// [3:2]	eess_oess
		data32 |= (0	<< 0);	// [1:0]	sel_ctl01
		hdmirx_wr_dwc(HDMIRX_DWC_HDMI_MODE_RECOVER, data32);
	}
}


int hdmirx_audio_init(void)
{
	#define AUD_CLK_DELTA   2000
	#define RX_8_CHANNEL        1        // 0=I2S 2-channel; 1=I2S 4 x 2-channel.
	int err = 0;
	unsigned long data32 = 0;
#if 1
    data32  = 0;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
	//weicheng.zhang set bit[15:13]=7
	data32 |= (7		<< 13); // [15:13]	n_cts_update_width:000=hdmirx_n_cts_update pulse width is 1 tmds_clk cycle; 001=2-cycle; 011=3-cycle; 111=4-cycle.
	data32 |= (0		<< 12); // [12] 	n_cts_update_on_pll4x:1=aud_pll4x_en change will result in a hdmirx_n_cts_update pulse; 0=No hdmirx_n_cts_update pulse on aud_pll4x_en change.
	data32 |= (1		<< 11); // [11] 	n_cts_update_man:1=manual assert an hdmirx_n_cts_update pulse; 0=no action.
	data32 |= (0		<< 10); // [10] 	pll_tmode_cntl
#endif
	data32 |= 0         << 9;   // [9]      force_afif_status:1=Use cntl_audfifo_status_cfg as fifo status; 0=Use detected afif_status.
    data32 |= 1         << 8;   // [8]      afif_status_auto:1=Enable audio FIFO status auto-exit EMPTY/FULL, if FIFO level is back to LipSync; 0=Once enters EMPTY/FULL, never exits.
    data32 |= 1         << 6;   // [ 7: 6]  Audio FIFO nominal level :0=s_total/4;1=s_total/8;2=s_total/16;3=s_total/32.
    data32 |= 3         << 4;   // [ 5: 4]  Audio FIFO critical level:0=s_total/4;1=s_total/8;2=s_total/16;3=s_total/32.
    data32 |= 0         << 3;   // [3]      afif_status_clr:1=Clear audio FIFO status to IDLE.
    data32 |= rx.ctrl.acr_mode  << 2;   // [2]      dig_acr_en
    data32 |= rx.ctrl.acr_mode  << 1;   // [1]      audmeas_clk_sel: 0=select aud_pll_clk; 1=select aud_acr_clk.
    data32 |= rx.ctrl.acr_mode  << 0;   // [0]      aud_clk_sel: 0=select aud_pll_clk; 1=select aud_acr_clk.
    hdmirx_wr_top(HDMIRX_TOP_ACR_CNTL_STAT, data32);
#if 0
    //hdmirx_wr_dwc( HDMIRX_DWC_AUDPLL_GEN_CTS, manual_acr_cts);
    //hdmirx_wr_dwc( HDMIRX_DWC_AUDPLL_GEN_N,   manual_acr_n);
    // Force N&CTS to start with, will switch to received values later on, for simulation speed up.
    data32  = 0;
    data32 |= 1 << 4;   // [4]      cts_n_ref: 0=used decoded; 1=use manual N&CTS.
    hdmirx_wr_dwc( HDMIRX_DWC_AUD_CLK_CTRL,   data32);
#endif
    data32  = 0;
    data32 |= 0 << 28;  // [28]     pll_lock_filter_byp
    data32 |= 0 << 24;  // [27:24]  pll_lock_toggle_div
    hdmirx_wr_dwc(HDMIRX_DWC_AUD_PLL_CTRL,   data32);    // DEFAULT: {1'b0, 3'd0, 4'd6, 4'd3, 4'd8, 1'b0, 1'b0, 1'b1, 1'b0, 12'd0}


    data32  = 0;
#if (MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV)
	//M6 config
    data32 |= 144    << 18;  // [26:18]  afif_th_start
    data32 |= 32     << 9;   // [17:9]   afif_th_max
    data32 |= 32     << 0;   // [8:0]    afif_th_min
#else
    //HDMI2.0 config
    data32 |= (80   << 18); // [26:18]  afif_th_start
    data32 |= (8    << 9);  // [17:9]   afif_th_max
    data32 |= (8    << 0);  // [8:0]    afif_th_min
    hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_TH,    data32);
#endif

    data32  = 0;
    data32 |= 1     << 16;  // [16]     afif_subpackets: 0=store all sp; 1=store only the ones' spX=1.
    data32 |= 0     << 0;   // [0]      afif_init
    hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_CTRL,  data32); // DEFAULT: {13'd0, 2'd0, 1'b1, 15'd0, 1'b0}

#if (MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV)
	//M6
    data32  = 0;
    data32 |= (RX_8_CHANNEL? 0x7 :0x0)  << 8;   // [10:8]   ch_map[7:5]
    data32 |= 1                         << 7;   // [7]      ch_map_manual
    data32 |= (RX_8_CHANNEL? 0x1f:0x3)  << 2;   // [6:2]    ch_map[4:0]
    data32 |= 1                         << 0;   // [1:0]    aud_layout_ctrl:0/1=auto layout; 2=layout 0; 3=layout 1.
#else
	//HDMI2.0
	data32  = 0;
    data32 |= (0                        << 8);  // [10:8]   ch_map[7:5]
    data32 |= (1                        << 7);  // [7]      ch_map_manual
    data32 |= ((RX_8_CHANNEL? 0x13:0x00)<< 2);  // [6:2]    ch_map[4:0]
    data32 |= (1                        << 0);  // [1:0]    aud_layout_ctrl:0/1=auto layout; 2=layout 0; 3=layout 1.
    hdmirx_wr_dwc(HDMIRX_DWC_AUD_CHEXTR_CTRL,   data32);// DEFAULT: {24'd0, 1'b0, 5'd0, 2'd0}
#endif
#endif
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
    data32  = 0;
    data32 |= 3     << 21;  // [22:21]  aport_shdw_ctrl
    data32 |= auto_aclk_mute     << 19;  // [20:19]  auto_aclk_mute
    data32 |= 1     << 10;  // [16:10]  aud_mute_speed
    data32 |= aud_avmute_en     << 7;   // [7]      aud_avmute_en
    data32 |= aud_mute_sel     << 5;   // [6:5]    aud_mute_sel
    data32 |= 1     << 3;   // [4:3]    aud_mute_mode
    data32 |= 0     << 1;   // [2:1]    aud_ttone_fs_sel
    data32 |= 0     << 0;   // [0]      testtone_en
    hdmirx_wr_dwc(HDMIRX_DWC_AUD_MUTE_CTRL,  data32); // DEFAULT: {9'd0, 2'd0, 2'd0, 2'd0, 7'd48, 2'd0, 1'b1, 2'd3, 2'd3, 2'd0, 1'b0}

    data32  = 0;
	data32 |= (0    << 16); // [17:16]  pao_rate
    data32 |= (0    << 12); // [12]     pao_disable
    data32 |= 0     << 4;   // [11:4]   audio_fmt_chg_thres
    data32 |= 0     << 1;   // [2:1]    audio_fmt
    data32 |= 0     << 0;   // [0]      audio_fmt_sel
    hdmirx_wr_dwc(HDMIRX_DWC_AUD_PAO_CTRL,   data32); // DEFAULT: {20'd0, 8'd176, 1'b0, 2'd0, 1'b0}

    data32  = 0;
    data32 |= 0     << 8;   // [8]      fc_lfe_exchg: 1=swap channel 3 and 4
    hdmirx_wr_dwc(HDMIRX_DWC_PDEC_AIF_CTRL,  data32); // DEFAULT: {23'd0, 1'b0, 8'd0}

    data32  = 0;
    data32 |= 0 << 2;   // [4:2]    deltacts_irqtrig
    data32 |= 0 << 0;   // [1:0]    cts_n_meas_mode
    hdmirx_wr_dwc(HDMIRX_DWC_PDEC_ACRM_CTRL, data32); // DEFAULT: {27'd0, 3'd0, 2'd1}

	data32	= 0;
	data32 |= 1		<< 10;	//mute
	data32 |= (0    << 9);  // [9]      sck_disable
	data32 |= (0    << 5);  // [8:5]    i2s_disable
	data32 |= (0    << 1);  // [4:1]    spdif_disable
	data32 |= 1		<< 0;	//enable all outputs and select 32-bit for I2S
	hdmirx_wr_dwc(HDMIRX_DWC_AUD_SAO_CTRL, data32);
#endif
	/* amlogic HDMIRX audio module enable*/

/**/
#if 0
    WRITE_MPEG_REG(AUDIN_SOURCE_SEL,    (0  <<12)   | // [14:12]cntl_hdmirx_chsts_sel: 0=Report chan1 status; 1=Report chan2 status; ...; 7=Report chan8 status.
                            (0xf<<8)    | // [11:8] cntl_hdmirx_chsts_en
                            (1  <<4)    | // [5:4]  spdif_src_sel: 1=Select HDMIRX SPDIF output as AUDIN source
                            (2 << 0));    // [1:0]  i2sin_src_sel: 2=Select HDMIRX I2S output as AUDIN source
#endif
return err;
}
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
void diff_with_testcode(void)
{
	unsigned long data32;
	data32	= 0;
	data32 |= (hdmirx_rd_dwc(HDMIRX_DWC_HDMI_MODE_RECOVER) & 0xf8000000);	// [31:27]	preamble_cnt_limit
	data32 |= (0	<< 24); // [25:24]	mr_vs_pol_adj_mode
	data32 |= (0	<< 18); // [18] 	spike_filter_en
	data32 |= (0	<< 13); // [17:13]	dvi_mode_hyst
	data32 |= (0	<< 8);	// [12:8]	hdmi_mode_hyst
	data32 |= (0	<< 6);	// [7:6]	hdmi_mode: 0=automatic
	data32 |= (2	<< 4);	// [5:4]	gb_det
	data32 |= (0	<< 2);	// [3:2]	eess_oess
	data32 |= (1	<< 0);	// [1:0]	sel_ctl01
	hdmirx_wr_dwc(HDMIRX_DWC_HDMI_MODE_RECOVER, data32);

	data32  = 0;
    data32 |= (0                << 28); // [28]     dcm_ph_diff_cnt_clr_p
    data32 |= (0                << 20); // [27:20]  dcm_ph_diff_cnt_thres
    data32 |= (1                << 18); // [18]     dcm_default_phase
    data32 |= (0                << 17); // [17]     dcm_pixel_phase_sel
    data32 |= (0                << 13); // [16:13]  dcm_pixel_phase
    data32 |= (1                << 12); // [12]     dcm_colour_depth_sel. Speed up simulation by forcing color_depth for the first 2 frames, this bit will be reset in test.c after 2 frames.
    ///data32 |= (rx_color_depth   << 8);  // [11:8]   dcm_colour_depth   ??
    data32 |= (5                << 2);  // [5:2]    dcm_gcp_zero_fields
    data32 |= (0                << 0);  // [1:0]    dcm_dc_operation
    hdmirx_wr_dwc(HDMIRX_DWC_HDMI_DCM_CTRL, data32);

	data32  = 0;
    data32 |= (0    << 9);  // [13:9]   audiodet_threshold
    hdmirx_wr_dwc(HDMIRX_DWC_PDEC_DBG_CTRL, data32);
}
#endif
#if (MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV)
unsigned int clk_util_clk_msr2( unsigned long   clk_mux, unsigned long   uS_gate_time )
{
    unsigned long dummy_rd;
    unsigned long   measured_val;
    unsigned long timeout = 0;
    // Set the measurement gate to 100uS
    WRITE_MPEG_REG(MSR_CLK_REG0, (READ_MPEG_REG(MSR_CLK_REG0) & ~(0xFFFF << 0)) | ((uS_gate_time-1) << 0) );
    // Disable continuous measurement
    // disable interrupts
    WRITE_MPEG_REG(MSR_CLK_REG0, (READ_MPEG_REG(MSR_CLK_REG0) & ~((1 << 18) | (1 << 17))) );
    WRITE_MPEG_REG(MSR_CLK_REG0, (READ_MPEG_REG(MSR_CLK_REG0) & ~(0xf << 20)) | ((clk_mux << 20) |  // Select MUX
                                                          (1 << 19) |     // enable the clock
                                                          (1 << 16)) );    // enable measuring
    dummy_rd = READ_MPEG_REG(MSR_CLK_REG0);
    // Wait for the measurement to be done
    while( (READ_MPEG_REG(MSR_CLK_REG0) & (1 << 31)) ) {
        mdelay(1);
        timeout++;
        if(timeout>100){
            return 0;
        }
    }
    // disable measuring
    WRITE_MPEG_REG(MSR_CLK_REG0, (READ_MPEG_REG(MSR_CLK_REG0) & ~(1 << 16)) | (0 << 16) );

    measured_val = READ_MPEG_REG(MSR_CLK_REG2);
    if( measured_val == 65535 ) {
        return(0);
    } else {
        // Return value in Hz
        return(measured_val*(1000000/uS_gate_time));
    }
}

unsigned int hdmirx_get_clock(int index)
{
    return clk_util_clk_msr2(index, 50);
}

unsigned int hdmirx_get_tmds_clock(void)
{
    return clk_util_clk_msr2(57, 50);
}

unsigned int hdmirx_get_pixel_clock(void)
{
    return clk_util_clk_msr2(58, 50);
}

#else
#ifndef CONFIG_CLK_MSR_NEW
static unsigned int clk_util_clk_msr(unsigned int clk_mux)
{
	unsigned int  msr;
	unsigned int regval = 0;
	aml_write_reg32(P_MSR_CLK_REG0,0);
	// Set the measurement gate to 64uS
	clrsetbits_le32(P_MSR_CLK_REG0,0xffff,64-1);
	// Disable continuous measurement
	// disable interrupts
	clrbits_le32(P_MSR_CLK_REG0,((1 << 18) | (1 << 17)));
	clrsetbits_le32(P_MSR_CLK_REG0,(0x1f<<20),(clk_mux<<20)|(1<<19)|(1<<16));

	aml_read_reg32(P_MSR_CLK_REG0);
	// Wait for the measurement to be done
	do {
	regval = aml_read_reg32(P_MSR_CLK_REG0);
	} while (regval & (1 << 31));
	// disable measuring
	clrbits_le32(P_MSR_CLK_REG0,(1 << 16));
	msr=(aml_read_reg32(P_MSR_CLK_REG2)+31)&0x000FFFFF;
	// Return value in MHz*measured_val
	return (msr>>6)*1000000;

}
#else
static  unsigned int clk_util_clk_msr(unsigned int clk_mux)
{
    unsigned int regval = 0;
    /// Set the measurement gate to 64uS
    clrsetbits_le32(P_MSR_CLK_REG0,0xffff,121);///122us

    // Disable continuous measurement
    // disable interrupts
    clrsetbits_le32(P_MSR_CLK_REG0,
        ((1 << 18) | (1 << 17)|(0x1f << 20)),///clrbits
        (clk_mux << 20) |                    /// Select MUX
        (1 << 19) |                          /// enable the clock
        (1 << 16));
    // Wait for the measurement to be done
    regval = aml_read_reg32(P_MSR_CLK_REG0);
    do {
        regval = aml_read_reg32(P_MSR_CLK_REG0);
    } while (regval & (1 << 31));

    // disable measuring
    clrbits_le32(P_MSR_CLK_REG0, (1 << 16));
    regval = (aml_read_reg32(P_MSR_CLK_REG2)) & 0x000FFFFF;
    regval += (regval/10000) * 6;
    // Return value in MHz*measured_val
    return (regval << 13);
}
#endif
unsigned int hdmirx_get_clock(int index)
{
    return clk_util_clk_msr(index);
}

unsigned int hdmirx_get_audio_clock(void)
{
    return clk_util_clk_msr(12);
}

unsigned int hdmirx_get_tmds_clock(void)
{
    return clk_util_clk_msr(28);
}

unsigned int hdmirx_get_pixel_clock(void)
{
    return clk_util_clk_msr(29);
}
#endif

void hdmirx_read_audio_info(struct aud_info_s* audio_info)
{
	/*get AudioInfo */
	audio_info->coding_type = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB0, CODING_TYPE);
	audio_info->channel_count = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB0, CHANNEL_COUNT);
	audio_info->sample_frequency = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB0, SAMPLE_FREQ);
	audio_info->sample_size = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB0, SAMPLE_SIZE);
	audio_info->coding_extension = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB0, AIF_DATA_BYTE_3);
	audio_info->channel_allocation = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB0, CH_SPEAK_ALLOC);
	audio_info->down_mix_inhibit = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB1, DWNMIX_INHIBIT);
	audio_info->level_shift_value = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AIF_PB1, LEVEL_SHIFT_VAL);
	audio_info->audio_samples_packet_received = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_AUD_STS, AUDS_RCV);

  	audio_info->cts = hdmirx_rd_dwc(HDMIRX_DWC_PDEC_ACR_CTS);
  	audio_info->n = hdmirx_rd_dwc(HDMIRX_DWC_PDEC_ACR_N);
  	if(audio_info->cts!=0){
		audio_info->audio_recovery_clock = (hdmirx_get_tmds_clock()/audio_info->cts)*audio_info->n/128;
	}else{
		audio_info->audio_recovery_clock = 0;
	}
}

void hdmirx_read_vendor_specific_info_frame(struct vendor_specific_info_s* vs)
{
    vs->identifier = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_VSI_ST0, IEEE_REG_ID);
    vs->hdmi_video_format = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_VSI_ST1, HDMI_VIDEO_FORMAT);
    vs->_3d_structure = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_VSI_ST1, H3D_STRUCTURE);
    vs->_3d_ext_data = hdmirx_rd_bits_dwc(HDMIRX_DWC_PDEC_VSI_ST1, H3D_EXT_DATA);
}
