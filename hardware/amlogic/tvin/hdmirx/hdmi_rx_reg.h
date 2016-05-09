/*
 * hdmirx_rx_reg.h brief registers address in HDMI RX module.
 *
 * Copyright (C) 2012 AMLOGIC, INC. All Rights Reserved.
 * Author: Rain Zhang <rain.zhang@amlogic.com>
 * Author: Xiaofei Zhu <xiaofei.zhu@amlogic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the smems of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#ifndef HDMI_RX_REG_H_
#define HDMI_RX_REG_H_

//#define HDMIRX_IRQ                            56//  24  define in irqs.h

/**
 * Bit field mask
 * @param m	width
 * @param n shift
 */
#define MSK(m, n)		(((1 << (m)) - 1) << (n))
/**
 * Bit mask
 * @param n shift
 */
#define _BIT(n)			MSK(1, (n))

/** PHY Gen3 Clock measurement lock threshold - default 8*/
#define LOCK_THRES                       0x63//0x08
/** register address: PHY Gen3 clock measurement unit configuration */
#define REG_HDMI_PHY_CMU_CONFIG			(0x02UL)
/** register address: PHY Gen3 system configuration */
#define REG_HDMI_PHY_SYSTEM_CONFIG		(0x03UL)
#define HDMIRX_PHY_MAINFSM_CTL			(0x05UL)
/** register address: PHY Gen3 main FSM status 1 */
#define REG_HDMI_PHY_MAINFSM_STATUS1	   (0x09UL)


#define HDMIRX_PHY_RESISTOR_CALIBRATION_1 (0x10UL)
#define HDMIRX_PHY_MAIN_FSM_OVERRIDE2	(0x08UL)
#define HDMIRX_PHY_CH0_EQ_CTRL3			(0x3EUL)
#define HDMIRX_PHY_CH1_EQ_CTRL3			(0x5EUL)
#define HDMIRX_PHY_CH2_EQ_CTRL3			(0x7EUL)

#define HDMIRX_PHY_EQCTRL1_CH0			(0x32UL)
#define HDMIRX_PHY_EQCTRL1_CH1			(0x52UL)
#define HDMIRX_PHY_EQCTRL1_CH2			(0x72UL)

#define HDMIRX_PHY_EQCTRL4_CH0			(0x3FUL)
#define HDMIRX_PHY_EQCTRL4_CH1			(0x5FUL)
#define HDMIRX_PHY_EQCTRL4_CH2			(0x7FUL)

#define HDMIRX_PHY_EQCTRL2_CH0			(0x33UL)
#define HDMIRX_PHY_EQCTRL2_CH1			(0x53UL)
#define HDMIRX_PHY_EQCTRL2_CH2			(0x73UL)

#define HDMIRX_PHY_EQSTAT3_CH0			(0x42UL)
#define HDMIRX_PHY_EQSTAT3_CH1			(0x62UL)
#define HDMIRX_PHY_EQSTAT3_CH2			(0x82UL)


#define OVL_PROT_CTRL                   (0x0DUL)
#define HDMIRX_PHY_CDR_CTRL_CNT					(0x0EUL) //HDMI2.0
#define HDMIRX_PHY_VOLTAGE_LEVEL		(0x22UL)
#define HDMIRX_PHY_MPLL_CTRL			(0x24UL)
#define MPLL_PARAMETERS2                (0x27UL)
#define MPLL_PARAMETERS3                (0x28UL)
#define MPLL_PARAMETERS4                (0x29UL)
#define MPLL_PARAMETERS5                (0x2AUL)
#define MPLL_PARAMETERS6                (0x2BUL)
#define MPLL_PARAMETERS7                (0x2CUL)
#define MPLL_PARAMETERS8                (0x2DUL)
#define MPLL_PARAMETERS9                (0x2EUL)
#define MPLL_PARAMETERS10                (0xC0UL)
#define MPLL_PARAMETERS11                (0xC1UL)
#define MPLL_PARAMETERS12                (0xC2UL)
#define MPLL_PARAMETERS13                (0xC3UL)
#define MPLL_PARAMETERS14                (0xC4UL)
#define MPLL_PARAMETERS15                (0xC5UL)
#define MPLL_PARAMETERS16                (0xC6UL)
#define MPLL_PARAMETERS17               (0xC7UL)
#define MPLL_PARAMETERS18                (0xC8UL)
#define MPLL_PARAMETERS19                (0xC9UL)
#define MPLL_PARAMETERS20                (0xCAUL)
#define MPLL_PARAMETERS21                (0xCBUL)



//------------------------------------------------------------------------------
// TOP-level wrapper registers addresses
//------------------------------------------------------------------------------

#define HDMIRX_TOP_SW_RESET                     0x000
#define HDMIRX_TOP_CLK_CNTL                     0x001
#define HDMIRX_TOP_HPD_PWR5V                    0x002
#define HDMIRX_TOP_PORT_SEL                     0x003
#define HDMIRX_TOP_EDID_GEN_CNTL                0x004
#define HDMIRX_TOP_EDID_ADDR_CEC                0x005
#define HDMIRX_TOP_EDID_DATA_CEC_PORT01         0x006
#define HDMIRX_TOP_EDID_DATA_CEC_PORT23         0x007
#define HDMIRX_TOP_EDID_GEN_STAT                0x008
#define HDMIRX_TOP_INTR_MASKN                   0x009
#define HDMIRX_TOP_INTR_STAT                    0x00A
#define HDMIRX_TOP_INTR_STAT_CLR                0x00B
#define HDMIRX_TOP_VID_CNTL                     0x00C
#define HDMIRX_TOP_VID_STAT                     0x00D
#define HDMIRX_TOP_ACR_CNTL_STAT                0x00E
#define HDMIRX_TOP_ACR_AUDFIFO                  0x00F
#define HDMIRX_TOP_ARCTX_CNTL                   0x010
#define HDMIRX_TOP_METER_HDMI_CNTL              0x011
#define HDMIRX_TOP_METER_HDMI_STAT              0x012
#define HDMIRX_TOP_VID_CNTL2                    0x013

// hdmi2.0 new start
#define HDMIRX_TOP_MEM_PD                       0x014
#define HDMIRX_TOP_EDID_RAM_OVR0                0x015
#define HDMIRX_TOP_EDID_RAM_OVR0_DATA           0x016
#define HDMIRX_TOP_EDID_RAM_OVR1                0x017
#define HDMIRX_TOP_EDID_RAM_OVR1_DATA           0x018
#define HDMIRX_TOP_EDID_RAM_OVR2                0x019
#define HDMIRX_TOP_EDID_RAM_OVR2_DATA           0x01a
#define HDMIRX_TOP_EDID_RAM_OVR3                0x01b
#define HDMIRX_TOP_EDID_RAM_OVR3_DATA           0x01c
#define HDMIRX_TOP_EDID_RAM_OVR4                0x01d
#define HDMIRX_TOP_EDID_RAM_OVR4_DATA           0x01e
#define HDMIRX_TOP_EDID_RAM_OVR5                0x01f
#define HDMIRX_TOP_EDID_RAM_OVR5_DATA           0x020
#define HDMIRX_TOP_EDID_RAM_OVR6                0x021
#define HDMIRX_TOP_EDID_RAM_OVR6_DATA           0x022
#define HDMIRX_TOP_EDID_RAM_OVR7                0x023
#define HDMIRX_TOP_EDID_RAM_OVR7_DATA           0x024
#define HDMIRX_TOP_EDID_GEN_STAT_B              0x025
#define HDMIRX_TOP_EDID_GEN_STAT_C              0x026
#define HDMIRX_TOP_EDID_GEN_STAT_D              0x027
#define HDMIRX_TOP_DONT_TOUCH0                  0x0fe   // Do not write to this register, it is a place holder to testbench to feed back onto APB.
#define HDMIRX_TOP_DONT_TOUCH1                  0x0ff   // Do not write to this register, it is a place holder to testbench to feed back onto APB.
// hdmi2.0 new end
#define HDMIRX_TOP_EDID_OFFSET                  0x200



/*
 * HDMI registers
 */
/** Register address: setup control */
#define HDMIRX_DWC_HDMI_SETUP_CTRL      (0x000UL)
/** Hot plug detect signaled */
#define 	HOT_PLUG_DETECT			_BIT(0)
/** Register address: override control */
#define HDMIRX_DWC_HDMI_OVR_CTRL        (0x004UL)
/** Register address: timer control */
#define HDMIRX_DWC_HDMI_TIMEHDMIRX_DWC_CTRL     (0x008UL)
/** Register address: resistor override */
#define HDMIRX_DWC_HDMI_RES_OVR         (0x010UL)
/** Register address: resistor status */
#define HDMIRX_DWC_HDMI_RES_STS         (0x014UL)
/** Register address: TMDS PLL control */
#define HDMIRX_DWC_HDMI_PLL_CTRL        (0x018UL)
/** Register address: TMDS PLL frequency range */
#define HDMIRX_DWC_HDMI_PLL_FRQSET1     (0x01CUL)
/** Register address: TMDS PLL frequency range */
#define HDMIRX_DWC_HDMI_PLL_FRQSET2     (0x020UL)
/** Register address: TMDS PLL PCP and ICP range */
#define HDMIRX_DWC_HDMI_PLL_PAR1        (0x024UL)
/** Register address: TMDS PLL PCP and ICP range */
#define HDMIRX_DWC_HDMI_PLL_PAR2        (0x028UL)
/** Register address: TMDS PLL KOSC and CCOLF range */
#define HDMIRX_DWC_HDMI_PLL_PAR3        (0x02CUL)
/** Register address: PLL post lock filter */
#define HDMIRX_DWC_HDMI_PLL_LCK_STS     (0x030UL)
/** Register address: PLL clock control */
#define HDMIRX_DWC_HDMI_CLK_CTRL        (0x034UL)
/** Register address: PCB diversity control */
#define HDMIRX_DWC_HDMI_PCB_CTRL        (0x038UL)
/** Input selector */
#define		INPUT_SELECT			_BIT(16)
/** Register address: phase control */
#define HDMIRX_DWC_HDMI_PHS_CTRL        (0x040UL)
/** Register address: used phases */
#define HDMIRX_DWC_HDMI_PHS_USD         (0x044UL)
/** Register address: miscellaneous operations control */
#define HDMIRX_DWC_HDMI_MISC_CTRL       (0x048UL)
/** Register address: EQ offset calibration */
#define HDMIRX_DWC_HDMI_EQOFF_CTRL      (0x04CUL)
/** Register address: EQ gain control */
#define HDMIRX_DWC_HDMI_EQGAIN_CTRL     (0x050UL)
/** Register address: EQ status */
#define HDMIRX_DWC_HDMI_EQCAL_STS       (0x054UL)
/** Register address: EQ results */
#define HDMIRX_DWC_HDMI_EQRESULT        (0x058UL)
/** Register address: EQ measurement control */
#define HDMIRX_DWC_HDMI_EQ_MEAS_CTRL    (0x05CUL)
/** Register address: HDMI mode recover */
#define HDMIRX_DWC_HDMI_MODE_RECOVER    (0x080UL)
/** Register address: HDMI error protection */
#define HDMIRX_DWC_HDMI_ERROR_PROTECT  (0x084UL)
/** Register address: validation and production test */
#define HDMIRX_DWC_HDMI_ERD_STS         (0x088UL)
/** Register address: video output sync signals control */
#define HDMIRX_DWC_HDMI_SYNC_CTRL       (0x090UL)
/** VS polarity adjustment */
#define		VS_POL_ADJ_MODE			MSK(2, 3)
/** HS polarity adjustment automatic */
#define		VS_POL_ADJ_AUTO			(2)
/** HS polarity adjustment */
#define		HS_POL_ADJ_MODE			MSK(2, 1)
/** HS polarity adjustment automatic inversion */
#define		HS_POL_ADJ_AUTO			(2)
/** Register address: clock measurement */
#define HDMIRX_DWC_HDMI_CKM_EVLTM       (0x094UL)
/** Evaluation period */
#define		EVAL_TIME				MSK(12, 4)
/** active wait period for TMDS stabilisation */
#define		TMDS_STABLE_TIMEOUT			(30)
/** Register address: legal clock count */
#define HDMIRX_DWC_HDMI_CKM_F           (0x098UL)
/** Maximum count for legal count */
#define 	CKM_MAXFREQ					MSK(16, 16)
/** Minimum count for legal count */
#define 	MINFREQ					MSK(16, 0)
/** Register address: measured clock results */
#define HDMIRX_DWC_HDMI_CKM_RESULT      (0x09CUL)
/** Measured clock is stable */
#define 	CLOCK_IN_RANGE			_BIT(17)
/** Measured clock rate in bits */
#define 	CLKRATE					MSK(16, 0)
/** Register address: sub-sampling control */
#define HDMIRX_DWC_HDMI_RESMPL_CTRL     (0x0A4UL)
/** Register address: deep color mode control */
#define HDMIRX_DWC_HDMI_DCM_CTRL        (0x0A8UL)
/** Register address: video output mute configuration */
#define HDMIRX_DWC_HDMI_VM_CFG_CH_0_1   (0x0B0UL)
/** Register address: video output mute configuration */
#define HDMIRX_DWC_HDMI_VM_CFG_CH2      (0x0B4UL)
/** Register address: spare */
#define HDMIRX_DWC_HDMI_SPARE           (0x0B8UL)
/** Register address: HDMI status */
#define HDMIRX_DWC_HDMI_STS             (0x0BCUL)
/** Current deep color mode */
#define		DCM_CURRENT_MODE		MSK(4, 28)
/** Deep color mode, 24 bit */
#define		DCM_CURRENT_MODE_24b	(4)
/** Deep color mode, 30 bit */
#define		DCM_CURRENT_MODE_30b	(5)
/** Deep color mode, 36 bit */
#define		DCM_CURRENT_MODE_36b	(6)
/** Deep color mode, 48 bit */
#define		DCM_CURRENT_MODE_48b	(7)

// HDMI 2.0 feature registers
//bit0-1  scramble ctrl
#define HDMIRX_DWC_HDMI20_CONTROL               0x0800
#define HDMIRX_DWC_SCDC_I2CCONFIG               0x0804
#define HDMIRX_DWC_SCDC_CONFIG                  0x0808
#define HDMIRX_DWC_CHLOCK_CONFIG                0x080C
#define HDMIRX_DWC_SCDC_REGS0                   0x0820
#define HDMIRX_DWC_SCDC_REGS1                   0x0824
#define HDMIRX_DWC_SCDC_REGS2                   0x0828
#define HDMIRX_DWC_SCDC_REGS3                   0x082C
#define HDMIRX_DWC_SCDC_MANSPEC0                0x0840
#define HDMIRX_DWC_SCDC_MANSPEC1                0x0844
#define HDMIRX_DWC_SCDC_MANSPEC2                0x0848
#define HDMIRX_DWC_SCDC_MANSPEC3                0x084C
#define HDMIRX_DWC_SCDC_MANSPEC4                0x0850
#define HDMIRX_DWC_SCDC_WRDATA0                 0x0860
#define HDMIRX_DWC_SCDC_WRDATA1                 0x0864
#define HDMIRX_DWC_SCDC_WRDATA2                 0x0868
#define HDMIRX_DWC_SCDC_WRDATA3                 0x086C
#define HDMIRX_DWC_SCDC_WRDATA4                 0x0870
#define HDMIRX_DWC_SCDC_WRDATA5                 0x0874
#define HDMIRX_DWC_SCDC_WRDATA6                 0x0878
#define HDMIRX_DWC_SCDC_WRDATA7                 0x087C
#define HDMIRX_DWC_HDMI20_STATUS                0x08E0


/*
 * hdcp register
 */
/** Register address: HDMI status */
#define HDMIRX_DWC_HDCP_DBG             (0x0E0UL)
/*
 * Video Mode registers
 */
/** Register address: video mode control */
#define HDMIRX_DWC_MD_HCTRL1            (0x140UL)
/** Register address: video mode control */
#define HDMIRX_DWC_MD_HCTRL2            (0x144UL)
/** Register address: horizontal sync */
#define HDMIRX_DWC_MD_HT0               (0x148UL)
/** Register address: horizontal offset */
#define HDMIRX_DWC_MD_HT1               (0x14CUL)
/** Horizontal total length */
#define 	HTOT_PIX				MSK(16, 16)
/** Horizontal offset length */
#define 	HOFS_PIX				MSK(16, 0)
/** Register address: horizontal active length */
#define HDMIRX_DWC_MD_HACT_PX           (0x150UL)
/** Horizontal active length */
#define 	HACT_PIX				MSK(16, 0)
/** Register address: horizontal active time */
#define HDMIRX_DWC_MD_HACT_PXA          (0x154UL)
/** Register address: vertical control */
#define HDMIRX_DWC_MD_VCTRL             (0x158UL)
/** Register address: vertical timing - sync pulse duration */
#define HDMIRX_DWC_MD_VSC               (0x15CUL)
/** Register address: vertical timing - frame duration */
#define HDMIRX_DWC_MD_VTC               (0x160UL)
/** Frame duration */
#define		VTOT_CLK				(~0)
/** Register address: vertical offset length */
#define HDMIRX_DWC_MD_VOL               (0x164UL)
/** Vertical offset length */
#define 	VOFS_LIN				MSK(16, 0)
/** Register address: vertical active length */
#define HDMIRX_DWC_MD_VAL               (0x168UL)
/** Vertical active length */
#define 	VACT_LIN				MSK(16, 0)
/** Register address: vertical timing */
#define HDMIRX_DWC_MD_VTH               (0x16CUL)
/** Register address: vertical total length */
#define HDMIRX_DWC_MD_VTL               (0x170UL)
/** Vertical total length */
#define 	VTOT_LIN				MSK(16, 0)
/** Register address: skew measurement trigger */
#define HDMIRX_DWC_MD_IL_CTRL           (0x174UL)
/** Register address: VS and HS skew */
#define HDMIRX_DWC_MD_IL_SKEW           (0x178UL)
/** Register address: V&H skew and filed detection */
#define HDMIRX_DWC_MD_IL_POL            (0x17CUL)
/** Register address: video mode status */
#define HDMIRX_DWC_MD_STS               (0x180UL)
/** Interlace active status */
#define 	ILACE					_BIT(3)
/*
 * Audio registers
 */
/** Register address: audio mode control */
#define HDMIRX_DWC_AUD_CTRL             (0x200UL)
/** Register address: audio PLL control */
#define HDMIRX_DWC_AUD_PLL_CTRL         (0x208UL)
/** Register address: audio PLL lock */
#define HDMIRX_DWC_AUD_PLL_LOCK         (0x20CUL)
/** Register address: DDS audio clock control */
#define HDMIRX_DWC_AUD_PLL_RESET        (0x210UL)
/** Register address: audio clock control */
#define HDMIRX_DWC_AUD_CLK_CTRL         (0x214UL)
/** Register address: ASP sync intervals */
#define HDMIRX_DWC_AUD_CLK_MASP         (0x218UL)
/** Register address: audio sync interval */
#define HDMIRX_DWC_AUD_CLK_MAUD         (0x21CUL)
/** Register address: sync interval reset */
#define HDMIRX_DWC_AUD_FILT_CTRL1       (0x220UL)
/** Register address: phase filter control */
#define HDMIRX_DWC_AUD_FILT_CTRL2       (0x224UL)
/** Register address: manual CTS control */
#define HDMIRX_DWC_AUD_CTS_MAN          (0x228UL)
/** Register address: manual N control */
#define HDMIRX_DWC_AUD_N_MAN            (0x22CUL)
/** Register address: audio clock status */
#define HDMIRX_DWC_AUD_CLK_STS          (0x23CUL)
/** Register address: audio FIFO control */
#define HDMIRX_DWC_AUD_FIFO_CTRL        (0x240UL)
/** Audio FIFO reset */
#define 	AFIF_INIT				_BIT(0)
/** Register address: audio FIFO threshold */
#define HDMIRX_DWC_AUD_FIFO_TH          (0x244UL)
/** Register address: audio FIFO fill */
#define HDMIRX_DWC_AUD_FIFO_FILL_S      (0x248UL)
/** Register address: audio FIFO fill minimum/maximum */
#define HDMIRX_DWC_AUD_FIFO_CLHDMIRX_DWC_MM     (0x24CUL)
/** Register address: audio FIFO fill status */
#define HDMIRX_DWC_AUD_FIFO_FILLSTS     (0x250UL)
/** Register address: audio output interface configuration */
#define HDMIRX_DWC_AUD_CHEXTR_CTRL     (0x254UL)
/** Register address: audio mute control */
#define HDMIRX_DWC_AUD_MUTE_CTRL        (0x258UL)
/** Manual/automatic audio mute control */
#define		AUD_MUTE_SEL			MSK(2, 5)
/** Force unmute (overrules all) */
#define		AUD_MUTE_FORCE_UN		(0)
/** Automatic mute when FIFO thresholds are exceeded */
#define		AUD_MUTE_FIFO_TH		(1)
/** Automatic mute when FIFO over/underflows */
#define		AUD_MUTE_FIFO_FL		(2)
/** Force mute (overrules all) */
#define		AUD_MUTE_FORCE			(3)
/** Register address: serial audio output control */
#define HDMIRX_DWC_AUD_SAO_CTRL         (0x260UL)
/** Register address: parallel audio output control */
#define HDMIRX_DWC_AUD_PAO_CTRL         (0x264UL)
/** Register address: audio FIFO status */
#define HDMIRX_DWC_AUD_FIFO_STS         (0x27CUL)

#define HDMIRX_DWC_AUDPLL_GEN_CTS       (0x280UL)
#define HDMIRX_DWC_AUDPLL_GEN_N         (0x284UL)

/** Register address: lock detector threshold */
#define HDMIRX_DWC_CI_PLLAUDIO_5        (0x28CUL)
/** Register address: test mode selection */
#define HDMIRX_DWC_CI_PLLAUDIO_4        (0x290UL)
/** Register address: bypass divider control */
#define HDMIRX_DWC_CI_PLLAUDIO_3        (0x294UL)
/** Register address: monitoring */
#define HDMIRX_DWC_CI_PLLAUDIO_2        (0x298UL)
/** Register address: control */
#define HDMIRX_DWC_CI_PLLAUDIO_1        (0x29CUL)
/** Register address: SNPS PHY GEN3 control - starting version 1.30a */
#define HDMIRX_DWC_SNPS_PHYG3_CTRL	(0x2C0UL)
/** Register address:  I2C Master: slave address - starting version 1.30a */
#define HDMIRX_DWC_I2CM_PHYG3_SLAVE 		(0x2C4UL)
/** Register address: I2C Master: register address - starting version 1.30a */
#define HDMIRX_DWC_I2CM_PHYG3_ADDRESS 		(0x2C8UL)
/** Register address: I2C Master: data to write to slave-starting version 1.30a*/
#define HDMIRX_DWC_I2CM_PHYG3_DATAO  		(0x2CCUL)
/** Register address: I2C Master: data read from slave-starting version 1.30a*/
#define HDMIRX_DWC_I2CM_PHYG3_DATAI  		(0x2D0UL)
/** Register address: I2C Master: operation RD/WR - starting version 1.30a */
#define HDMIRX_DWC_I2CM_PHYG3_OPERATION		(0x2D4UL)
/** Register address: I2C Master: SS/HS mode - starting version 1.30a */
#define HDMIRX_DWC_I2CM_PHYG3_MODE			(0x2D8UL)
/** Register address: I2C Master: soft reset - starting version 1.30a */
#define HDMIRX_DWC_I2CM_PHYG3_SOFTRST		(0x2DCUL)
/** Register address: I2C Master: ss mode counters  - starting version 1.30a */
#define HDMIRX_DWC_I2CM_PHYG3_SS_CNTS		(0x2E0UL)
/** Register address:I2C Master:  hs mode counters  - starting version 1.30a */
#define HDMIRX_DWC_I2CM_PHYG3_FS_HCNT		(0x2E4UL)
/*
 * Packet Decoder and FIFO Control registers
 */
/** Register address: packet decoder and FIFO control */
#define HDMIRX_DWC_PDEC_CTRL            (0x300UL)
/** Packet FIFO store filter enable */
#define		PFIFO_STORE_FILTER_EN	_BIT(31)
/** Packet FIFO store packet */
#define		PFIFO_STORE_PACKET		_BIT(16)
/** Packet FIFO clear min/max information */
#define		PD_FIFO_FILL_INFO_CLR	_BIT(8)
/** Packet FIFO skip one packet */
#define		PD_FIFO_SKIP			_BIT(6)
/** Packet FIFO clear */
#define		PD_FIFO_CLR				_BIT(5)
/** Packet FIFO write enable */
#define		PD_FIFO_WE				_BIT(4)
/** Packet error detection/correction enable */
#define		PDEC_BCH_EN				_BIT(0)
/** Register address: packet decoder and FIFO configuration */
#define HDMIRX_DWC_PDEC_FIFO_CFG        (0x304UL)
/** Register address: packet decoder and FIFO status */
#define HDMIRX_DWC_PDEC_FIFO_STS        (0x308UL)
/** Register address: packet decoder and FIFO byte data */
#define HDMIRX_DWC_PDEC_FIFO_DATA       (0x30CUL)
/** Register address: packet decoder and FIFO debug control */
#define HDMIRX_DWC_PDEC_DBG_CTRL        (0x310UL)
/** Register address: packet decoder and FIFO measured timing gap */
#define HDMIRX_DWC_PDEC_DBG_TMAX        (0x314UL)
/** Register address: CTS loop */
#define HDMIRX_DWC_PDEC_DBG_CTS         (0x318UL)
/** Register address: ACP frequency count */
#define HDMIRX_DWC_PDEC_DBG_ACP         (0x31CUL)
/** Register address: signal errors in data island packet */
#define HDMIRX_DWC_PDEC_DBG_ERHDMIRX_DWC_CORR   (0x320UL)
/** Register address: CTS reset measurement control */
#define HDMIRX_DWC_PDEC_ACRM_CTRL       (0x330UL)
/** Register address: maximum CTS div N value */
#define HDMIRX_DWC_PDEC_ACRM_MAX        (0x334UL)
/** Register address: minimum CTS div N value */
#define HDMIRX_DWC_PDEC_ACRM_MIN        (0x338UL)
/** Register address: audio sub packet control */
#define HDMIRX_DWC_PDEC_ASP_CTRL        (0x340UL)
/** Automatic mute all video channels */
#define		AUTO_VMUTE				_BIT(6)
/** Automatic mute audio sub packets */
#define		AUTO_SPFLAT_MUTE		MSK(4, 2)
/** Register address: audio sub packet errors */
#define HDMIRX_DWC_PDEC_ASP_ERR         (0x344UL)
/** Register address: packet decoder status, see packet interrupts */
#define HDMIRX_DWC_PDEC_STS             (0x360UL)
/** Register address: Packet Decoder Audio Status*/
#define HDMIRX_DWC_PDEC_AUD_STS         (0x364UL)
#define AUDS_RCV					MSK(1, 0)
/** Register address: general control packet AV mute */
#define HDMIRX_DWC_PDEC_GCP_AVMUTE      (0x380UL)
/** Register address: audio clock regeneration */
#define HDMIRX_DWC_PDEC_ACR_CTS        (0x390UL)
/** Audio clock regeneration, CTS parameter */
#define		CTS_DECODED				MSK(20, 0)
/** Register address: audio clock regeneration */
#define HDMIRX_DWC_PDEC_ACR_N         	(0x394UL)
/** Audio clock regeneration, N parameter */
#define		N_DECODED				MSK(20, 0)
/** Register address: auxiliary video information info frame */
#define HDMIRX_DWC_PDEC_AVI_HB         	(0x3A0UL)
/** PR3-0, pixel repetition factor */
#define 	PIX_REP_FACTOR			MSK(4, 24)
/** Register address: auxiliary video information info frame */
#define HDMIRX_DWC_PDEC_AVI_PB         	(0x3A4UL)
/** VIC6-0, video mode identification code */
#define 	VID_IDENT_CODE			MSK(7, 24)
/** ITC, IT content */
#define		IT_CONTENT				_BIT(23)
/** EC2-0, extended colorimetry */
#define		EXT_COLORIMETRY			MSK(3, 20)
/** Q1-0, RGB quantization range */
#define		RGB_QUANT_RANGE			MSK(2, 18)
/** Q1-0, YUV quantization range */
#define		YUV_QUANT_RANGE			MSK(2, 30)
/** SC1-0, non-uniform scaling information */
#define		NON_UNIF_SCALE			MSK(2, 16)
/** C1-0, colorimetry information */
#define		COLORIMETRY				MSK(2, 14)
/** M1-0, picture aspect ratio */
#define		PIC_ASPECT_RATIO		MSK(2, 12)
/** R3-0, active format aspect ratio */
#define		ACT_ASPECT_RATIO		MSK(4, 8)
/** Y1-0, video format */
#define		VIDEO_FORMAT			MSK(2, 5)
/** A0, active format information present */
#define		ACT_INFO_PRESENT		_BIT(4)
/** B1-0, bar valid information */
#define		BAR_INFO_VALID			MSK(2, 2)
/** S1-0, scan information from packet extraction */
#define		SCAN_INFO				MSK(2, 0)
/** Register address: auxiliary video information info frame */
#define HDMIRX_DWC_PDEC_AVI_TBB        	(0x3A8UL)
/** Line number to start of bottom bar */
#define		LIN_ST_BOT_BAR			MSK(16, 16)
/** Line number to end of top bar */
#define		LIN_END_TOP_BAR			MSK(16, 0)
/** Register address: auxiliary video information info frame */
#define HDMIRX_DWC_PDEC_AVI_LRB        	(0x3ACUL)
/** Pixel number of start right bar */
#define		PIX_ST_RIG_BAR			MSK(16, 16)
/** Pixel number of end left bar */
#define		PIX_END_LEF_BAR			MSK(16, 0)
/** Register address: special audio layout control for multi-channel audio */
#define HDMIRX_DWC_PDEC_AIF_CTRL       	(0x3C0UL)
/** Register address: audio info frame */
#define HDMIRX_DWC_PDEC_AIF_HB         	(0x3C4UL)
/** Register address: audio info frame */
#define HDMIRX_DWC_PDEC_AIF_PB0        	(0x3C8UL)
/** CA7-0, channel/speaker allocation */
#define 	CH_SPEAK_ALLOC			MSK(8, 24)
/** CTX, coding extension */
#define 	AIF_DATA_BYTE_3			MSK(8, 16)
/** SF2-0, sample frequency */
#define 	SAMPLE_FREQ				MSK(3, 10)
/** SS1-0, sample size */
#define 	SAMPLE_SIZE				MSK(2, 8)
/** CT3-0, coding type */
#define 	CODING_TYPE				MSK(4, 4)
/** CC2-0, channel count */
#define 	CHANNEL_COUNT			MSK(3, 0)
/** Register address: audio info frame */
#define HDMIRX_DWC_PDEC_AIF_PB1       	(0x3CCUL)
/** DM_INH, down-mix inhibit */
#define 	DWNMIX_INHIBIT			_BIT(7)
/** LSV3-0, level shift value */
#define 	LEVEL_SHIFT_VAL			MSK(4, 3)
/** Register address: gamut sequence number */
#define HDMIRX_DWC_PDEC_GMD_HB         	(0x3D0UL)
/** Register address: gamut meta data */
#define HDMIRX_DWC_PDEC_GMD_PB         	(0x3D4UL)

/*
* Vendor Specific Info Frame */
#define HDMIRX_DWC_PDEC_VSI_ST0         (0x3E0UL)
#define IEEE_REG_ID         MSK(24,0)

#define HDMIRX_DWC_PDEC_VSI_ST1         (0x3E4UL)
#define H3D_STRUCTURE       MSK(4,16)
#define H3D_EXT_DATA        MSK(4,20)
#define HDMI_VIDEO_FORMAT   MSK(3,5)

/*
 * DTL Interface registers
 */
/** Register address: dummy register for testing */
#define HDMIRX_DWC_DUMMY_IP_REG        	(0xF00UL)
/*
 * Packet Decoder Interrupt registers
 */
/** Register address: packet decoder interrupt clear enable */
#define HDMIRX_DWC_PDEC_IEN_CLR        	(0xF78UL)
/** Register address: packet decoder interrupt set enable */
#define HDMIRX_DWC_PDEC_IEN_SET        	(0xF7CUL)
/** Register address: packet decoder interrupt status */
#define HDMIRX_DWC_PDEC_ISTS           	(0xF80UL)
/** Register address: packet decoder interrupt enable */
#define HDMIRX_DWC_PDEC_IEN            	(0xF84UL)
/** Register address: packet decoder interrupt clear status */
#define HDMIRX_DWC_PDEC_ICLR           	(0xF88UL)
/** Register address: packet decoder interrupt set status */
#define HDMIRX_DWC_PDEC_ISET           	(0xF8CUL)
/** DVI detection status */
#define		DVIDET					_BIT(28)
/** AIF checksum changed */
#define		AIF_CKS_CHG				_BIT(25)
/** AVI checksum changed */
#define		AVI_CKS_CHG				_BIT(24)
/** Vendor Specific Info frame changed */
#define		VSI_CKS_CHG				_BIT(15)
/** Packet FIFO new entry */
#define		PD_FIFO_NEW_ENTRY		_BIT(8)
/** Packet FIFO overflow */
#define		PD_FIFO_OVERFL			_BIT(4)
/** Packet FIFO underflow */
#define		PD_FIFO_UNDERFL			_BIT(3)
/*
 * Audio Clock Interrupt registers
 */
/** Register address: audio clock interrupt clear enable */
#define HDMIRX_DWC_AUD_CLK_IEN_CLR     	(0xF90UL)
/** Register address: audio clock interrupt set enable */
#define HDMIRX_DWC_AUD_CLK_IEN_SET     	(0xF94UL)
/** Register address: audio clock interrupt status */
#define HDMIRX_DWC_AUD_CLK_ISTS        	(0xF98UL)
/** Register address: audio clock interrupt enable */
#define HDMIRX_DWC_AUD_CLK_IEN         	(0xF9CUL)
/** Register address: audio clock interrupt clear status */
#define HDMIRX_DWC_AUD_CLK_ICLR        	(0xFA0UL)
/** Register address: audio clock interrupt set status */
#define HDMIRX_DWC_AUD_CLK_ISET        	(0xFA4UL)
/*
 * Audio FIFO Interrupt registers
 */
/** Register address: audio FIFO interrupt clear enable */
#define HDMIRX_DWC_AUD_FIFO_IEN_CLR    	(0xFA8UL)
/** Register address: audio FIFO interrupt set enable */
#define HDMIRX_DWC_AUD_FIFO_IEN_SET    	(0xFACUL)
/** Register address: audio FIFO interrupt status */
#define HDMIRX_DWC_AUD_FIFO_ISTS       	(0xFB0UL)
/** Register address: audio FIFO interrupt enable */
#define HDMIRX_DWC_AUD_FIFO_IEN        	(0xFB4UL)
/** Register address: audio FIFO interrupt clear status */
#define HDMIRX_DWC_AUD_FIFO_ICLR       	(0xFB8UL)
/** Register address: audio FIFO interrupt set status */
#define HDMIRX_DWC_AUD_FIFO_ISET       	(0xFBCUL)
/** Audio FIFO overflow interrupt */
#define		AFIF_OVERFL				_BIT(4)
/** Audio FIFO underflow interrupt */
#define		AFIF_UNDERFL			_BIT(3)
/*
 * Mode Detection Interrupt registers
 */
/** Register address: mode detection interrupt clear enable */
#define HDMIRX_DWC_MD_IEN_CLR          	(0xFC0UL)
/** Register address: mode detection interrupt set enable */
#define HDMIRX_DWC_MD_IEN_SET          	(0xFC4UL)
/** Register address: mode detection interrupt status */
#define HDMIRX_DWC_MD_ISTS             	(0xFC8UL)
/** Register address: mode detection interrupt enable */
#define HDMIRX_DWC_MD_IEN              	(0xFCCUL)
/** Register address: mode detection interrupt clear status */
#define HDMIRX_DWC_MD_ICLR             	(0xFD0UL)
/** Register address: mode detection interrupt set status */
#define HDMIRX_DWC_MD_ISET             	(0xFD4UL)
/** Video mode interrupts */
#define		VIDEO_MODE				(MSK(3,9)|MSK(2,6)|_BIT(3))
/*
 * HDMI Interrupt registers
 */
/** Register address: HDMI interrupt clear enable */
#define HDMIRX_DWC_HDMI_IEN_CLR        	(0xFD8UL)
/** Register address: HDMI interrupt set enable */
#define HDMIRX_DWC_HDMI_IEN_SET        	(0xFDCUL)
/** Register address: HDMI interrupt status */
#define HDMIRX_DWC_HDMI_ISTS           	(0xFE0UL)
/** Register address: HDMI interrupt enable */
#define HDMIRX_DWC_HDMI_IEN            	(0xFE4UL)
/** Register address: HDMI interrupt clear status */
#define HDMIRX_DWC_HDMI_ICLR           	(0xFE8UL)
/** Register address: HDMI interrupt set status */
#define HDMIRX_DWC_HDMI_ISET           	(0xFECUL)
/** AKSV receive interrupt */
#define 	AKSV_RCV				_BIT(25)
/** Deep color mode change interrupt */
#define 	DCM_CURRENT_MODE_CHG	_BIT(16)
/** Clock change interrupt */
#define 	CLK_CHANGE				_BIT(6)
/*
 * DMI registers
 */
/** Register address: DMI software reset */
#define HDMIRX_DWC_DMI_SW_RST          (0xFF0UL)
#define		IAUDIOCLK_DOMAIN_RESET	_BIT(4)
/** Register address: DMI disable interface */
#define HDMIRX_DWC_DMI_DISABLE_IF      (0xFF4UL)
/** Register address: DMI module ID */
#define HDMIRX_DWC_DMI_MODULE_ID       (0xFFCUL)

/*
 * HDCP registers
 */
/** Register address: control */
#define HDMIRX_DWC_HDCP_CTRL			(0x0C0UL)
/** HDCP key decryption */
#define		KEY_DECRYPT_ENABLE		_BIT(1)
/** HDCP activation */
#define		HDCP_ENABLE				_BIT(0)
/** Register address: configuration */
#define HDMIRX_DWC_HDCP_SETTINGS		(0x0C4UL)
/** Register address: key description seed */
#define HDMIRX_DWC_HDCP_SEED			(0x0C8UL)
/** Register address: receiver key selection */
#define HDMIRX_DWC_HDCP_BKSV1			(0x0CCUL)
/** Register address: receiver key selection */
#define HDMIRX_DWC_HDCP_BKSV0 			(0x0D0UL)
/** Register address: key index */
#define HDMIRX_DWC_HDCP_KIDX			(0x0D4UL)
/** Register address: encrypted key */
#define HDMIRX_DWC_HDCP_KEY1			(0x0D8UL)
/** Register address: encrypted key */
#define HDMIRX_DWC_HDCP_KEY0			(0x0DCUL)
/** Register address: debug */
#define HDMIRX_DWC_HDCP_DBG				(0x0E0UL)
/** Register address: transmitter key selection vector */
#define HDMIRX_DWC_HDCP_AKSV1			(0x0E4UL)
/** Register address: transmitter key selection vector */
#define HDMIRX_DWC_HDCP_AKSV0 			(0x0E8UL)
/** Register address: session random number */
#define HDMIRX_DWC_HDCP_AN1				(0x0ECUL)
/** Register address: session random number */
#define HDMIRX_DWC_HDCP_AN0 			(0x0F0UL)
/** Register address: EESS, WOO */
#define HDMIRX_DWC_HDCP_EESS_WOO		(0x0F4UL)
/** Register address: key set writing status */
#define HDMIRX_DWC_HDCP_STS				(0x0FCUL)
/** HDCP encrypted status */
#define ENCRYPTED_STATUS			_BIT(9)
/** HDCP key set writing status */
#define		HDCP_KEY_WR_OK_STS		_BIT(0)
/** Register address: repeater KSV list control */
#define	HDMIRX_DWC_HDCP_RPT_CTRL		(0x600UL)
/** KSV list key set writing status */
#define		KSV_HOLD				_BIT(6)
/** KSV list waiting status */
#define		WAITING_KSV				_BIT(5)
/** Repeater capability */
#define		REPEATER				_BIT(3)
/** KSV list ready */
#define		KSVLIST_READY			_BIT(2)
/** Register address: repeater status */
#define	HDMIRX_DWC_HDCP_RPT_BSTATUS		(0x604UL)
/** Topology error indicator */
#define		MAX_CASCADE_EXCEEDED	_BIT(11)
/** Repeater cascade depth */
#define		DEPTH					MSK(3, 8)
/** Topology error indicator */
#define		MAX_DEVS_EXCEEDED		_BIT(7)
/** Attached downstream device count */
#define		DEVICE_COUNT			MSK(7, 0)
/** Register address: repeater KSV FIFO control */
#define	HDMIRX_DWC_HDCP_RPT_KSVFIFOCTRL	(0x608UL)
/** Register address: repeater KSV FIFO */
#define	HDMIRX_DWC_HDCP_RPT_KSVFIFO1	(0x60CUL)
/** Register address: repeater KSV FIFO */
#define	HDMIRX_DWC_HDCP_RPT_KSVFIFO0	(0x610UL)

// CEC Controller registers addresses
#define HDMIRX_DWC_CEC_CTRL                     0x1F00
#define HDMIRX_DWC_CEC_STAT                     0x1F04
#define HDMIRX_DWC_CEC_MASK                     0x1F08
#define HDMIRX_DWC_CEC_POLARITY                 0x1F0C
#define HDMIRX_DWC_CEC_INT                      0x1F10
#define HDMIRX_DWC_CEC_ADDR_L                   0x1F14
#define HDMIRX_DWC_CEC_ADDR_H                   0x1F18
#define HDMIRX_DWC_CEC_TX_CNT                   0x1F1C
#define HDMIRX_DWC_CEC_RX_CNT                   0x1F20
#define HDMIRX_DWC_CEC_TX_DATA0                 0x1F40
#define HDMIRX_DWC_CEC_TX_DATA1                 0x1F44
#define HDMIRX_DWC_CEC_TX_DATA2                 0x1F48
#define HDMIRX_DWC_CEC_TX_DATA3                 0x1F4C
#define HDMIRX_DWC_CEC_TX_DATA4                 0x1F50
#define HDMIRX_DWC_CEC_TX_DATA5                 0x1F54
#define HDMIRX_DWC_CEC_TX_DATA6                 0x1F58
#define HDMIRX_DWC_CEC_TX_DATA7                 0x1F5C
#define HDMIRX_DWC_CEC_TX_DATA8                 0x1F60
#define HDMIRX_DWC_CEC_TX_DATA9                 0x1F64
#define HDMIRX_DWC_CEC_TX_DATA10                0x1F68
#define HDMIRX_DWC_CEC_TX_DATA11                0x1F6C
#define HDMIRX_DWC_CEC_TX_DATA12                0x1F70
#define HDMIRX_DWC_CEC_TX_DATA13                0x1F74
#define HDMIRX_DWC_CEC_TX_DATA14                0x1F78
#define HDMIRX_DWC_CEC_TX_DATA15                0x1F7C
#define HDMIRX_DWC_CEC_RX_DATA0                 0x1F80
#define HDMIRX_DWC_CEC_RX_DATA1                 0x1F84
#define HDMIRX_DWC_CEC_RX_DATA2                 0x1F88
#define HDMIRX_DWC_CEC_RX_DATA3                 0x1F8C
#define HDMIRX_DWC_CEC_RX_DATA4                 0x1F90
#define HDMIRX_DWC_CEC_RX_DATA5                 0x1F94
#define HDMIRX_DWC_CEC_RX_DATA6                 0x1F98
#define HDMIRX_DWC_CEC_RX_DATA7                 0x1F9C
#define HDMIRX_DWC_CEC_RX_DATA8                 0x1FA0
#define HDMIRX_DWC_CEC_RX_DATA9                 0x1FA4
#define HDMIRX_DWC_CEC_RX_DATA10                0x1FA8
#define HDMIRX_DWC_CEC_RX_DATA11                0x1FAC
#define HDMIRX_DWC_CEC_RX_DATA12                0x1FB0
#define HDMIRX_DWC_CEC_RX_DATA13                0x1FB4
#define HDMIRX_DWC_CEC_RX_DATA14                0x1FB8
#define HDMIRX_DWC_CEC_RX_DATA15                0x1FBC
#define HDMIRX_DWC_CEC_LOCK                     0x1FC0
#define HDMIRX_DWC_CEC_WKUPCTRL                 0x1FC4

#endif
