/*
 * Amlogic G9TV
 * HDMI RX
 * Copyright (C) 2014 Amlogic, Inc.
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
#ifdef CONFIG_TVIN_VDIN_CTRL
//#define CONFIG_AML_AUDIO_DSP
#endif
#ifdef CONFIG_AML_AUDIO_DSP
#define M2B_IRQ0_DSP_AUDIO_EFFECT (7)
#define DSP_CMD_SET_HDMI_SR   (6)
extern int  mailbox_send_audiodsp(int overwrite,int num,int cmd,const char *data,int len);
#endif

#define HDMI_STATE_CHECK_FREQ     (20*5)
#define HW_MONITOR_TIME_UNIT    (1000/HDMI_STATE_CHECK_FREQ)

static int audio_enable = 1;
MODULE_PARM_DESC(audio_enable, "\n audio_enable \n");
module_param(audio_enable, int, 0664);

static int sample_rate_change_th = 1000;
MODULE_PARM_DESC(sample_rate_change_th, "\n sample_rate_change_th \n");
module_param(sample_rate_change_th, int, 0664);

static int audio_sample_rate_stable_count_th = 15; // audio change <--> audio_sample_rate is stable
MODULE_PARM_DESC(audio_sample_rate_stable_count_th, "\n audio_sample_rate_stable_count_th \n");
module_param(audio_sample_rate_stable_count_th, int, 0664);

static int sig_pll_unlock_cnt = 0;			//signal unstable PLL unlock
static int sig_pll_unlock_max = 100;
MODULE_PARM_DESC(sig_pll_unlock_max, "\n sig_pll_unlock_max \n");
module_param(sig_pll_unlock_max, int, 0664);

static int sig_pll_lock_cnt = 0;
static unsigned sig_pll_lock_max = 20;		//signal unstable PLL lock
MODULE_PARM_DESC(sig_pll_lock_max, "\n sig_pll_lock_max \n");
module_param(sig_pll_lock_max, int, 0664);

static bool force_hdmi_5v_high = false;
MODULE_PARM_DESC(force_hdmi_5v_high, "\n force_hdmi_5v_high \n");
module_param(force_hdmi_5v_high, bool, 0664);

static int sig_lost_lock_cnt = 0;	//signal ready PLL lock --> unlock
static int sig_lost_lock_max = 3;	//10->6 tiammao box timingchange issue
MODULE_PARM_DESC(sig_lost_lock_max, "\n sig_lost_lock_max \n");
module_param(sig_lost_lock_max, int, 0664);

static int sig_stable_cnt = 0;
static int sig_stable_max = 30;
MODULE_PARM_DESC(sig_stable_max, "\n sig_stable_max \n");
module_param(sig_stable_max, int, 0664);

static int hpd_wait_cnt = 0;
static int hpd_wait_max = 10;
MODULE_PARM_DESC(hpd_wait_max, "\n hpd_wait_max \n");
module_param(hpd_wait_max, int, 0664);

static int sig_unstable_cnt = 0;
static int sig_unstable_max = 50;
MODULE_PARM_DESC(sig_unstable_max, "\n sig_unstable_max \n");
module_param(sig_unstable_max, int, 0664);

static int sig_unready_cnt = 0;
static int sig_unready_max = 6;//10;
MODULE_PARM_DESC(sig_unready_max, "\n sig_unready_max \n");
module_param(sig_unready_max, int, 0664);

static bool enable_hpd_reset = false;
MODULE_PARM_DESC(enable_hpd_reset, "\n enable_hpd_reset \n");
module_param(enable_hpd_reset, bool, 0664);

static int pow5v_max_cnt = 4;
MODULE_PARM_DESC(pow5v_max_cnt, "\n pow5v_max_cnt \n");
module_param(pow5v_max_cnt, int, 0664);

static int sig_unstable_reset_hpd_cnt = 0;
static int sig_unstable_reset_hpd_max = 5;
MODULE_PARM_DESC(sig_unstable_reset_hpd_max, "\n sig_unstable_reset_hpd_max \n");
module_param(sig_unstable_reset_hpd_max, int, 0664);

int rgb_quant_range = 0;
MODULE_PARM_DESC(rgb_quant_range, "\n rgb_quant_range \n");
module_param(rgb_quant_range, int, 0664);

int yuv_quant_range = 0;
MODULE_PARM_DESC(yuv_quant_range, "\n yuv_quant_range \n");
module_param(yuv_quant_range, int, 0664);

static int it_content = 0;
MODULE_PARM_DESC(it_content, "\n it_content \n");
module_param(it_content, int, 0664);

static bool current_port_hpd_ctl = true;
MODULE_PARM_DESC(current_port_hpd_ctl, "\n current_port_hpd_ctl \n");
module_param(current_port_hpd_ctl, bool, 0664);

static int reset_phy_level = 2;
MODULE_PARM_DESC(reset_phy_level, "\n reset_phy_level \n");
module_param(reset_phy_level, int, 0664);

static int force_format = 0;
MODULE_PARM_DESC(force_format, "\n force_format \n");
module_param(force_format, int, 0664);
/* timing diff offset */
static int diff_pixel_th = 30;
static int diff_line_th = 20;
static int diff_frame_th = 50; /* 50*10 khz offset */
MODULE_PARM_DESC(diff_pixel_th, "\n diff_pixel_th \n");
module_param(diff_pixel_th, int, 0664);
MODULE_PARM_DESC(diff_line_th, "\n diff_line_th \n");
module_param(diff_line_th, int, 0664);
MODULE_PARM_DESC(diff_frame_th, "\n diff_frame_th \n");
module_param(diff_frame_th, int, 0664);

static int port_map = 0x3210;
MODULE_PARM_DESC(port_map, "\n port_map \n");
module_param(port_map, int, 0664);

static int cfg_clk = 24000; //510/20*1000
MODULE_PARM_DESC(cfg_clk, "\n cfg_clk \n");
module_param(cfg_clk, int, 0664);

static int lock_thres = LOCK_THRES;
MODULE_PARM_DESC(lock_thres, "\n lock_thres \n");
module_param(lock_thres, int, 0664);

static int fast_switching = 0;//1;
MODULE_PARM_DESC(fast_switching, "\n fast_switching \n");
module_param(fast_switching, int, 0664);

static int fsm_enhancement = 1;
MODULE_PARM_DESC(fsm_enhancement, "\n fsm_enhancement \n");
module_param(fsm_enhancement, int, 0664);

static int port_select_ovr_en = 0;
MODULE_PARM_DESC(port_select_ovr_en, "\n port_select_ovr_en \n");
module_param(port_select_ovr_en, int, 0664);

static int phy_cmu_config_force_val = 0;
MODULE_PARM_DESC(phy_cmu_config_force_val, "\n phy_cmu_config_force_val \n");
module_param(phy_cmu_config_force_val, int, 0664);

static int phy_system_config_force_val = 0;
MODULE_PARM_DESC(phy_system_config_force_val, "\n phy_system_config_force_val \n");
module_param(phy_system_config_force_val, int, 0664);

static int acr_mode = 0;// Select which ACR scheme:// 0=Analog PLL based ACR;1=Digital ACR.
MODULE_PARM_DESC(acr_mode, "\n acr_mode \n");
module_param(acr_mode, int, 0664);

static int edid_mode = 0;
MODULE_PARM_DESC(edid_mode, "\n edid_mode \n");
module_param(edid_mode, int, 0664);

static int force_vic = 0;
MODULE_PARM_DESC(force_vic, "\n force_vic \n");
module_param(force_vic, int, 0664);

static int force_ready = 0;
MODULE_PARM_DESC(force_ready, "\n force_ready \n");
module_param(force_ready, int, 0664);

static int repeat_check = 1;
MODULE_PARM_DESC(repeat_check, "\n repeat_check \n");
module_param(repeat_check, int, 0664);

static int force_state = 0;
MODULE_PARM_DESC(force_state, "\n force_state \n");
module_param(force_state, int, 0664);

static int force_audio_sample_rate = 0;
MODULE_PARM_DESC(force_audio_sample_rate, "\n force_audio_sample_rate \n");
module_param(force_audio_sample_rate, int, 0664);

static int audio_sample_rate = 0;//used in other module
MODULE_PARM_DESC(audio_sample_rate, "\n audio_sample_rate \n");
module_param(audio_sample_rate, int, 0664);

static int auds_rcv_sts = 0;
MODULE_PARM_DESC(auds_rcv_sts, "\n auds_rcv_sts \n");
module_param(auds_rcv_sts, int, 0664);

static int audio_coding_type = 0;
MODULE_PARM_DESC(audio_coding_type, "\n audio_coding_type \n");
module_param(audio_coding_type, int, 0664);

static int audio_channel_count = 0;
MODULE_PARM_DESC(audio_channel_count, "\n audio_channel_count \n");
module_param(audio_channel_count, int, 0664);

static int frame_rate = 0;
MODULE_PARM_DESC(frame_rate, "\n frame_rate \n");
module_param(frame_rate, int, 0664);

static bool use_eq_cal = 0;
MODULE_PARM_DESC(use_eq_cal, "\n use_eq_cal \n");
module_param(use_eq_cal, bool, 0664);

static int tmds_valid_cnt_max = 20;
MODULE_PARM_DESC(tmds_valid_cnt_max, "\n tmds_valid_cnt_max \n");
module_param(tmds_valid_cnt_max, int, 0664);

static int dwc_rst_wait_cnt_max = 0;
MODULE_PARM_DESC(dwc_rst_wait_cnt_max, "\n dwc_rst_wait_cnt_max \n");
module_param(dwc_rst_wait_cnt_max, int, 0664);

//static int sw_reset_edid=0;

static bool port0_need_run_EQ = true;
static bool port1_need_run_EQ = true;
static bool port2_need_run_EQ = true;


#define FSM_LOG_ENABLE		0x01
#define VIDEO_LOG_ENABLE	0x02
#define AUDIO_LOG_ENABLE	0x04
#define PACKET_LOG_ENABLE   0x08
#define CEC_LOG_ENABLE		0x10
#define REG_LOG_ENABLE		0x20
//0x100--irq print;
//0x200-other print
/* bit 0, printk; bit 8 enable irq log */
int hdmirx_log_flag = 0x1;
MODULE_PARM_DESC(hdmirx_log_flag, "\n hdmirx_log_flag \n");
module_param(hdmirx_log_flag, int, 0664);

static bool frame_skip_en = 1;  //skip frame when signal unstable
MODULE_PARM_DESC(frame_skip_en, "\n frame_skip_en \n");
module_param(frame_skip_en, bool, 0664);

static bool use_frame_rate_check = 0;//1; /* enable/disable frame rate check  */
MODULE_PARM_DESC(use_frame_rate_check, "\n use_frame_rate_check \n");
module_param(use_frame_rate_check, bool, 0664);

//video mode clock above or below 3.4Gbps
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
static int pre_tmds_clk_div4 = 0;
static int force_clk_rate = 0;
#else
static int pre_tmds_clk_div4 = 0;
static int force_clk_rate = 0x10;  //0x10 force 0,0x11 force 1
#endif
MODULE_PARM_DESC(force_clk_rate, "\n force_clk_rate \n");
module_param(force_clk_rate, int, 0664);

static bool hw_dbg_en = 1; /* only for hardware test */
MODULE_PARM_DESC(hw_dbg_en, "\n hw_dbg_en \n");
module_param(hw_dbg_en, bool, 0664);

static int last_color_fmt = 0;
static bool reset_sw = true;
static int sm_pause = 0;
//static int EQ_state = 0;
int eq_setting = 0;
//extern int eq_setting_ch0;
//extern int eq_setting_ch1;
//extern int eq_setting_ch2;
static int tmds_valid_cnt = 0;
static int dwc_rst_wait_cnt = 0;
static int wait_clk_stable = 0;

//static int dwc_rst_wait_cnt_max = 30;


/***********************
  TVIN driver interface
************************/

#define HDMIRX_HWSTATE_INIT					0
#define HDMIRX_HWSTATE_HDMI5V_LOW			1
#define HDMIRX_HWSTATE_HDMI5V_HIGH			2
#define HDMIRX_HWSTATE_HPD_READY			3
#define HDMIRX_HWSTATE_SIG_UNSTABLE         4
#define HDMIRX_HWSTATE_DWC_RST_WAIT			5
#define HDMIRX_HWSTATE_SIG_STABLE			6
#define HDMIRX_HWSTATE_TIMINGCHANGE			7
#define HDMIRX_HWSTATE_SIG_READY			8
#define HDMIRX_HWSTATE_EQ_CALIBRATION		9
#define HDMIRX_HWSTATE_WAIT_CLK_STABLE		10


struct rx rx;

static unsigned long tmds_clock_old = 0;

/** TMDS clock delta [kHz] */
#define TMDS_CLK_DELTA			(125)
/** Pixel clock minimum [kHz] */
#define PIXEL_CLK_MIN			TMDS_CLK_MIN
/** Pixel clock maximum [kHz] */
#define PIXEL_CLK_MAX			TMDS_CLK_MAX
/** Horizontal resolution minimum */
#define HRESOLUTION_MIN			(320)
/** Horizontal resolution maximum */
#define HRESOLUTION_MAX			(4096)
/** Vertical resolution minimum */
#define VRESOLUTION_MIN			(240)
/** Vertical resolution maximum */
#define VRESOLUTION_MAX			(4455)
/** Refresh rate minimum [Hz] */
#define REFRESH_RATE_MIN		(100)
/** Refresh rate maximum [Hz] */
#define REFRESH_RATE_MAX		(25000)

#define TMDS_TOLERANCE  (4000)
#define MAX_AUDIO_SAMPLE_RATE		(192000+1000)	//192K
#define MIN_AUDIO_SAMPLE_RATE		(8000-1000)		//8K


int hdmi_rx_ctrl_edid_update(void);
static void dump_state(unsigned char enable);
static void dump_audio_info(unsigned char enable);
//static unsigned int get_vic_from_timing(struct hdmi_rx_ctrl_video* video_par);
static unsigned int get_index_from_ref(struct hdmi_rx_ctrl_video* video_par);
void hdmirx_hw_init2(void);



/**
 * Clock event handler
 * @param[in,out] ctx context information
 * @return error code
 */

static long hdmi_rx_ctrl_get_tmds_clk(struct hdmi_rx_ctrl *ctx)
{
	return ctx->tmds_clk;
}

static int clock_handler(struct hdmi_rx_ctrl *ctx)
{
	int error = 0;
	unsigned long tclk = 0;

    if (sm_pause) {
        return 0;
    }

	if (ctx == 0)	{
		return -EINVAL;
	}
	tclk = hdmi_rx_ctrl_get_tmds_clk(ctx);
	//hdmirx_get_video_info(ctx, &rx.video_params);
	if (((tmds_clock_old + TMDS_TOLERANCE) > tclk) &&
		((tmds_clock_old - TMDS_TOLERANCE) < tclk)) {
		return 0;
	}
	if ((tclk == 0) && (tmds_clock_old != 0)) {
		//if(video_format_change())
	    rx.change = sig_unready_max;
		if (hdmirx_log_flag & 0x100) {
		    printk("[hdmirx clock_handler] clk change\n");
	}
		/* TODO Review if we need to turn off the display
		video_if_mode(false, 0, 0, 0);
		*/
#if 0
		/* workaround for sticky HDMI mode */
		error |= rx_ctrl_config(ctx, rx.port, &rx.hdcp);
#endif
	}
	else
	{
#if 0
		error |= hdmi_rx_phy_config(&rx.phy, rx.port, tclk, v.deep_color_mode);
#endif
	}
	tmds_clock_old = ctx->tmds_clk;
//#if MESSAGES
	//ctx->log_info("TMDS clock: %3u.%03uMHz",
	//		ctx->tmds_clk / 1000, ctx->tmds_clk % 1000);
//#endif
	return error;
}
#if 0
static unsigned char is_3d_sig(void)
{
	if((rx.vendor_specific_info.identifier == 0x000c03)&&
	(rx.vendor_specific_info.hdmi_video_format == 0x2)){
		return 1;
	}
		return 0;
}
#endif
#if 0
/*
*make sure if video format change
*/
bool video_format_change(void)
{
	if(is_3d_sig()){	//only for 3D sig
		if (((rx.cur_video_params.hactive + 5) < (rx.reltime_video_params.hactive)) ||
			((rx.cur_video_params.hactive - 5) > (rx.reltime_video_params.hactive)) ||
			((rx.cur_video_params.vactive + 5) < (rx.reltime_video_params.vactive)) ||
			((rx.cur_video_params.vactive - 5) > (rx.reltime_video_params.vactive)) ||
			(rx.cur_video_params.pixel_repetition != rx.reltime_video_params.pixel_repetition)) {

			if(hdmirx_log_flag&0x200){
				printk("[hdmi] 3D timing change: hactive(%d=>%d), vactive(%d=>%d), pixel_repeat(%d=>%d), video_format(%d=>%d),video_VIC(%d=>%d)\n",
				rx.cur_video_params.hactive, rx.reltime_video_params.hactive,
				rx.cur_video_params.vactive, rx.reltime_video_params.vactive,
				rx.cur_video_params.pixel_repetition, rx.reltime_video_params.pixel_repetition,
				rx.cur_video_params.video_format, rx.reltime_video_params.video_format,
				rx.cur_video_params.video_mode, rx.reltime_video_params.video_mode);
			}
			return true;

		} else {
			return false;
		}
	} else {	// for 2d sig
		return true;
	}
}

bool is_format_change(struct hdmi_rx_ctrl *ctx)
{
	int error = 0;
	int vic = 0;
	bool ret = false;
	struct hdmi_rx_ctrl_video v;

	if (ctx == 0) {
		return -EINVAL;
	}

	error |= hdmirx_get_video_info(ctx, &v);
	if ((error == 0) && (((rx.cur_video_params.hactive + 5) < (v.hactive)) ||
			     ((rx.cur_video_params.hactive - 5) > (v.hactive)) ||
			     ((rx.cur_video_params.vactive + 5) < (v.vactive)) ||
			     ((rx.cur_video_params.vactive - 5) > (v.vactive)) ||
			     (rx.cur_video_params.pixel_repetition != v.pixel_repetition)))
	{
		ret = true;
		if(hdmirx_log_flag&0x200){
			printk("HDMI mode change: hactive(%d=>%d), vactive(%d=>%d), pixel_repeat(%d=>%d), video_format(%d=>%d)\n",
			rx.cur_video_params.hactive,		v.hactive,
			rx.cur_video_params.vactive,		v.vactive,
			rx.cur_video_params.pixel_repetition,	v.pixel_repetition,
			rx.cur_video_params.video_format,	v.video_format);
		}

	}

	return ret;
}
#endif

/**
 * Video event handler
 * @param[in,out] ctx context information
 * @return error code
 */
static int video_handler(struct hdmi_rx_ctrl *ctx)
{
	int error = 0;
	//struct hdmi_rx_ctrl_video v;
	//struct hdmi_rx_ctrl_video *pre = &rx.pre_video_params;

	if(hdmirx_log_flag&0x100)
	hdmirx_print("%s \n", __func__);

    if(sm_pause){
        return 0;
    }

	if (ctx == 0) {
		return -EINVAL;
	}
	#if 0
	error |= hdmirx_get_video_info(ctx, &v);
	if ((error == 0) &&
	    ((abs((signed int)pre->hactive -(signed int)v.hactive) > diff_pixel_th) ||
	     (abs((signed int)pre->vactive -(signed int)v.vactive) > diff_line_th) ||
	     (pre->pixel_repetition != v.pixel_repetition)))	{

	    /* only state is ready need skip frame when signal change */
	    if (rx.state != HDMIRX_HWSTATE_SIG_READY)
	        return 0;
	    rx.change = sig_unready_max;
	    /* need check ... */
	    //if (get_vic_from_timing(&v) !=0) {
	    if (hdmirx_log_flag&0x100) {
	        printk("[HDMIrx video_handler] VIC(%d=>%d) hactive(%d=>%d), vactive(%d=>%d), pixel_repeat(%d=>%d), video_format(%d=>%d)\n",
	             pre->video_mode, v.video_mode, pre->hactive, v.hactive,
	             pre->vactive, v.vactive, pre->pixel_repetition, v.pixel_repetition,
	             pre->video_format, v.video_format);
	    }
	}
	#endif

	return error;
}

static int vsi_handler(void)
{
    if (sm_pause) {
        return 0;
    }
	//hdmirx_read_vendor_specific_info_frame(&rx.vendor_specific_info.identifier);
	hdmirx_read_vendor_specific_info_frame(&rx.vendor_specific_info);
	return 0;
}

/**
 * Audio event handler
 * @param[in,out] ctx context information
 * @return error code
 */
#if 0
static int audio_handler(struct hdmi_rx_ctrl *ctx)
{
	int error = 0;
  /*
	struct hdmi_rx_ctrl_audio a;
  if(hdmirx_log_flag&0x100)
    hdmirx_print("%s \n", __func__);

	if (ctx == 0)
	{
		return -EINVAL;
	}
	error |= hdmi_rx_ctrl_get_audio(ctx, &a);
	if (error == 0)
	{
		ctx->log_info("Audio: CT=%u CC=%u SF=%u SS=%u CA=%u",
				a.coding_type, a.channel_count, a.sample_frequency,
				a.sample_size, a.channel_allocation);
	}
	*/
	//hdmirx_read_audio_info();

	return error;
}
#endif

static int hdmi_rx_ctrl_irq_handler(struct hdmi_rx_ctrl *ctx)
{
    int error = 0;
    unsigned i = 0;
    uint32_t intr_hdmi = 0;
    uint32_t intr_md = 0;
#ifdef CEC_FUNC_ENABLE
	uint32_t intr_cec = 0;
#endif
    uint32_t intr_pedc = 0;
    //uint32_t intr_aud_clk = 0;
    uint32_t intr_aud_fifo = 0;
    uint32_t data = 0;
    unsigned long tclk = 0;
    unsigned long ref_clk;
    unsigned evaltime = 0;

    bool clk_handle_flag = false;
    bool video_handle_flag = false;
    //bool audio_handle_flag = false;
#ifdef CEC_FUNC_ENABLE
    bool cec_get_msg_flag = false;    //cec
    bool cec_get_ack_flag = false;
#endif
    bool vsi_handle_flag = false;

    ref_clk = ctx->md_clk;

    /* clear interrupt quickly */
    intr_hdmi = hdmirx_rd_dwc(HDMIRX_DWC_HDMI_ISTS) & hdmirx_rd_dwc(HDMIRX_DWC_HDMI_IEN);
    if (intr_hdmi != 0) {
        hdmirx_wr_dwc(HDMIRX_DWC_HDMI_ICLR, intr_hdmi);
    }

    intr_md = hdmirx_rd_dwc(HDMIRX_DWC_MD_ISTS) & hdmirx_rd_dwc(HDMIRX_DWC_MD_IEN);
    if (intr_md != 0) {
        hdmirx_wr_dwc(HDMIRX_DWC_MD_ICLR, intr_md);
    }

    intr_pedc = hdmirx_rd_dwc(HDMIRX_DWC_PDEC_ISTS) & hdmirx_rd_dwc(HDMIRX_DWC_PDEC_IEN);
    if (intr_pedc != 0) {
        hdmirx_wr_dwc(HDMIRX_DWC_PDEC_ICLR, intr_pedc);
    }

    //intr_aud_clk = hdmirx_rd_dwc(RA_AUD_CLK_ISTS) & hdmirx_rd_dwc(RA_AUD_CLK_IEN);
    //if (intr_aud_clk != 0) {
    //    hdmirx_wr_dwc(RA_AUD_CLK_ICLR, intr_aud_clk);
    //}
#ifdef CEC_FUNC_ENABLE
	intr_cec = hdmirx_rd_dwc(HDMIRX_DWC_AUD_CLK_ISTS) & hdmirx_rd_dwc(HDMIRX_DWC_AUD_CLK_IEN);
	if (intr_cec != 0) {
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_CLK_ICLR, intr_cec);
	}
	// EOM irq
	if(intr_cec & (1 << 17)) {
		if(hdmirx_log_flag&CEC_LOG_ENABLE)
    		hdmirx_print("\nEOM \n");
		cec_get_msg_flag = true;
	}
	// ACK irq
	if(intr_cec & (1 << 16)) {
		if(hdmirx_log_flag&CEC_LOG_ENABLE)
    		hdmirx_print("\nACK \n");
		cec_get_ack_flag = true;
	}
#endif

    intr_aud_fifo = hdmirx_rd_dwc(HDMIRX_DWC_AUD_FIFO_ISTS) & hdmirx_rd_dwc(HDMIRX_DWC_AUD_FIFO_IEN);
    if (intr_aud_fifo != 0) {
        hdmirx_wr_dwc(HDMIRX_DWC_AUD_FIFO_ICLR, intr_aud_fifo);
    }

    if (intr_hdmi != 0) {
        if (get(intr_hdmi, CLK_CHANGE) != 0) {
            clk_handle_flag = true;
            evaltime = (ref_clk * 4095) / 158000;
            data = hdmirx_rd_dwc(HDMIRX_DWC_HDMI_CKM_RESULT);
            tclk = ((ref_clk * get(data, CLKRATE)) / evaltime);
            if(hdmirx_log_flag&0x100)
                hdmirx_print("[HDMIrx isr] CLK_CHANGE (%d) \n", tclk);
            if (tclk == 0) {
                //error |= hdmirx_interrupts_hpd(false);
                error |= hdmirx_control_clk_range(TMDS_CLK_MIN, TMDS_CLK_MAX);
            } else {
                for (i = 0; i < TMDS_STABLE_TIMEOUT; i++) { /* time for TMDS to stabilise */
                    ;
                }
                tclk = ((ref_clk * get(data, CLKRATE)) / evaltime);
                error |= hdmirx_control_clk_range(tclk - TMDS_CLK_DELTA, tclk + TMDS_CLK_DELTA);
                //error |= hdmirx_interrupts_hpd(true);
            }
            ctx->tmds_clk = tclk;
        }
        if (get(intr_hdmi, DCM_CURRENT_MODE_CHG) != 0) {
            if(hdmirx_log_flag&0x400)
                hdmirx_print("[HDMIrx isr] DMI DCM_CURRENT_MODE_CHG \n");
            video_handle_flag = true;
        }
        //if (get(intr_hdmi, AKSV_RCV) != 0) {
        //    if(hdmirx_log_flag&0x400)
        //        hdmirx_print("[HDMIrx isr] AKSV_RCV \n");
        //        //execute[hdmi_rx_ctrl_event_aksv_reception] = true;
        //}
        ctx->debug_irq_hdmi++;
    }

    if (intr_md != 0) {
        if (get(intr_md, VIDEO_MODE) != 0) {
            if(hdmirx_log_flag&0x400)
                hdmirx_print("[HDMIrx isr] VIDEO_MODE: %x\n", intr_md);
            video_handle_flag = true;
        }
        ctx->debug_irq_video_mode++;
    }

    if (intr_pedc != 0) {
        //hdmirx_wr_dwc(RA_PDEC_ICLR, intr_pedc);
        if (get(intr_pedc, DVIDET|AVI_CKS_CHG) != 0) {
            if(hdmirx_log_flag&0x400)
                hdmirx_print("[HDMIrx isr] AVI_CKS_CHG \n");
            video_handle_flag = true;
        }
        if (get(intr_pedc, VSI_CKS_CHG) != 0) {
            if(hdmirx_log_flag&0x400)
                hdmirx_print("[HDMIrx isr] VSI_CKS_CHG \n");
            vsi_handle_flag = true;
        }
        //if (get(intr_pedc, AIF_CKS_CHG) != 0) {
        //    if(hdmirx_log_flag&0x400)
        //        hdmirx_print("[HDMIrx isr] AIF_CKS_CHG \n");
        //    audio_handle_flag = true;
        //}
        //if (get(intr_pedc, PD_FIFO_NEW_ENTRY) != 0) {
        //    if(hdmirx_log_flag&0x400)
        //        hdmirx_print("[HDMIrx isr] PD_FIFO_NEW_ENTRY \n");
        //    //execute[hdmi_rx_ctrl_event_packet_reception] = true;
        //}
        if (get(intr_pedc, PD_FIFO_OVERFL) != 0) {
            if(hdmirx_log_flag&0x100)
                hdmirx_print("[HDMIrx isr] PD_FIFO_OVERFL \n");
            error |= hdmirx_packet_fifo_rst();
        }
        ctx->debug_irq_packet_decoder++;
    }

    //if (intr_aud_clk != 0) {
    //    if(hdmirx_log_flag&0x400)
    //        hdmirx_print("[HDMIrx isr] RA_AUD_CLK \n");
    //        ctx->debug_irq_audio_clock++;
    //}

    if (intr_aud_fifo != 0) {
        if (get(intr_aud_fifo, AFIF_OVERFL|AFIF_UNDERFL) != 0) {
            if(hdmirx_log_flag&0x100)
                hdmirx_print("[HDMIrx isr] AFIF_OVERFL|AFIF_UNDERFL \n");
            error |= hdmirx_audio_fifo_rst();
        }
        ctx->debug_irq_audio_fifo++;
    }

    if(clk_handle_flag){
        clock_handler(ctx);
    }
    if(video_handle_flag){
        video_handler(ctx);
    }
    if(vsi_handle_flag){
        vsi_handler();
    }
#ifdef CEC_FUNC_ENABLE
	if((cec_get_msg_flag)||(cec_get_ack_flag)){
		cec_handler(cec_get_msg_flag,cec_get_ack_flag);
	}
#endif
    return error;
}


static irqreturn_t irq_handler(int irq, void *params)
{
    int error = 0;
    unsigned long hdmirx_top_intr_stat;
    if (params == 0) {
        pr_info("%s: %s\n", __func__, "RX IRQ invalid parameter");
        return IRQ_HANDLED;
    }

    hdmirx_top_intr_stat = hdmirx_rd_top(HDMIRX_TOP_INTR_STAT);
reisr:    hdmirx_wr_top(HDMIRX_TOP_INTR_STAT_CLR, hdmirx_top_intr_stat); // clear interrupts in HDMIRX-TOP module

    /* must clear ip interrupt quickly */
    if(hdmirx_top_intr_stat & (1 << 31)){
        error = hdmi_rx_ctrl_irq_handler(&((struct rx *)params)->ctrl);
        if (error < 0) {
            if (error != -EPERM) {
                pr_info("%s: RX IRQ handler %d\n", __func__, error);
            }
        }
    }

    /* top interrupt handler */
    if (hdmirx_top_intr_stat & (0xf << 2)) {    // [5:2] hdmirx_5v_rise
        //rx.tx_5v_status = true;
        if(hdmirx_log_flag&0x400)
            printk("[HDMIrx isr] 5v rise \n");
    } /* if (hdmirx_top_intr_stat & (0xf << 2)) // [5:2] hdmirx_5v_rise */

    if (hdmirx_top_intr_stat & (0xf << 6)) {    // [9:6] hdmirx_5v_fall
        //rx.tx_5v_status = false;
        if(hdmirx_log_flag&0x400)
            printk("[HDMIrx isr] 5v fall \n");
    } /* if (hdmirx_top_intr_stat & (0xf << 6)) // [9:6] hdmirx_5v_fall */

    /* check the ip interrupt again */
    hdmirx_top_intr_stat = hdmirx_rd_top(HDMIRX_TOP_INTR_STAT);
    if (hdmirx_top_intr_stat & (1 << 31)){
        if(hdmirx_log_flag&0x1000)
            printk("[HDMIrx isr] need clear ip irq--- \n");
        goto reisr;

    }
    return IRQ_HANDLED;

}



typedef struct{
	unsigned int sample_rate;
	unsigned char aud_info_sf;
	unsigned char channel_status_id;
}sample_rate_info_t;

sample_rate_info_t sample_rate_info[]=
{
	{32000,  0x1,  0x3},
	{44100,  0x2,  0x0},
	{48000,  0x3,  0x2},
	{88200,  0x4,  0x8},
	{96000,  0x5,  0xa},
	{176400, 0x6,  0xc},
	{192000, 0x7,  0xe},
	//{768000, 0, 0x9},
	{0, 0, 0}
};

static int get_real_sample_rate(void)
{
    int i;
    int ret_sample_rate = rx.aud_info.audio_recovery_clock;
    for(i=0; sample_rate_info[i].sample_rate; i++){
        if(rx.aud_info.audio_recovery_clock > sample_rate_info[i].sample_rate){
            if((rx.aud_info.audio_recovery_clock-sample_rate_info[i].sample_rate)<sample_rate_change_th){
                ret_sample_rate = sample_rate_info[i].sample_rate;
                break;
            }
        }
        else{
            if((sample_rate_info[i].sample_rate - rx.aud_info.audio_recovery_clock)<sample_rate_change_th){
                ret_sample_rate = sample_rate_info[i].sample_rate;
                break;
            }
        }
    }
    return ret_sample_rate;
}

static unsigned char is_sample_rate_stable(int sample_rate_pre, int sample_rate_cur)
{
    unsigned char ret = 0;
	if(sample_rate_pre > sample_rate_cur){
		if((sample_rate_pre - sample_rate_cur) < sample_rate_change_th){
			ret = 1;
		}
	}else{
		if((sample_rate_cur - sample_rate_pre) < sample_rate_change_th){
			ret = 1;
		}
	}
    return ret;
}

bool hdmirx_hw_check_frame_skip(void)
{
    if ((force_state&0x10) || (!frame_skip_en))
        return false;
    else if ((rx.state != HDMIRX_HWSTATE_SIG_READY) || (rx.change > 0)) {
	    return true;
    }

    return false;
}

int hdmirx_hw_get_color_fmt(void)
{
	int color_format = 0;
	int format = rx.pre_video_params.video_format;

	if(force_format&0x10){
		format = force_format&0xf;
	}
	if (rx.change > 0)
	    return last_color_fmt;

	switch(format){
	case 1:
		color_format = 3; /* YUV422 */ // yuv422->444 automatically in top level
		break;
	case 2:
		color_format = 1; /* YUV444*/
		break;
	case 3://YUV420
		color_format = 1; /* YUV444*/
		break;
	case 0:
	default:
		color_format = 0; /* RGB444 */
		break;
		}

	last_color_fmt = color_format;

	return color_format;
}

int hdmirx_hw_get_dvi_info(void)
{
	int ret = 0;

	if(rx.pre_video_params.sw_dvi){
		ret = 1;
	}
	return ret;
}

int hdmirx_hw_get_3d_structure(unsigned char* _3d_structure, unsigned char* _3d_ext_data)
{
	if((rx.vendor_specific_info.identifier == 0x000c03)&&
	(rx.vendor_specific_info.hdmi_video_format == 0x2)){
	*_3d_structure = rx.vendor_specific_info._3d_structure;
	*_3d_ext_data = rx.vendor_specific_info._3d_ext_data;
	return 0;
	}
	return -1;
}

int hdmirx_hw_get_pixel_repeat(void)
{
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9TV)
	return (rx.pre_video_params.pixel_repetition+1);
#else
	/* use hdmirx hscaler for 4k2k input */
    if ((rx.pre_video_params.sw_vic >= HDMI_3840_2160p) && (rx.pre_video_params.sw_vic <= HDMI_4096_2160p))
        return (2);
    else
        return (rx.pre_video_params.pixel_repetition+1);
#endif
}


unsigned char is_frame_packing(void)
{

#if 1
    return (rx.pre_video_params.sw_fp);
#else
    if((rx.vendor_specific_info.identifier == 0x000c03)&&
    (rx.vendor_specific_info.hdmi_video_format == 0x2)&&
    (rx.vendor_specific_info._3d_structure == 0x0)){
    return 1;
    }
    return 0;
#endif
}

unsigned char is_alternative(void)
{
    return (rx.pre_video_params.sw_alternative);
}

typedef struct{
    unsigned int vic;
    unsigned char vesa_format;
    unsigned int ref_freq; /* 8 bit tmds clock */
    uint16_t active_pixels;
    uint16_t active_lines;
    uint16_t active_lines_fp;
    uint16_t active_lines_alternative;
	uint8_t repetition_times;
    uint16_t frame_rate;
}freq_ref_t;

static freq_ref_t freq_ref[]=
{
/* basic format*/
	{HDMI_640x480p60,	0,	25000,	640,	480,	480,	480,	0,	3000},
	{HDMI_480p60,		0,  27000,  720,	480,	1005,	480,	0,	3000},
	{HDMI_480p60_16x9,	0,  27000,  720,	480,	1005,	480,	0,	3000},
	{HDMI_480i60,		0,  27000,	1440,	240,	240,	240,	1,	3000},
	{HDMI_480i60_16x9,	0,  27000,	1440,	240,	240,	240,	1,	3000},
	{HDMI_576p50,		0,  27000,  720,	576,	1201,	576,	0,	2500},
	{HDMI_576p50_16x9, 	0,  27000,  720,	576, 	1201,	576,	0, 	2500},
	{HDMI_576i50,      	0,  27000,	1440,	288,  	288, 	288,	1, 	2500},
	{HDMI_576i50_16x9, 	0,  27000, 	1440,	288,  	288, 	288,	1, 	2500},
	{HDMI_576i50_16x9, 	0,  27000, 	1440,	145,  	145, 	145,	2, 	2500},
	{HDMI_720p60,      	0,  74250, 	1280,	720, 	1470, 	720,	0, 	3000},
	{HDMI_720p50,      	0,  74250, 	1280,	720, 	1470, 	720,	0, 	2500},
	{HDMI_1080i60,     	0,  74250, 	1920,	540, 	2228,	1103,	0, 	3000},
	{HDMI_1080i50,     	0,  74250, 	1920,	540, 	2228, 	1103,	0, 	2500},
	{HDMI_1080p60,     	0,	148500, 1920,	1080, 	1080, 	2160,	0, 	3000},
	{HDMI_1080p24,     	0,  74250,	1920,	1080, 	2205, 	2160,	0, 	1200},
	{HDMI_1080p25,     	0,  74250,	1920,	1080, 	2205, 	2160,	0, 	1250},
	{HDMI_1080p30,     	0,  74250,	1920,	1080, 	2205, 	2160,	0, 	1500},
	{HDMI_1080p50,     	0,	148500, 1920,	1080, 	1080,	2160,	0, 	2500},
/* extend format */
	{HDMI_1440x240p60,		0, 	27000, 1440, 240, 240, 240,	1, 3000},	//vic 8
	{HDMI_1440x240p60_16x9, 0, 	27000, 1440, 240, 240, 240,	1, 3000},	//vic 9
	{HDMI_2880x480i60,      0, 	54000, 2880, 240, 240, 240,	9, 3000},	//vic 10
	{HDMI_2880x480i60_16x9, 0, 	54000, 2880, 240, 240, 240,	9, 3000},	//vic 11
	{HDMI_2880x240p60,      0, 	54000, 2880, 240, 240, 240,	9, 3000},	//vic 12
	{HDMI_2880x240p60_16x9, 0, 	54000, 2880, 240, 240, 240,	9, 3000},	//vic 13
	{HDMI_1440x480p60,      0, 	54000, 1440, 480, 480, 480,	1, 3000},	//vic 14
	{HDMI_1440x480p60_16x9, 0, 	54000, 1440, 480, 480, 480,	1, 3000},	//vic 15

	{HDMI_1440x288p50,      0, 	27000, 1440, 288, 288, 288,	1, 2500},	//vic 23
	{HDMI_1440x288p50_16x9, 0, 	27000, 1440, 288, 288, 288,	1, 2500},	//vic 24
	{HDMI_2880x576i50,      0, 	54000, 2880, 288, 288, 288,	9, 2500},	//vic 25
	{HDMI_2880x576i50_16x9, 0, 	54000, 2880, 288, 288, 288,	9, 2500},	//vic 26
	{HDMI_2880x288p50,      0, 	54000, 2880, 288, 288, 288,	9, 2500},	//vic 27
	{HDMI_2880x288p50_16x9, 0, 	54000, 2880, 288, 288, 288,	9, 2500},	//vic 28
	{HDMI_1440x576p50,      0, 	54000, 1440, 576, 576, 576,	1, 2500},	//vic 29
	{HDMI_1440x576p50_16x9, 0, 	54000, 1440, 576, 576, 576,	1, 2500},	//vic 30

	{HDMI_2880x480p60,      0, 	108000, 2880, 480,  480, 480, 0, 3000},	//vic 35
	{HDMI_2880x480p60_16x9, 0, 	108000, 2880, 480,  480, 480, 0, 3000},	//vic 36
	{HDMI_2880x576p50,      0,	108000, 2880, 576,  576, 576, 0, 2500},	//vic 37
	{HDMI_2880x576p50_16x9, 0,	108000, 2880, 576,  576, 576, 0, 2500},	//vic 38
	{HDMI_1080i50_1250,     0,  72000, 	1920, 540,  540, 540, 0, 2500},	//vic 39
	{HDMI_720p24,           0,  74250, 	1280, 720, 1470, 720, 0, 1200},	//vic 60
	{HDMI_720p30,           0,  74250, 	1280, 720, 1470, 720, 0, 1500},	//vic 62

/* vesa format*/
	{HDMI_800_600,   1, 0,  800,  600,  600,  600, 0, 0},
	{HDMI_1024_768,  1, 0, 1024,  768,  768,  768, 0, 0},
	{HDMI_720_400,   1, 0,  720,  400,  400,  400, 0, 0},
	{HDMI_1280_768,  1, 0, 1280,  768,  768,  768, 0, 0},
	{HDMI_1280_800,  1, 0, 1280,  800,  800,  800, 0, 0},
	{HDMI_1280_960,  1, 0, 1280,  960,  960,  960, 0, 0},
	{HDMI_1280_1024, 1, 0, 1280, 1024, 1024, 1024, 0, 0},
	{HDMI_1360_768,  1, 0, 1360,  768,  768,  768, 0, 0},
	{HDMI_1366_768,  1, 0, 1366,  768,  768,  768, 0, 0},
	{HDMI_1600_1200, 1, 0, 1600, 1200, 1200, 1200, 0, 0},
	{HDMI_1920_1200, 1, 0, 1920, 1200, 1200, 1200, 0, 0},
	{HDMI_1440_900,  1, 0, 1440,  900,  900,  900, 0, 0},
	{HDMI_1400_1050, 1, 0, 1400, 1050, 1050, 1050, 0, 0},
	{HDMI_1680_1050, 1, 0, 1680, 1050, 1050, 1050, 0, 0},          //vic 79
    /* 4k2k mode */
    {HDMI_3840_2160p, 0, 0, 3840, 2160, 2160, 2160,	0, 0},//
    {HDMI_4096_2160p, 0, 0, 4096, 2160, 2160, 2160,	0, 0},//         /* 81 */
    //4k2k 420mode hactive = hactive/2
	{HDMI_3840_2160p, 0, 0, 1920, 2160, 2160, 2160,	0, 0},
	{HDMI_4096_2160p, 0, 0, 2048, 2160, 2160, 2160,	0, 0},
	{HDMI_1080p60,	0,	74250, 960,	1080, 	1080, 	1080,	0, 	3000},

	/* for AG-506 */
	{HDMI_480p60, 0, 27000, 720, 483, 483, 483 ,0, 3000},
	{0, 0, 0, 0, 0, 0, 0, 0, 0}
};


#if 0
unsigned int get_vic_from_timing(struct hdmi_rx_ctrl_video* video_par)
{
	int i;
	for(i = 0; freq_ref[i].vic; i++){
		if((abs((signed int)video_par->hactive - (signed int)freq_ref[i].active_pixels) <= diff_pixel_th) &&
		   ((abs((signed int)video_par->vactive - (signed int)freq_ref[i].active_lines) <= diff_line_th) ||
		    (abs((signed int)video_par->vactive - (signed int)freq_ref[i].active_lines_fp) <= diff_line_th) ||
            (abs((signed int)video_par->vactive - (signed int)freq_ref[i].active_lines_alternative) <= diff_line_th)
           )) {
           if((abs(video_par->refresh_rate - freq_ref[i].frame_rate) <= diff_frame_th) ||
				(freq_ref[i].frame_rate == 0)){
				break;
           }
		}
	}
	return freq_ref[i].vic;
}
#endif
unsigned int get_index_from_ref(struct hdmi_rx_ctrl_video* video_par)
{
	int i;
	for(i = 0; freq_ref[i].vic; i++){
		if((abs(video_par->hactive - freq_ref[i].active_pixels) <= diff_pixel_th) &&
		   ((abs(video_par->vactive - freq_ref[i].active_lines) <= diff_line_th) ||
			(abs(video_par->vactive - freq_ref[i].active_lines_fp) <= diff_line_th) ||
			(abs(video_par->vactive - freq_ref[i].active_lines_alternative) <= diff_line_th)
		   )) {
		   if((abs(video_par->refresh_rate - freq_ref[i].frame_rate) <= diff_frame_th) ||
			(freq_ref[i].frame_rate == 0)){
				if((HDMI_1360_768 == freq_ref[i].vic) || (HDMI_1366_768 == freq_ref[i].vic)){
					if(abs(video_par->hactive - freq_ref[i].active_pixels) <= 2)
						break;
				 }else
					break;
			}
		}
	}
	return i;
}


enum tvin_sig_fmt_e hdmirx_hw_get_fmt(void)
{
	/* to do:
	TVIN_SIG_FMT_HDMI_1280x720P_24Hz_FRAME_PACKING,
	TVIN_SIG_FMT_HDMI_1280x720P_30Hz_FRAME_PACKING,

	TVIN_SIG_FMT_HDMI_1920x1080P_24Hz_FRAME_PACKING,
	TVIN_SIG_FMT_HDMI_1920x1080P_30Hz_FRAME_PACKING, // 150
	*/
	enum tvin_sig_fmt_e fmt = TVIN_SIG_FMT_NULL;
	unsigned int vic = rx.pre_video_params.sw_vic;

	if(force_vic){
		vic = force_vic;
	}

	switch(vic){
		/* basic format */
		case HDMI_640x480p60:		    /*1*/
			fmt = TVIN_SIG_FMT_HDMI_640X480P_60HZ;
			break;
		case HDMI_480p60:                   /*2*/
		case HDMI_480p60_16x9:              /*3*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_720X480P_60HZ_FRAME_PACKING;
			}
			else{
				fmt = TVIN_SIG_FMT_HDMI_720X480P_60HZ;
			}
			break;
		case HDMI_720p60:                   /*4*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_60HZ_FRAME_PACKING;
			}
			else{
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_60HZ;
			}
			break;
		case HDMI_1080i60:                  /*5*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_1920X1080I_60HZ_FRAME_PACKING;
			} else if (is_alternative()) {
			    fmt = TVIN_SIG_FMT_HDMI_1920X1080I_60HZ_ALTERNATIVE;
			} else {
				fmt = TVIN_SIG_FMT_HDMI_1920X1080I_60HZ;
			}
			break;
		case HDMI_480i60:                   /*6*/
		case HDMI_480i60_16x9:              /*7*/
			fmt = TVIN_SIG_FMT_HDMI_1440X480I_60HZ;
			break;
		case HDMI_1080p60:		    /*16*/
		    fmt = TVIN_SIG_FMT_HDMI_1920X1080P_60HZ;
			if(is_alternative() && (rx.pre_video_params.video_format==3))
				fmt = TVIN_SIG_FMT_HDMI_3840_2160_00HZ;
			break;
		case HDMI_1080p24:		    /*32*/
			if(is_frame_packing()){
			    fmt = TVIN_SIG_FMT_HDMI_1920X1080P_24HZ_FRAME_PACKING;
			} else if (is_alternative()) {
			    fmt = TVIN_SIG_FMT_HDMI_1920X1080P_24HZ_ALTERNATIVE;
			} else {
			    fmt = TVIN_SIG_FMT_HDMI_1920X1080P_24HZ;
			}
			break;
		case HDMI_576p50:		    /*17*/
		case HDMI_576p50_16x9:		    /*18*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_720X576P_50HZ_FRAME_PACKING;
			}
			else{
				fmt = TVIN_SIG_FMT_HDMI_720X576P_50HZ;
			}
			break;
		case HDMI_720p50:		    /*19*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_50HZ_FRAME_PACKING;
			}
			else{
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_50HZ;
			}
			break;
		case HDMI_1080i50:		    /*20*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_1920X1080I_50HZ_FRAME_PACKING;
			} else if (is_alternative()) {
			    fmt = TVIN_SIG_FMT_HDMI_1920X1080I_50HZ_ALTERNATIVE;
			} else {
				fmt = TVIN_SIG_FMT_HDMI_1920X1080I_50HZ_A;
			}
			break;
		case HDMI_576i50:		   /*21*/
		case HDMI_576i50_16x9:		   /*22*/
			fmt = TVIN_SIG_FMT_HDMI_1440X576I_50HZ;
			break;
		case HDMI_1080p50:		   /*31*/
			fmt = TVIN_SIG_FMT_HDMI_1920X1080P_50HZ;
			if(is_alternative() && (rx.pre_video_params.video_format==3))
				fmt = TVIN_SIG_FMT_HDMI_3840_2160_00HZ;
			break;
		case HDMI_1080p25:		   /*33*/
			fmt = TVIN_SIG_FMT_HDMI_1920X1080P_25HZ;
			break;
		case HDMI_1080p30:		   /*34*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_1920X1080P_30HZ_FRAME_PACKING;
			} else if (is_alternative()) {
			    fmt = TVIN_SIG_FMT_HDMI_1920X1080P_30HZ_ALTERNATIVE;
			} else {
				fmt = TVIN_SIG_FMT_HDMI_1920X1080P_30HZ;
			}
			break;
		case HDMI_720p24:		   /*60*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_24HZ_FRAME_PACKING;
			}
			else{
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_24HZ;
			}
			break;
		case HDMI_720p30:		   /*62*/
			if(is_frame_packing()){
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_30HZ_FRAME_PACKING;
			}
			else{
				fmt = TVIN_SIG_FMT_HDMI_1280X720P_30HZ;
			}
			break;

		/* extend format */
		case HDMI_1440x240p60:
		case HDMI_1440x240p60_16x9:
			fmt = TVIN_SIG_FMT_HDMI_1440X240P_60HZ;
			break;
		case HDMI_2880x480i60:
		case HDMI_2880x480i60_16x9:
			fmt = TVIN_SIG_FMT_HDMI_2880X480I_60HZ;
			break;
		case HDMI_2880x240p60:
		case HDMI_2880x240p60_16x9:
			fmt = TVIN_SIG_FMT_HDMI_2880X240P_60HZ;
			break;
		case HDMI_1440x480p60:
		case HDMI_1440x480p60_16x9:
			fmt = TVIN_SIG_FMT_HDMI_1440X480P_60HZ;
			break;
		case HDMI_1440x288p50:
		case HDMI_1440x288p50_16x9:
			fmt = TVIN_SIG_FMT_HDMI_1440X288P_50HZ;
			break;
		case HDMI_2880x576i50:
		case HDMI_2880x576i50_16x9:
			fmt = TVIN_SIG_FMT_HDMI_2880X576I_50HZ;
			break;
		case HDMI_2880x288p50:
		case HDMI_2880x288p50_16x9:
			fmt = TVIN_SIG_FMT_HDMI_2880X288P_50HZ;
			break;
		case HDMI_1440x576p50:
		case HDMI_1440x576p50_16x9:
			fmt = TVIN_SIG_FMT_HDMI_1440X576P_50HZ;
			break;

		case HDMI_2880x480p60:
		case HDMI_2880x480p60_16x9:
			fmt = TVIN_SIG_FMT_HDMI_2880X480P_60HZ;
			break;
		case HDMI_2880x576p50:
		case HDMI_2880x576p50_16x9:
			fmt = TVIN_SIG_FMT_HDMI_2880X576P_60HZ; //????, should be TVIN_SIG_FMT_HDMI_2880x576P_50Hz
			break;
		case HDMI_1080i50_1250:
			fmt = TVIN_SIG_FMT_HDMI_1920X1080I_50HZ_B;
			break;
		case HDMI_1080I120:	/*46*/
			fmt = TVIN_SIG_FMT_HDMI_1920X1080I_120HZ;
			break;
		case HDMI_720p120:	/*47*/
			fmt = TVIN_SIG_FMT_HDMI_1280X720P_120HZ;
			break;
		case HDMI_1080p120:	/*63*/
			fmt = TVIN_SIG_FMT_HDMI_1920X1080P_120HZ;
			break;

		/* vesa format*/
		case HDMI_800_600:	/*65*/
			fmt = TVIN_SIG_FMT_HDMI_800X600_00HZ;
			break;
		case HDMI_1024_768:	/*66*/
			fmt = TVIN_SIG_FMT_HDMI_1024X768_00HZ;
			break;
		case HDMI_720_400:
			fmt = TVIN_SIG_FMT_HDMI_720X400_00HZ;
			break;
		case HDMI_1280_768:
			fmt = TVIN_SIG_FMT_HDMI_1280X768_00HZ;
			break;
		case HDMI_1280_800:
			fmt = TVIN_SIG_FMT_HDMI_1280X800_00HZ;
			break;
		case HDMI_1280_960:
			fmt = TVIN_SIG_FMT_HDMI_1280X960_00HZ;
			break;
		case HDMI_1280_1024:
			fmt = TVIN_SIG_FMT_HDMI_1280X1024_00HZ;
			break;
		case HDMI_1360_768:
			fmt = TVIN_SIG_FMT_HDMI_1360X768_00HZ;
			break;
		case HDMI_1366_768:
			fmt = TVIN_SIG_FMT_HDMI_1366X768_00HZ;
			break;
		case HDMI_1600_1200:
			fmt = TVIN_SIG_FMT_HDMI_1600X1200_00HZ;
			break;
		case HDMI_1920_1200:
			fmt = TVIN_SIG_FMT_HDMI_1920X1200_00HZ;
			break;
		case HDMI_1440_900:
			fmt = TVIN_SIG_FMT_HDMI_1440X900_00HZ;
			break;
		case HDMI_1400_1050:
			fmt = TVIN_SIG_FMT_HDMI_1400X1050_00HZ;
			break;
		case HDMI_1680_1050:
			fmt = TVIN_SIG_FMT_HDMI_1680X1050_00HZ;
			break;
            /* 4k2k mode */
		case HDMI_3840_2160p:
		    fmt = TVIN_SIG_FMT_HDMI_3840_2160_00HZ;
		    break;
		case HDMI_4096_2160p:
		    fmt = TVIN_SIG_FMT_HDMI_4096_2160_00HZ;
		    break;
		default:
			break;
		}

	return fmt;
}

bool hdmirx_hw_pll_lock(void)
{
	return (rx.state==HDMIRX_HWSTATE_SIG_READY);
}

bool hdmirx_hw_is_nosig(void)
{
	return rx.no_signal;
}



static bool is_packetinfo_change(struct hdmi_rx_ctrl_video *pre, struct hdmi_rx_ctrl_video *cur)
{
	//1. dvi
	//if (cur->dvi != cur->sw_dvi)
		//return true;
	//2. hdcp encrypted
	//if((cur->hdcp_enc_state != pre->hdcp_enc_state)){
		//printk("cur->hdcp_enc_state=%d,pre->hdcp_enc_state=%d\n",cur->hdcp_enc_state,pre->hdcp_enc_state);
		//return true;
	//}
	//3. colorspace change
	if(cur->video_format != pre->video_format){
		if(hdmirx_log_flag & VIDEO_LOG_ENABLE)
			printk("cur->video_format=%d,pre->video_format=%d\n",cur->video_format,pre->video_format);
		return true;
	}
	if(cur->interlaced != pre->interlaced) {
		if(hdmirx_log_flag & VIDEO_LOG_ENABLE)
			printk("cur->interlaced=%d,pre->interlaced=%d\n",cur->interlaced,pre->interlaced);
		return true;
	}
	return false;
}

static bool is_hdcp_state_error(struct hdmi_rx_ctrl_video *cur)
{
	//if ((cur->hdcp_enc_state==0) && (hdmirx_rd_dwc(0xE0) != 0)){
		//return true;
	//}
	if((rx.port == 0) && (hdmirx_rd_top(HDMIRX_TOP_EDID_GEN_STAT) >> 22))
		return true;
	if((rx.port == 1) && (hdmirx_rd_top(HDMIRX_TOP_EDID_GEN_STAT_B) >> 22))
		return true;
	if((rx.port == 2) && (hdmirx_rd_top(HDMIRX_TOP_EDID_GEN_STAT_C) >> 22))
		return true;

	return false;
}
/*
 * check timing info
 */
static bool is_timing_stable(struct hdmi_rx_ctrl_video *pre, struct hdmi_rx_ctrl_video *cur)
{
    bool ret = true;
#if 0
    //int pixel_clk = hdmirx_get_pixel_clock();

    //frame_rate = (cur->htotal==0||cur->vtotal==0)?0:
    //        (pixel_clk/cur->htotal)*100/cur->vtotal;
    if (    /*video_par->pixel_clk < PIXEL_CLK_MIN || video_par->pixel_clk > PIXEL_CLK_MAX*/
        (pixel_clk/1000) < PIXEL_CLK_MIN || (pixel_clk/1000) > PIXEL_CLK_MAX
        || cur->hactive < HRESOLUTION_MIN
        || cur->hactive > HRESOLUTION_MAX
        || cur->htotal < (cur->hactive + cur->hoffset)
        || cur->vactive < VRESOLUTION_MIN
        || cur->vactive > VRESOLUTION_MAX
        || cur->vtotal < (cur->vactive + cur->voffset)
        /*|| cur->refresh_rate < REFRESH_RATE_MIN
        || cur->refresh_rate > REFRESH_RATE_MAX
        */) {
            if(hdmirx_log_flag&0x200)
                printk("%s timing error pixel clk %d hactive %d htotal %d(%d) vactive %d vtotal %d(%d) %d\n", __func__,
                    pixel_clk/1000,
                    cur->hactive,  cur->htotal, cur->hactive+cur->hoffset,
                    cur->vactive,  cur->vtotal, cur->vactive+cur->voffset,
                    cur->refresh_rate);
            return false;
    }
#endif
    if ((abs((signed int)pre->hactive - (signed int)cur->hactive) > diff_pixel_th) ||
        (abs((signed int)pre->vactive - (signed int)cur->vactive) > diff_line_th)) {//||
        //(pre->pixel_repetition != cur->pixel_repetition)) {
        ret = false;

        if(hdmirx_log_flag&0x200){
            printk("[hdmirx] timing unstable: hactive(%d=>%d), vactive(%d=>%d), pixel_repeat(%d=>%d), video_format(%d=>%d)\n",
                    pre->hactive,        cur->hactive,
                    pre->vactive,        cur->vactive,
                    pre->pixel_repetition,   cur->pixel_repetition,
                    pre->video_format,   cur->video_format);
        }
    }
    return ret;
}
/*
 * check frame rate
 */
static bool is_frame_rate_change(struct hdmi_rx_ctrl_video *pre, struct hdmi_rx_ctrl_video *cur)
{
    bool ret = false;
    unsigned int pre_rate = (unsigned int)pre->refresh_rate * 2;
    unsigned int cur_rate = (unsigned int)cur->refresh_rate * 2;

    if ((abs((signed int)pre_rate - (signed int)cur_rate) > diff_frame_th)) {
        ret = true;

        if(hdmirx_log_flag&0x200){
            printk("[hdmirx] frame rate change: refresh_rate(%d=>%d), frame_rate:%d\n",
                    pre->refresh_rate,        cur->refresh_rate, cur_rate);
        }
    }
    return ret;
}

static int get_timing_fmt(struct hdmi_rx_ctrl_video *video_par)
{
	int i;
	int ret = 1;

	video_par->sw_vic = 0;
	video_par->sw_dvi = 0;
	video_par->sw_fp = 0;
	video_par->sw_alternative = 0;
	frame_rate = video_par->refresh_rate * 2;

	if((frame_rate > 9000) && use_frame_rate_check) {
        if(hdmirx_log_flag&0x200)
            printk("[hdmirx] frame_rate not support,sw_vic:%d,hw_vic:%d, frame_rate:%d \n",
                    video_par->sw_vic,video_par->video_mode,frame_rate);
        return ret;
	}
	// HDMI format fast detection
    for (i = 0; freq_ref[i].vic; i++) {
        if (freq_ref[i].vic == video_par->video_mode) {
            if ((abs(video_par->hactive - freq_ref[i].active_pixels) <= diff_pixel_th) &&
                ((abs(video_par->vactive - freq_ref[i].active_lines) <= diff_line_th) ||
                 (abs(video_par->vactive - freq_ref[i].active_lines_fp) <= diff_line_th) ||
                 (abs(video_par->vactive - freq_ref[i].active_lines_alternative) <= diff_line_th))) {
            break;
            }
        }
    }
    /* hdmi mode */
    if (freq_ref[i].vic != 0) {
        /*found standard hdmi mode */
        video_par->sw_vic = freq_ref[i].vic;
        if ((freq_ref[i].active_lines != freq_ref[i].active_lines_fp)
            && (abs(video_par->vactive - freq_ref[i].active_lines_fp) <= diff_line_th))
            video_par->sw_fp = 1;
        else if ((freq_ref[i].active_lines != freq_ref[i].active_lines_alternative)
            && (abs(video_par->vactive - freq_ref[i].active_lines_alternative) <= diff_line_th))
            video_par->sw_alternative = 1;
		/*********** repetition Check patch start ***********/
        if(repeat_check) {
			if(freq_ref[i].repetition_times != 9) {
				if(video_par->pixel_repetition != freq_ref[i].repetition_times){
					if(hdmirx_log_flag&PACKET_LOG_ENABLE)
						printk("\n repetition error1 %d : %d(standard)",video_par->pixel_repetition,freq_ref[i].repetition_times);
					video_par->pixel_repetition = freq_ref[i].repetition_times;
				}
			}
		}
		/************ repetition Check patch end ************/
        if(hdmirx_log_flag&0x200)
            printk("[hdmirx] standard hdmi mode,sw_vic:%d,hw_vic:%d, frame_rate:%d \n",
                    video_par->sw_vic,video_par->video_mode,frame_rate);
        return ret;
    }

    /* check the timing information */
    i = get_index_from_ref(video_par);
    video_par->sw_vic = freq_ref[i].vic;

    //if (video_par->video_mode != 0)
	if(video_par->sw_vic != 0)
	{
        /* non standard vic mode */
		video_par->sw_dvi = video_par->dvi;
        if ((freq_ref[i].active_lines != freq_ref[i].active_lines_fp)
            && (abs(video_par->vactive - freq_ref[i].active_lines_fp) <= diff_line_th))
            video_par->sw_fp = 1;
        else if ((freq_ref[i].active_lines != freq_ref[i].active_lines_alternative)
                && (abs(video_par->vactive - freq_ref[i].active_lines_alternative) <= diff_line_th))
            video_par->sw_alternative = 1;
		/*********** repetition Check patch start ***********/
        if(repeat_check) {
			if(freq_ref[i].repetition_times != 9) {
				if(video_par->pixel_repetition != freq_ref[i].repetition_times){
					if(hdmirx_log_flag&PACKET_LOG_ENABLE)
						printk("\n repetition error2 %d : %d(standard)",video_par->pixel_repetition,freq_ref[i].repetition_times);
					video_par->pixel_repetition = freq_ref[i].repetition_times;
				}
			}
		}
		/************ repetition Check patch end ************/
        if(hdmirx_log_flag&0x200)
            printk("[hdmirx] non standard hdmi mode,sw_vic:%d,hw_vic:%d, frame_rate:%d\n",
                    video_par->sw_vic,video_par->video_mode,frame_rate);
        return ret;
    }
	return ret;
}

/*
 * init audio information
 */
static void audio_status_init(void)
{
    audio_sample_rate = 0;
    audio_coding_type = 0;
    audio_channel_count = 0;
    auds_rcv_sts = 0;
}

static void Signal_status_init(void)
{
	hpd_wait_cnt = 0;
	sig_pll_unlock_cnt = 0;
	sig_pll_lock_cnt = 0;
    sig_unstable_cnt = 0;
	sig_stable_cnt = 0;
	sig_lost_lock_cnt = 0;
	sig_stable_cnt = 0;
    sig_unstable_cnt = 0;
    sig_unready_cnt = 0;
	sig_unstable_reset_hpd_cnt = 0;
	pre_tmds_clk_div4 = 0;
	rx.no_signal = true;
}

//----------------------------------------------------------
//func:		port A,B,C,D  hdmitx-5v monitor & HPD control
//note:		G9TV portD no used
//----------------------------------------------------------
void HPD_controller(void)
{
	uint32_t tmp_5v = 0;

	tmp_5v = hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V);

	if((tmp_5v>>20)&0x01){
		if(rx.portA_pow5v_state<pow5v_max_cnt)
			rx.portA_pow5v_state ++;
	}else{
		if(rx.portA_pow5v_state>0)
			rx.portA_pow5v_state --;
	}
	if((tmp_5v>>21)&0x01){
		if(rx.portB_pow5v_state<pow5v_max_cnt)
			rx.portB_pow5v_state ++;
	}else{
		if(rx.portB_pow5v_state>0)
			rx.portB_pow5v_state --;
	}
	if((tmp_5v>>22)&0x01){
		if(rx.portC_pow5v_state<pow5v_max_cnt)
			rx.portC_pow5v_state ++;
	}else{
		if(rx.portC_pow5v_state>0)
			rx.portC_pow5v_state --;
	}

	if(multi_port_edid_enable)
	{
		//------------port A-------------//
		if(rx.portA_pow5v_state == 0){
			if(rx.port == 0)
				rx.current_port_tx_5v_status = 0;
			if(rx.portA_pow5v_state_pre==1){
				rx.portA_pow5v_state_pre = 0;
				port0_need_run_EQ = true;
				if(current_port_hpd_ctl)
					hdmirx_set_hpd(0, 0);
			}
		}else if(rx.portA_pow5v_state == pow5v_max_cnt){
			if(rx.port == 0)
				rx.current_port_tx_5v_status = 1;
			if(rx.portA_pow5v_state_pre==0){
				rx.portA_pow5v_state_pre = 1;
				if(current_port_hpd_ctl)
					hdmirx_set_hpd(0, 1);
			}
		}
		//------------port B-------------//
		if(rx.portB_pow5v_state == 0){
			if(rx.port == 1)
				rx.current_port_tx_5v_status = 0;
			if(rx.portB_pow5v_state_pre==1){
				rx.portB_pow5v_state_pre = 0;
				port1_need_run_EQ = true;
				if(current_port_hpd_ctl)
					hdmirx_set_hpd(1, 0);
			}
		}else if(rx.portB_pow5v_state == pow5v_max_cnt){
			if(rx.port == 1)
				rx.current_port_tx_5v_status = 1;
			if(rx.portB_pow5v_state_pre==0){
				rx.portB_pow5v_state_pre = 1;
				if(current_port_hpd_ctl)
					hdmirx_set_hpd(1, 1);
			}
		}
		//-------------port C-------------//
		if(rx.portC_pow5v_state == 0){
			if(rx.port == 2)
				rx.current_port_tx_5v_status = 0;
			if(rx.portC_pow5v_state_pre==1){
				rx.portC_pow5v_state_pre = 0;
				port2_need_run_EQ = true;
				if(current_port_hpd_ctl)
					hdmirx_set_hpd(2, 0);
			}
		}else if(rx.portC_pow5v_state == pow5v_max_cnt){
			if(rx.port == 2)
				rx.current_port_tx_5v_status = 1;
			if(rx.portC_pow5v_state_pre==0){
				rx.portC_pow5v_state_pre = 1;
				if(current_port_hpd_ctl)
					hdmirx_set_hpd(2, 1);
			}
		}
		//------------------------------//
	}
	else
	{
		if(rx.port == 0){
			if(rx.portA_pow5v_state == pow5v_max_cnt)
				rx.current_port_tx_5v_status = 1;
			else
				rx.current_port_tx_5v_status = 0;
		}
		else if(rx.port == 1){
			if(rx.portB_pow5v_state == pow5v_max_cnt)
				rx.current_port_tx_5v_status = 1;
			else
				rx.current_port_tx_5v_status = 0;
		}
		else if(rx.port == 2){
			if(rx.portC_pow5v_state == pow5v_max_cnt)
				rx.current_port_tx_5v_status = 1;
			else
				rx.current_port_tx_5v_status = 0;
		}
	}
	if(force_hdmi_5v_high)
		rx.current_port_tx_5v_status = 1;
}
/*
static bool doublecheck_repetition(int size)
{
	int i;
    int buf[11] = {0};
	if (repeat_check == 0)
		return true;

    for(i=0; i<size; i++)
    {
        if(repeat_buf[i] > 10)
			continue;
    	if((++buf[repeat_buf[i]]) > size/2) {
			rx.pre_video_params.pixel_repetition = repeat_buf[i];
            return true;
    	}
    }
    return false;
}
*/
void hdmirx_error_count_config(void)
{}

bool hdmirx_tmds_pll_lock(void)
{
	if((hdmirx_rd_dwc (HDMIRX_DWC_HDMI_PLL_LCK_STS) & 0x01) == 0)
		return false;
	else
		return true;
}

bool hdmirx_audio_pll_lock(void)
{
	if (hw_dbg_en) return true;

	if((hdmirx_rd_dwc (HDMIRX_DWC_AUD_PLL_CTRL) & (1<<31)) == 0)
		return false;
	else
		return true;
}

void hdmirx_clock_rate_monitor(void)
{
	static int clk_rate_status[5] = {0};
	static int idx = 0;

	if(force_clk_rate&0x10){
		hdmirx_wr_phy(HDMIRX_PHY_CDR_CTRL_CNT,(hdmirx_rd_phy(HDMIRX_PHY_CDR_CTRL_CNT)&(~(1<<8)))|((force_clk_rate&1)<<8));
	}else{
		idx++;
		if(idx>=5)
			idx=0;
		clk_rate_status[idx] = (hdmirx_rd_dwc (HDMIRX_DWC_SCDC_REGS0) >> 17) & 1;
		if((clk_rate_status[0] == clk_rate_status[1]) &&
			(clk_rate_status[1] == clk_rate_status[2])&&
			(clk_rate_status[2] == clk_rate_status[3])&&
			(clk_rate_status[3] == clk_rate_status[4])){
			if(pre_tmds_clk_div4 != clk_rate_status[0]){
				pre_tmds_clk_div4 = clk_rate_status[0];
				if(pre_tmds_clk_div4)
					hdmirx_wr_phy(HDMIRX_PHY_CDR_CTRL_CNT,hdmirx_rd_phy(HDMIRX_PHY_CDR_CTRL_CNT)|(1<<8));
				else
					hdmirx_wr_phy(HDMIRX_PHY_CDR_CTRL_CNT,hdmirx_rd_phy(HDMIRX_PHY_CDR_CTRL_CNT)&(~(1<<8)));
			}
		}
	}
}

void hdmirx_sw_reset(int level)
{
	unsigned long   data32 = 0;
	if(level == 1){
		#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
		data32 |= 0 << 7;   // [7]		vid_enable
		#endif
		#ifdef CEC_FUNC_ENABLE
	    data32 |= 1 << 5;   // [5]      cec_enable
		#else
		data32 |= 0 << 5;   // [5]      cec_enable
		#endif
	    data32 |= 0 << 4;   // [4]      aud_enable
		data32 |= 0 << 3;   // [3]      bus_enable
		data32 |= 1 << 2;   // [2]      hdmi_enable
		data32 |= 0 << 1;   // [1]      modet_enable
		data32 |= 0 << 0;   // [0]      cfg_enable

	} else if(level == 2){
		#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
		data32 |= 0 << 7;   // [7]		vid_enable
		#endif
		#ifdef CEC_FUNC_ENABLE
	    data32 |= 1 << 5;   // [5]      cec_enable
		#else
		data32 |= 0 << 5;   // [5]      cec_enable
		#endif
	    data32 |= 1 << 4;   // [4]      aud_enable
		data32 |= 0 << 3;   // [3]      bus_enable
		data32 |= 1 << 2;   // [2]      hdmi_enable
		data32 |= 1 << 1;   // [1]      modet_enable
		data32 |= 0 << 0;   // [0]      cfg_enable

	}
	hdmirx_wr_dwc(HDMIRX_DWC_DMI_SW_RST, data32);
}
void hdmirx_DWC_reset(void)
{
	printk("hdmirx_DWC_reset\n");
	//audio_status_init();
	//Signal_status_init();
	//hdmirx_wr_top (HDMIRX_TOP_SW_RESET , 0x02);
	//hdmirx_wr_top (HDMIRX_TOP_SW_RESET , 0x00);
    hdmirx_packet_fifo_rst();
	hdmirx_audio_fifo_rst();
	hdmirx_sw_reset(2);
	//hdmirx_wr_dwc(HDMIRX_DWC_DMI_SW_RST, 0x10092);
}

void hdmirx_module_control(bool enable)
{
	unsigned long   data32;
	// Enable functional modules
	data32  =  hdmirx_rd_dwc(HDMIRX_DWC_DMI_DISABLE_IF);
	if(enable){
		//data32  |=  (_BIT(7));  //vid_enable
		data32  |=  (_BIT(4));  //audio_enable
		data32  |=  (_BIT(2));  //hdmi_enable
		data32  |=  (_BIT(1));  //modet_enable
	}else{
		//data32  &=  ~(_BIT(7));  //vid_enable
		data32  &=  ~(_BIT(4));  //audio_enable
		data32  &=  ~(_BIT(2));  //hdmi_enable
		data32  &=  ~(_BIT(1));  //modet_enable
	}
	hdmirx_wr_dwc(HDMIRX_DWC_DMI_DISABLE_IF, data32);    // DEFAULT: {31'd0, 1'b0}
	if(hdmirx_log_flag & REG_LOG_ENABLE)
		printk("hdmirx disable module %lx\n",data32);
}
// EQ calibration
#define MIN_SLOPE		50
#define ACC_MIN_LIMIT	0
#define ACC_LIMIT		370
//#define EQ_MAX_SETTING 12//for very long cable
#define EQ_SHORT_CABLE_BEST_SETTING	4
#define MINDIFF 	4
void hdmirx_phy_EQ_workaround_init(void)
{
	//ConfEqualSingle
	hdmirx_wr_phy(HDMIRX_PHY_MAIN_FSM_OVERRIDE2, 0x0);
	printk("hdmirx_phy_EQ_workaround_init\n");
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL1_CH0, 0x0211);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL1_CH1, 0x0211);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL1_CH2, 0x0211);

}
void hdmirx_phy_ConfEqualSetting(int ch0_lockVector,int ch1_lockVector,int ch2_lockVector)
{
	//ConfEqualSetting
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL4_CH0, 1<<ch0_lockVector);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL2_CH0, 0x0024);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL2_CH0, 0x0026);

	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL4_CH1, 1<<ch1_lockVector);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL2_CH1, 0x0024);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL2_CH1, 0x0026);

	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL4_CH2, 1<<ch2_lockVector);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL2_CH2, 0x0024);
	hdmirx_wr_phy(HDMIRX_PHY_EQCTRL2_CH2, 0x0026);

	//printk("lockvector ch0:%d\n", hdmirx_rd_phy(HDMIRX_PHY_EQCTRL4_CH0));

}

bool maxvsmin(int ch0Setting, int ch1Setting, int ch2Setting)
{
	int min = ch0Setting;
	int max = ch0Setting;
	if (ch1Setting > max) max = ch1Setting;
	if (ch2Setting > max) max = ch2Setting;
	if (ch1Setting < min) min = ch1Setting;
	if (ch2Setting < min) min = ch2Setting;

	if ((max - min) > MINDIFF) {
		printk("MINMAX ERROR\n");
		return 0;
	}
	return 1;
}

void configure_eq_setting(int rx_port_sel, int ch0Setting, int ch1Setting, int ch2Setting)
{
	unsigned int data32;
	printk("configure_eq_setting\n");
    // PDDQ = 1'b1; PHY_RESET = 1'b0;
    data32  = 0;
    data32 |= 1             << 6;   // [6]      physvsretmodez
    data32 |= 2             << 4;   // [5:4]    cfgclkfreq
    data32 |= rx_port_sel   << 2;   // [3:2]    portselect
    data32 |= 1             << 1;   // [1]      phypddq
    data32 |= 0             << 0;   // [0]      phyreset
    hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,    data32); // DEFAULT: {27'd0, 3'd0, 2'd1}


	hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, ch0Setting);
	hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, ch1Setting);
	hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, ch2Setting);
	hdmirx_wr_phy(HDMIRX_PHY_MAIN_FSM_OVERRIDE2, 0x40);

    // PDDQ = 1'b0; PHY_RESET = 1'b0;
    data32  = 0;
    data32 |= 1             << 6;   // [6]      physvsretmodez
    data32 |= 2             << 4;   // [5:4]    cfgclkfreq
    data32 |= rx_port_sel   << 2;   // [3:2]    portselect
    data32 |= 0             << 1;   // [1]      phypddq
    data32 |= 0             << 0;   // [0]      phyreset
    hdmirx_wr_dwc(HDMIRX_DWC_SNPS_PHYG3_CTRL,    data32); // DEFAULT: {27'd0, 3'd0, 2'd1}
}

#define EQ_DATA_INIT				0
#define EQ_SET_LOCK_VECTOR			1
#define EQ_SET_FORCE_FMS_STATE		2
#define EQ_CHECK_TMDS_VALID			3
#define EQ_AQUIRE_EARLY_COUNTER		4
#define EQ_GET_CABLE_TYPE			5
#define EQ_CONF_BEST_SETTING		6

#define EQ_WAITTIME 						1		//wait time between early/late counter acquisitions
#define EQ_SLOPEACM_MINTHRESHOLD			0		//Slope acumulator, minimum limit  to consider setting suitable for long cable
#define EQ_SLOPEACM_LONG_CABLE_THRESHOLD	360 	//Slope acumulator threshold to considere as long cable
#define EQ_MINSLOPE_VERYLONGCABLE			50		//minimum slope at maximum setting to consider it as a long cable

#define EQ_COUNTERTHRESHOLD 				512		//threshold for equalized system
#define EQ_MAX_SETTING 						13		//Maximum allowable setting
#define EQ_SHORT_CABLE_BEST_SETTING 		4		//Default best setting for short cables (electrical short length)
#define EQ_ERROR_CABLE_BEST_SETTING 		4		//Default setting when not good equalization is achieved, same as short cable
#define EQ_BOUNDARYSPREAD			 		20		//Stop averaging (Stable measures), if early/late counter acquisitions are within the following range 1oread +-20
#define EQ_MIN_ACQ_STABLE_DETECTION			3		//Minimum number of early/late counter acquisitions  to considere a stable acquisition
#define EQ_WAITTIME							1		//wait time between early/late counter acquisitions
#define EQ_SLOPEACM_MINTHRESHOLD 			0		//Slope acumulator, minimum limit  to consider setting suitable for long cable
#define	EQ_SLOPEACM_LONG_CABLE_THRESHOLD	360		//Slope acumulator threshold to considere as long cable
#define EQ_MINSLOPE_VERYLONGCABLE			50		//minimum slope at maximum setting to consider it as a long cable

void hdmirx_phy_EQ_workaround(void)
{
	static int nretry = 0;
	//static int eq_setting = 0;


	static int EQ_state = EQ_DATA_INIT;

	//Auxiliary variable to perform early-late counters averaging
	static int ch0_accumulator = 0;
	static int ch1_accumulator = 0;
	static int ch2_accumulator = 0;

	//Variables to store early-late counters averaging
	static int ch0_early_cnt = 0;
	static int ch1_early_cnt = 0;
	static int ch2_early_cnt = 0;

	//Variables to store early-late counters slope
	static int ch0_slope_accumulator = 0;
	static int ch1_slope_accumulator = 0;
	static int ch2_slope_accumulator = 0;

	//Auxiliary variable to detect early-late counters trend (up/down)
	static int last_ch0_early_cnt = 0;
	static int last_ch1_early_cnt = 0;
	static int last_ch2_early_cnt = 0;


	//EQ Best Long cable setting and valid flag
	static int ch0_eq_best_long_setting = 6;
	static int ch1_eq_best_long_setting = 6;
	static int ch2_eq_best_long_setting = 6;
	static int ch0_valid_long_setting = 0;
	static int ch1_valid_long_setting = 0;
	static int ch2_valid_long_setting = 0;

	//EQ Best Short cable setting and valid flag
	static int ch0_eq_best_short_setting = 4;
	static int ch1_eq_best_short_setting = 4;
	static int ch2_eq_best_short_setting = 4;
	static int ch0_valid_short_setting = 0;
	static int ch1_valid_short_setting = 0;
	static int ch2_valid_short_setting = 0;

	//EQ Final Setting should be programed to the PHY
	static int ch0_eq_best_setting = 4;
	static int ch1_eq_best_setting = 4;
	static int ch2_eq_best_setting = 4;

	//Auxiliary variables to detect stable acquisitions this will save 2mS * EQ_MAX_SETTING when good cables are used
	static int upperBound_acqCH0, upperBound_acqCH1, upperBound_acqCH2;
	static int lowerBound_acqCH0, lowerBound_acqCH1, lowerBound_acqCH2;
	static int outBound_acqCH0, outBound_acqCH1, outBound_acqCH2;

	//Auxiliary variables to control algorithm status
	static bool ch0_finish_flag = 0;
	static bool ch1_finish_flag = 0;
	static bool ch2_finish_flag = 0;
	static bool ch0_error_cable_flag = 0;
	static bool ch1_error_cable_flag = 0;
	static bool ch2_error_cable_flag = 0;

	//TMDSVALID FLAG
	static bool tmds_valid_flag = 0;

	//Auxiliary variable to control how many times Algorithm will retry if any error is detected
	static bool minmax_err_flag = 0;
	static int minmax_check_cnt = 0;

	//static int tmds_valid_cnt = 0;
	//static int tmds_valid_cnt_max = 10;
	int eq_MAINFSM_STATUS1 = 0;

	int nacq = 5;

	switch(EQ_state){
	case EQ_DATA_INIT:
		//***************************   IMPORTANT   ***************************
		//*************************** PLEASE README ***************************
		//The following piece of code was added here to make sure that before running the algorithm
		//all needed condition are available if not
		//The conditions are, PHY is receiving a stable clock  [reg (MAINFSM_STATUS1) 0x09[8]=1'b1]
		//and PLL_RATE if greater that 94.5MHz reg (MAINFSM_STATUS1) 0x09[10:9]=2'b0x]
		//If stable clock is not asserted MAIN FSM should wait for such signal and call again the algorithm.
		//int eq_MAINFSM_STATUS1 = 0;
		eq_MAINFSM_STATUS1 = hdmirx_rd_phy(REG_HDMI_PHY_MAINFSM_STATUS1); // GET PHY STATUS (MAINFSM_STATUS1)
		hdmirx_wr_phy(HDMIRX_PHY_MAIN_FSM_OVERRIDE2, 0x0); //Make sure that SW is not overriding the equalization not mandatory code (this is the default status)

		if ((eq_MAINFSM_STATUS1 >> 8 & 0x1) == 0) //clock_stable = false => Main FSM should wait for clock stable
		//Is mandatory to have a stable clock before starting to do equalization
		{
			rx.state = HDMIRX_HWSTATE_WAIT_CLK_STABLE; //You should add some extra state on MAIN FSM to wait clock_stable flag from the PHY
													  //otherwise this will be a never end status if no clock from the source
			rx.pre_state = HDMIRX_HWSTATE_EQ_CALIBRATION;//HDMIRX_HWSTATE_HPD_READY;
			break;
		}
		if((( eq_MAINFSM_STATUS1 >>10)& 0x1) != 0){
			//pll_rate smaller than 94.5MHz, algorithm not needed
			//Please make sure that PHY get the default status
			hdmirx_wr_phy(0x33, 0x0020);	//default status for register 0x33
			hdmirx_wr_phy(0x53, 0x0020); //default status for register 0x53
			hdmirx_wr_phy(0x73, 0x0020); //default status for register 0x73

			if(rx.port == 0) {
				printk("port0-000\n");
				port0_eq_setting_ch0 = 0;
				port0_eq_setting_ch1 = 0;
				port0_eq_setting_ch2 = 0;
				port0_need_run_EQ = true;
			} else if(rx.port == 1) {
				printk("port1-000\n");
				port1_eq_setting_ch0 = 0;
				port1_eq_setting_ch1 = 0;
				port1_eq_setting_ch2 = 0;
				port1_need_run_EQ = true;
			} else {
				printk("port2-000\n");
				port2_eq_setting_ch0 = 0;
				port2_eq_setting_ch1 = 0;
				port2_eq_setting_ch2 = 0;
				port2_need_run_EQ = true;
			}
			rx.state = HDMIRX_HWSTATE_SIG_UNSTABLE;
			rx.pre_state = HDMIRX_HWSTATE_HPD_READY;
			break;
		}
		//*************************** END  NOTICE ***************************


		ch0_finish_flag = 0;
		ch1_finish_flag = 0;
		ch2_finish_flag = 0;

		ch0_error_cable_flag = 0;
		ch1_error_cable_flag = 0;
		ch2_error_cable_flag = 0;

		ch0_valid_long_setting = 0;
		ch1_valid_long_setting = 0;
		ch2_valid_long_setting = 0;
		ch0_valid_short_setting = 0;
		ch1_valid_short_setting = 0;
		ch2_valid_short_setting = 0;
		ch0_accumulator = 0;
		ch1_accumulator = 0;
		ch2_accumulator = 0;
		ch0_slope_accumulator = 0;
		ch1_slope_accumulator = 0;
		ch2_slope_accumulator = 0;
		eq_setting = 0;

		EQ_state = EQ_SET_LOCK_VECTOR;

	break;
	case EQ_SET_LOCK_VECTOR:
		//tmds_valid_flag = 1;
		ch0_early_cnt = 0;
		ch1_early_cnt = 0;
		ch2_early_cnt = 0;
		outBound_acqCH0 = 0;
		outBound_acqCH1 = 0;
		outBound_acqCH2 = 0;
		nretry = 0;
		hdmirx_phy_ConfEqualSetting(eq_setting,eq_setting,eq_setting);
		EQ_state = EQ_SET_FORCE_FMS_STATE;
		//printk("set lock vector->set force FMS state\n");
	break;
	case EQ_SET_FORCE_FMS_STATE:
		hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1809);
		hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1819);
		hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1809);
		EQ_state = EQ_CHECK_TMDS_VALID;	//please make sure that there's at least 10 mS between that state and  TMDSVALID detection/test
		if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
			printk("set for FMS state->check TMDS valid\n");
	break;
	case EQ_CHECK_TMDS_VALID:
		if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
			printk("check TMDS valid\n"); 	//Please check the following
										//between the following printk instructions
										//printk("set for FMS state->check TMDS valid\n"); AND printk("check TMDS valid\n");	you have at lest 10ms

		if(!hdmirx_tmds_pll_lock()){					//Is this the tmds valid detection? ALG shouldn't wait for a long time (much more than 10mS) for TMDSVALID assertion
			if(tmds_valid_cnt++ > tmds_valid_cnt_max){ 	//how long are you waiting for TMDSVALID? should be ~~10mS
														//did you see any case where TMDSVALID doesn't get asserted after 10mS but get asserted after a long time?
														//System shouldn't wait for TMDS VALID assertion more than 10mS
														//because if TMDSVALID is not asserted means that setting is not good enough
														//and must proceed for the next one
														//please send to us some log file (early counters slope acumulator and setting ) where TMDS VALID just gets asserted after long time
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("tmds_valid_cnt=%d\n", tmds_valid_cnt);
				//printk("TMDS not valid\n");
				tmds_valid_flag = 0;
				tmds_valid_cnt = 0;
				EQ_state = EQ_GET_CABLE_TYPE;
			}
		}
		else{
			tmds_valid_cnt = 0;
			tmds_valid_flag = 1;
			ch0_early_cnt = hdmirx_rd_phy(HDMIRX_PHY_EQSTAT3_CH0);
			//printk("cho early cnt=[%d]\n", ch0_early_cnt);
			ch1_early_cnt = hdmirx_rd_phy(HDMIRX_PHY_EQSTAT3_CH1);
			ch2_early_cnt = hdmirx_rd_phy(HDMIRX_PHY_EQSTAT3_CH2);

			ch0_accumulator = ch0_early_cnt;
			ch1_accumulator = ch1_early_cnt;
			ch2_accumulator = ch2_early_cnt;
			upperBound_acqCH0 = ch0_accumulator + EQ_BOUNDARYSPREAD; lowerBound_acqCH0 =	ch0_accumulator - EQ_BOUNDARYSPREAD;
			upperBound_acqCH1 = ch1_accumulator + EQ_BOUNDARYSPREAD; lowerBound_acqCH1 =	ch1_accumulator - EQ_BOUNDARYSPREAD;
			upperBound_acqCH2 = ch2_accumulator + EQ_BOUNDARYSPREAD; lowerBound_acqCH2 =	ch1_accumulator - EQ_BOUNDARYSPREAD;
			EQ_state = EQ_AQUIRE_EARLY_COUNTER;
			//printk("check TMDS valid->aquire early counter\n");
		}
	break;
	case EQ_AQUIRE_EARLY_COUNTER:
		//***************************   IMPORTANT   ***************************
		//*************************** PLEASE README ***************************
		//the execution of the following loop before jumping to the MAIN FSM will save 500mS, taking into account that MAIN FSM takes 10mS to finish.
		while(1) //Important loop to save 500mS of execution time. Loop takes a maximum time of ~5mS
		{
			nretry ++;
			//printk("the current eq setting is[%d]\n", eq_setting);
			//hdmi_rx_phy_ConfEqualAutoCalib
			hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1809);
			hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1819);
			hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1809);
			mdelay(EQ_WAITTIME);	//wait EQ_WAITTIME to have a stable read from Early edge counter

			//Update boundaries to detect a stable acquisitions
			if (ch0_accumulator > upperBound_acqCH0 || ch0_accumulator < lowerBound_acqCH0)
				outBound_acqCH0++;
			if (ch1_accumulator > upperBound_acqCH1 || ch1_accumulator < lowerBound_acqCH1)
				outBound_acqCH1++;
			if (ch2_accumulator > upperBound_acqCH2 || ch2_accumulator < lowerBound_acqCH2)
				outBound_acqCH2++;
			if (nretry == EQ_MIN_ACQ_STABLE_DETECTION) //Finish averaging because Stable acquisitions were detected, minimum of three readings between boundaries to finish the averaging
			{
				if ( outBound_acqCH0 == 0 && outBound_acqCH1 == 0 && outBound_acqCH2== 0)
				{
					//printk("STABLE ACQ\n");
					nacq = EQ_MIN_ACQ_STABLE_DETECTION;
					//printk("3---1111early_cnt_ch0=[%d],nacq=[%d]\n", ch0_early_cnt, nacq);
					ch0_early_cnt = ch0_accumulator / nacq;
					if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
						printk("3early_cnt_ch0=[%d]\n", ch0_early_cnt);
					ch1_early_cnt = ch1_accumulator / nacq;
					if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
						printk("3early_cnt_ch1=[%d]\n", ch1_early_cnt);
					ch2_early_cnt = ch2_accumulator / nacq;
					if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
						printk("3early_cnt_ch2=[%d]\n", ch2_early_cnt);
					nretry = 0;
					EQ_state = EQ_GET_CABLE_TYPE;
					//printk("aquire early counter->get cable type\n");
					break;
				}
			}
			else if(nretry == nacq){	//Finish averaging,  maximum number of readings achieved
				nacq = nretry;
				//printk("5----1111early_cnt_ch0=[%d],nacq=[%d]\n", ch0_early_cnt, nacq);
				ch0_early_cnt = ch0_accumulator / nacq;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("5early_cnt_ch0=[%d]\n", ch0_early_cnt);
				ch1_early_cnt = ch1_accumulator / nacq;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("5early_cnt_ch1=[%d]\n", ch1_early_cnt);
				ch2_early_cnt = ch2_accumulator / nacq;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("5early_cnt_ch2=[%d]\n", ch2_early_cnt);
				nretry = 0;
				EQ_state = EQ_GET_CABLE_TYPE;
				//printk("aquire early counter->get cable type\n");
			break;
			}
			//Read next set of Early edges counter
			ch0_early_cnt = hdmirx_rd_phy(HDMIRX_PHY_EQSTAT3_CH0);
			ch1_early_cnt = hdmirx_rd_phy(HDMIRX_PHY_EQSTAT3_CH1);
			ch2_early_cnt = hdmirx_rd_phy(HDMIRX_PHY_EQSTAT3_CH2);

			//update the averaging accumulator
			ch0_accumulator += ch0_early_cnt;
			ch1_accumulator += ch1_early_cnt;
			ch2_accumulator += ch2_early_cnt;
		}
		//*************************** PLEASE README ***************************
		//if the following break gets removed, algorithm becomes 10mS*EQ_MAX_SETTING faster
		//break;

	case EQ_GET_CABLE_TYPE:
	{
		int ch0_stepSlope = 0;
		int ch1_stepSlope = 0;
		int ch2_stepSlope = 0;
		//printk("tmds_valid_flag=[%d]\n", tmds_valid_flag);


		//CABLE SETTING FOR THE BEST EQUALIZATION
		//LONG CABLE EQUALIZATION
		if(tmds_valid_flag == 1){
			//printk("enter cable type detection\n");
			if((ch0_early_cnt < last_ch0_early_cnt) && (ch0_finish_flag == 0)){
				ch0_slope_accumulator += (last_ch0_early_cnt - ch0_early_cnt);
				if((0 == ch0_valid_long_setting) && (ch0_early_cnt < EQ_COUNTERTHRESHOLD) && (ch0_slope_accumulator > EQ_SLOPEACM_MINTHRESHOLD)){
					ch0_eq_best_long_setting = eq_setting;
					ch0_valid_long_setting = 1;
				}
				//ch0_accumulator = ch0_accumulator + (last_ch0_early_cnt -	ch0_early_cnt);
				ch0_stepSlope = last_ch0_early_cnt-ch0_early_cnt;
			}
			if((ch1_early_cnt < last_ch1_early_cnt) && (ch1_finish_flag == 0)){
				ch1_slope_accumulator += (last_ch1_early_cnt - ch1_early_cnt);
				if((0 == ch1_valid_long_setting) &&	(ch1_early_cnt < EQ_COUNTERTHRESHOLD) && (ch1_slope_accumulator > EQ_SLOPEACM_MINTHRESHOLD)){
					ch1_eq_best_long_setting = eq_setting;
					ch1_valid_long_setting = 1;
				}
				//ch1_accumulator = ch1_accumulator + (last_ch1_early_cnt -	ch1_early_cnt);
				ch1_stepSlope = last_ch1_early_cnt-ch1_early_cnt;
			}
			if((ch2_early_cnt < last_ch2_early_cnt) && (ch2_finish_flag == 0)){
				ch2_slope_accumulator += (last_ch2_early_cnt - ch2_early_cnt);
				if((0 == ch2_valid_long_setting) && (ch2_early_cnt < EQ_COUNTERTHRESHOLD) &&(ch2_slope_accumulator > EQ_SLOPEACM_MINTHRESHOLD)){
					ch2_eq_best_long_setting = eq_setting;
					ch2_valid_long_setting = 1;
				}
				//ch2_accumulator = ch2_accumulator + (last_ch2_early_cnt -	ch2_early_cnt);
				ch2_stepSlope = last_ch2_early_cnt-ch2_early_cnt;
			}
		}
		//SHORT CABLE EQUALIZATION
		if (tmds_valid_flag == 1 &&  eq_setting <= EQ_SHORT_CABLE_BEST_SETTING) {
			if ( ch0_early_cnt < EQ_COUNTERTHRESHOLD && ch0_valid_short_setting == 0 ) {	//Short setting better than default
				ch0_eq_best_short_setting=eq_setting;
				ch0_valid_short_setting=1;
			}
			if (eq_setting == EQ_SHORT_CABLE_BEST_SETTING && ch0_valid_short_setting == 0){	//default Short setting is valid
				ch0_eq_best_short_setting=EQ_SHORT_CABLE_BEST_SETTING;
				ch0_valid_short_setting=1;
			}

			if ( ch1_early_cnt < EQ_COUNTERTHRESHOLD && ch1_valid_short_setting == 0) {	//Short setting better than default
				ch1_eq_best_short_setting=eq_setting;
				ch1_valid_short_setting=1;
			}
			if (eq_setting == EQ_SHORT_CABLE_BEST_SETTING && ch1_valid_short_setting == 0){	//Short setting is valid
				ch1_eq_best_short_setting=EQ_SHORT_CABLE_BEST_SETTING;
				ch1_valid_short_setting=1;
			}

			if ( ch2_early_cnt < EQ_COUNTERTHRESHOLD && ch2_valid_short_setting == 0) {	//Short setting better than default
				ch2_eq_best_short_setting=eq_setting;
				ch2_valid_short_setting=1;
			}
			if (eq_setting == EQ_SHORT_CABLE_BEST_SETTING && ch2_valid_short_setting == 0){	//Short setting is valid
				ch2_eq_best_short_setting=EQ_SHORT_CABLE_BEST_SETTING;
				ch2_valid_short_setting=1;
			}
		}

		//CABLE DETECTION
		//long cable
		if((1 == ch0_valid_long_setting)&& (ch0_slope_accumulator > EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch0_finish_flag == 0)){
			ch0_eq_best_setting = ch0_eq_best_long_setting;
			ch0_finish_flag = 1;
			if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
				printk("ch0:long cable");
		}
		if((1 == ch1_valid_long_setting)&& (ch1_slope_accumulator > EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch1_finish_flag == 0)){
			ch1_eq_best_setting = ch1_eq_best_long_setting;
			ch1_finish_flag = 1;
			if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
				printk("ch1:long cable");
		}
		if((1 == ch2_valid_long_setting)&& (ch2_slope_accumulator > EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch2_finish_flag == 0)){
			ch2_eq_best_setting = ch2_eq_best_long_setting;
			ch2_finish_flag = 1;
			if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
				printk("ch2:long cable");
		}

		if(eq_setting >= EQ_MAX_SETTING){	//Maximum setting achieved without long cable detection, so decision must be taken
			if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
				printk("eq reach the max settin---%d\n", eq_setting);
			//short cable
			if((ch0_slope_accumulator < EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch0_finish_flag == 0)){
				ch0_eq_best_setting = ch0_eq_best_short_setting;
				ch0_finish_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch0:short cable\n");
			}
			if((ch1_slope_accumulator < EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch1_finish_flag == 0)){
				ch1_eq_best_setting = ch1_eq_best_short_setting;
				ch1_finish_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch1:short cable\n");
			}
			if((ch2_slope_accumulator < EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch2_finish_flag == 0)){
				ch2_eq_best_setting = ch2_eq_best_short_setting;
				ch2_finish_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch2:short cable");
			}
			//very long cable
			if((ch0_slope_accumulator > EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch0_stepSlope > EQ_MINSLOPE_VERYLONGCABLE) && tmds_valid_flag && (ch0_finish_flag == 0)){
				ch0_eq_best_setting = EQ_MAX_SETTING;
				ch0_finish_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch0:very long cable\n");
			}
			if((ch1_slope_accumulator > EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch1_stepSlope > EQ_MINSLOPE_VERYLONGCABLE) && tmds_valid_flag && (ch1_finish_flag == 0)){
				ch1_eq_best_setting = EQ_MAX_SETTING;
				ch1_finish_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch1:very long cable\n");
			}
			if((ch2_slope_accumulator > EQ_SLOPEACM_LONG_CABLE_THRESHOLD) && (ch2_stepSlope > EQ_MINSLOPE_VERYLONGCABLE) && tmds_valid_flag && (ch2_finish_flag == 0)){
				ch2_eq_best_setting = EQ_MAX_SETTING;
				ch2_finish_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch2:very long cable\n");
			}
			//error cable
			if(ch0_finish_flag == 0){
				ch0_eq_best_setting = EQ_ERROR_CABLE_BEST_SETTING;
				ch0_finish_flag = 1;
				ch0_error_cable_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch0:error cable\n");
			}
			if(ch1_finish_flag == 0){
				ch1_eq_best_setting = EQ_ERROR_CABLE_BEST_SETTING;
				ch1_finish_flag = 1;
				ch1_error_cable_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch1:error cable\n");
			}
			if(ch2_finish_flag == 0){
				ch2_eq_best_setting = EQ_ERROR_CABLE_BEST_SETTING;
				ch2_finish_flag = 1;
				ch2_error_cable_flag = 1;
				if(hdmirx_log_flag&VIDEO_LOG_ENABLE)
					printk("ch2:error cable\n");
			}
		}

		if(ch0_finish_flag && ch1_finish_flag && ch2_finish_flag){	//Algorithm already take the decision for all Channels
			//printk("all chs finish the calibration\n");
			//printk("Final Settings CH0[%d],CH1[%d],CH2[%d]\n",ch0_eq_best_setting, ch1_eq_best_setting,ch2_eq_best_setting);
			EQ_state = EQ_CONF_BEST_SETTING;
		}
		else{	//Updating latest acquisition memory and jump to the next setting
			eq_setting++;
			printk("eq=[%d]\n", eq_setting);
			last_ch0_early_cnt = ch0_early_cnt;
			last_ch1_early_cnt = ch1_early_cnt;
			last_ch2_early_cnt = ch2_early_cnt;
			EQ_state = EQ_SET_LOCK_VECTOR;
		}

		//printk("acc0=%d\n", ch0_slope_accumulator);
		break;
	}
	case EQ_CONF_BEST_SETTING:
		//printk("best EQ ch0=%d, ch1=%d, ch2=%d",ch0_eq_best_setting,ch1_eq_best_setting,ch2_eq_best_setting);
		//hdmirx_wr_phy(HDMIRX_PHY_MAIN_FSM_OVERRIDE2, 0x0);//0x08->0 ,auto mode
		//hdmirx_phy_ConfEqualSetting(ch0_eq_best_setting,ch1_eq_best_setting,ch2_eq_best_setting);
		//hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1809);
		//hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1819);
		//hdmirx_wr_phy(HDMIRX_PHY_MAINFSM_CTL, 0x1809);
		//hdmirx_wr_phy(HDMIRX_PHY_CH0_EQ_CTRL3, ch0_eq_best_setting);
		//hdmirx_wr_phy(HDMIRX_PHY_CH1_EQ_CTRL3, ch1_eq_best_setting);
		//hdmirx_wr_phy(HDMIRX_PHY_CH2_EQ_CTRL3, ch2_eq_best_setting);
		//hdmirx_wr_phy(HDMIRX_PHY_MAIN_FSM_OVERRIDE2, 0x40);
		//eq_setting_ch0 = ch0_eq_best_setting;
		//eq_setting_ch1 = ch1_eq_best_setting;
		//eq_setting_ch2 = ch2_eq_best_setting;
		minmax_err_flag = 0;
		if((maxvsmin(ch0_eq_best_setting, ch1_eq_best_setting, ch2_eq_best_setting) == 0) || ch0_error_cable_flag==1 || ch1_error_cable_flag == 1 || ch2_error_cable_flag == 1 ){	//Min Max or error Cable
			ch0_eq_best_setting = EQ_ERROR_CABLE_BEST_SETTING;
			ch1_eq_best_setting = EQ_ERROR_CABLE_BEST_SETTING;
			ch2_eq_best_setting = EQ_ERROR_CABLE_BEST_SETTING;
			minmax_err_flag = 1;
			minmax_check_cnt++;
		}
		//sm_pause = 1;
		//hdmirx_wr_dwc(0x2c0, 0x57);
		//printk("0x3c0=[%x]\n", hdmirx_rd_dwc(0x2c0));
		if((minmax_err_flag == 0) || (minmax_check_cnt >= 2)){ //Algorithm finish
			rx.state = HDMIRX_HWSTATE_TIMINGCHANGE;
			rx.pre_state = HDMIRX_HWSTATE_EQ_CALIBRATION;
			tmds_valid_cnt = 0; //What this means?	Are you waiting for tmdsvalid?
			sm_pause = 0;
			minmax_check_cnt = 0;
			printk("eq cal->timing chage\n");
		}
		else{ //error detected and will retry
			EQ_state = EQ_DATA_INIT;
			break;
		}
		configure_eq_setting(rx.port, ch0_eq_best_setting, ch1_eq_best_setting, ch2_eq_best_setting); //please check readme for this particular function
		printk("best EQ ch0=%d, ch1=%d,ch2=%d",ch0_eq_best_setting,ch1_eq_best_setting,ch2_eq_best_setting);
		eq_setting_ch0 = ch0_eq_best_setting;
		eq_setting_ch1 = ch1_eq_best_setting;
		eq_setting_ch2 = ch2_eq_best_setting;
		if(rx.port == 0) {
			printk("port0\n");
			port0_eq_setting_ch0 = eq_setting_ch0;
			port0_eq_setting_ch1 = eq_setting_ch1;
			port0_eq_setting_ch2 = eq_setting_ch2;
			port0_need_run_EQ = false;
		} else if(rx.port == 1) {
			printk("port1\n");
			port1_eq_setting_ch0 = eq_setting_ch0;
			port1_eq_setting_ch1 = eq_setting_ch1;
			port1_eq_setting_ch2 = eq_setting_ch2;
			port1_need_run_EQ = false;
		} else {
			printk("port2\n");
			port2_eq_setting_ch0 = eq_setting_ch0;
			port2_eq_setting_ch1 = eq_setting_ch1;
			port2_eq_setting_ch2 = eq_setting_ch2;
			port2_need_run_EQ = false;
		}

		printk("quit!!!\n");
		EQ_state = EQ_DATA_INIT;
		break;
	}
}

bool need_run_EQ_workaround(void)
{
	if((rx.port == 0) && (true == port0_need_run_EQ))
		return true;
	if((rx.port == 1) && (true == port1_need_run_EQ))
		return true;
	if((rx.port == 2) && (true == port2_need_run_EQ))
		return true;
	return false;
}
void hdmirx_hw_monitor(void)
{
    int pre_sample_rate;

	if(sm_pause) //debug mode. pause state machine
		return;

	HPD_controller();//Hdmitx 5v detection
	switch(rx.state){
	case HDMIRX_HWSTATE_EQ_CALIBRATION:
		hdmirx_phy_EQ_workaround();
		break;
	case HDMIRX_HWSTATE_WAIT_CLK_STABLE:
		wait_clk_stable++;
		if(wait_clk_stable > 30){
			rx.state = HDMIRX_HWSTATE_TIMINGCHANGE;
			rx.pre_state = HDMIRX_HWSTATE_WAIT_CLK_STABLE;
			wait_clk_stable = 0;
			break;
		}
		rx.state = HDMIRX_HWSTATE_EQ_CALIBRATION;
		rx.pre_state = HDMIRX_HWSTATE_WAIT_CLK_STABLE;
		break;
	case HDMIRX_HWSTATE_INIT:
		if(!multi_port_edid_enable){
			hdmirx_set_hpd(rx.port, 0);
		}
			hdmirx_hw_config();
			hdmi_rx_ctrl_edid_update();

		audio_status_init();
	  	Signal_status_init();
		if(!multi_port_edid_enable){
			rx.state = HDMIRX_HWSTATE_HDMI5V_LOW;
			port0_need_run_EQ = true;
			port1_need_run_EQ = true;
			port2_need_run_EQ = true;
		}else{
			rx.state = HDMIRX_HWSTATE_HDMI5V_LOW;
		}
		rx.pre_state = HDMIRX_HWSTATE_INIT;
		//hdmirx_print("Hdmirx driver version: %s\n", HDMIRX_VER);
		break;
	case HDMIRX_HWSTATE_HDMI5V_LOW:
		if(rx.current_port_tx_5v_status){
			if(hpd_wait_cnt++ < hpd_wait_max)  //waiting hpd low 100ms at least
				break;
			hpd_wait_cnt = 0;
			rx.state = HDMIRX_HWSTATE_HDMI5V_HIGH;
			rx.pre_state = HDMIRX_HWSTATE_HDMI5V_LOW;
			hdmirx_print("\n[HDMIRX State] 5v low->5v high\n");
		}
		break;
	case HDMIRX_HWSTATE_HDMI5V_HIGH:
		if(!rx.current_port_tx_5v_status){
			rx.no_signal = true;
			rx.state = HDMIRX_HWSTATE_INIT;
			rx.pre_state = HDMIRX_HWSTATE_HDMI5V_HIGH;
			hdmirx_print("\n[HDMIRX State] 5v high->HPD_LOW\n");
		} else {
			//if((!multi_port_edid_enable) || (rx.pre_state == HDMIRX_HWSTATE_SIG_READY))
			hdmirx_set_hpd(rx.port, 1);
			rx.state = HDMIRX_HWSTATE_HPD_READY;
			rx.pre_state = HDMIRX_HWSTATE_HDMI5V_HIGH;
			//rx.no_signal = false;
			hdmirx_print("\n[HDMIRX State] 5v high->hpd ready\n");
		}
		break;
	case HDMIRX_HWSTATE_HPD_READY:
		if(!rx.current_port_tx_5v_status){
			rx.no_signal = true;
			rx.state = HDMIRX_HWSTATE_INIT;
			rx.pre_state = HDMIRX_HWSTATE_HPD_READY;
			//hdmirx_set_hpd(rx.port, 0);
			hdmirx_print("\n[HDMIRX State] hpd ready ->HPD_LOW\n");
		} else {
				if(hpd_wait_cnt++ <= hpd_wait_max)  //delay 300ms
					break;
				hpd_wait_cnt = 0;

			if(need_run_EQ_workaround()){
				hdmirx_phy_EQ_workaround_init();
				rx.state = HDMIRX_HWSTATE_EQ_CALIBRATION;
				rx.pre_state = HDMIRX_HWSTATE_HPD_READY;
			}else{
				rx.state = HDMIRX_HWSTATE_TIMINGCHANGE;
				rx.pre_state = HDMIRX_HWSTATE_HPD_READY;
			}
			hdmirx_print("\n[HDMIRX State] hpd ready->timing change\n");
		}
		break;
	case HDMIRX_HWSTATE_TIMINGCHANGE:
		if(rx.current_port_tx_5v_status == 0){
			rx.no_signal = true;
			hdmirx_print("[HDMIRX State] timingchange->init\n");
			rx.state = HDMIRX_HWSTATE_INIT;
			rx.pre_state = HDMIRX_HWSTATE_TIMINGCHANGE;
			break;
		}
		if(reset_sw){
			if(reset_phy_level == 2) {
				hdmirx_phy_init(rx.port, 0);
			} else if(reset_phy_level == 1) {
				hdmirx_phy_init_reset(rx.port, 0);
			} else if(reset_phy_level == 0) {
				//hdmirx_hw_reset();
				hdmirx_hw_config();
			}
		}
		pre_tmds_clk_div4 = 0;
		rx.state = HDMIRX_HWSTATE_SIG_UNSTABLE;
		rx.pre_state = HDMIRX_HWSTATE_TIMINGCHANGE;
		hdmirx_print("[HDMIRX State] TIMINGCHANGE -> unstable\n");
		break;
	case HDMIRX_HWSTATE_SIG_UNSTABLE:
		if(rx.current_port_tx_5v_status == 0){
			rx.no_signal = true;
			//hdmirx_print("[HDMIRX State] unstable->init\n");
			rx.state = HDMIRX_HWSTATE_INIT;
			rx.pre_state = HDMIRX_HWSTATE_SIG_UNSTABLE;
			break;
		}
		hdmirx_clock_rate_monitor();  //clk rate monitor, check scdc status
		//tmds valid flag ,dwc0x30 bit0
		if (hdmirx_tmds_pll_lock()){
			if(sig_pll_lock_cnt++ > sig_pll_lock_max){
				memset(&rx.vendor_specific_info, 0, sizeof(struct vendor_specific_info_s));
				//hdmirx_get_video_info(&rx.ctrl, &rx.cur_video_params);

				rx.state = HDMIRX_HWSTATE_DWC_RST_WAIT;
				rx.pre_state = HDMIRX_HWSTATE_SIG_UNSTABLE;
				if(reset_sw){
					hdmirx_module_control(1);
					hdmirx_DWC_reset();
				}
				sig_pll_unlock_cnt = 0;
				sig_pll_lock_cnt = 0;
				rx.no_signal = false;
				hdmirx_print("[HDMIRX State] unstable->dwc rst\n");
			}
		}else{
			if((sig_pll_lock_cnt) && (hdmirx_log_flag & VIDEO_LOG_ENABLE))
				hdmirx_print("[HDMIRX State] unstable: sig_pll_lock_cnt = %d\n",sig_pll_lock_cnt);

			sig_pll_lock_cnt = 0;
			sig_pll_unlock_cnt++;
			if(sig_pll_unlock_cnt == sig_pll_unlock_max) {
				rx.no_signal = true;
			}
			if(sig_pll_unlock_cnt == sig_pll_unlock_max*5){
				hdmirx_error_count_config();	//recount EQ & resist calibration
				rx.state = HDMIRX_HWSTATE_TIMINGCHANGE;
				rx.pre_state = HDMIRX_HWSTATE_SIG_UNSTABLE;
				hdmirx_print("[HDMIRX State] unstable->timingchange\n");
				pre_tmds_clk_div4 = 0;
				sig_pll_unlock_cnt = 0;
			}
		}
		break;
	case HDMIRX_HWSTATE_DWC_RST_WAIT:
		if(dwc_rst_wait_cnt++ < dwc_rst_wait_cnt_max)
			break;
		dwc_rst_wait_cnt = 0;
		rx.state = HDMIRX_HWSTATE_SIG_STABLE;
		rx.pre_state = HDMIRX_HWSTATE_DWC_RST_WAIT;
		hdmirx_print("[HDMIRX State] dwc reset->stable\n");
		break;
	case HDMIRX_HWSTATE_SIG_STABLE:
		if(rx.current_port_tx_5v_status == 0){
			rx.no_signal = true;
			hdmirx_print("[HDMIRX State] stable->HPD_LOW\n");
			rx.state = HDMIRX_HWSTATE_INIT;
			rx.pre_state = HDMIRX_HWSTATE_SIG_STABLE;
			break;
		}
		memcpy(&rx.pre_video_params, &rx.cur_video_params, sizeof(struct hdmi_rx_ctrl_video));
		hdmirx_get_video_info(&rx.ctrl, &rx.cur_video_params);
		if(is_timing_stable(&rx.pre_video_params, &rx.cur_video_params) || (force_ready)) {
			if (sig_stable_cnt++ > sig_stable_max) {
				get_timing_fmt(&rx.pre_video_params);
				if((rx.pre_video_params.sw_vic == HDMI_MAX_IS_UNSUPPORT)||
					(rx.pre_video_params.sw_vic == HDMI_Unkown)){
					if(hdmirx_log_flag & VIDEO_LOG_ENABLE)
						hdmirx_print("[HDMIRX State] stable->unkown vic\n");
					if(sig_stable_cnt < (sig_stable_max<<3))
						break;
					rx.state = HDMIRX_HWSTATE_TIMINGCHANGE;
					rx.pre_state = HDMIRX_HWSTATE_SIG_STABLE;
					break;
				}
				if(rx.pre_video_params.sw_dvi == 1){
					if(sig_stable_cnt < (sig_stable_max*7))
						break;
				}
				sig_stable_cnt = 0;
				sig_unstable_cnt = 0;
				rx.change = 0;
				//if(reset_sw)
					//hdmirx_sw_reset(2);
				//hdmirx_module_control(1);
				sig_lost_lock_max = 50;
				sig_unready_max = 20;
				//hdmirx_wr_dwc(HDMIRX_DWC_HDMI_MODE_RECOVER, 0x30010000);
				rx.state = HDMIRX_HWSTATE_SIG_READY;
				rx.pre_state = HDMIRX_HWSTATE_SIG_STABLE;
				rx.no_signal = false;
				//sw_reset_edid = 1 ;

				//memcpy(&rx.video_params, &rx.pre_video_params,sizeof(struct hdmi_rx_ctrl_video));
				memset(&rx.aud_info, 0, sizeof(struct aud_info_s));
				hdmirx_config_video(&rx.pre_video_params);
				hdmirx_print("[HDMIRX State] stable->ready\n");
				if(hdmirx_log_flag & VIDEO_LOG_ENABLE)
					dump_state(0x1);
			}
		}else{
			sig_stable_cnt = 0;
			if(sig_unstable_cnt++ >sig_unstable_max) {
				rx.state = HDMIRX_HWSTATE_TIMINGCHANGE;
				rx.pre_state = HDMIRX_HWSTATE_SIG_STABLE;
				sig_stable_cnt = 0;
				sig_unstable_cnt = 0;
				hdmirx_error_count_config();	//recount EQ & resist calibration
				if(enable_hpd_reset){
					sig_unstable_reset_hpd_cnt++;
					if(sig_unstable_reset_hpd_cnt >= sig_unstable_reset_hpd_max){
						rx.state = HDMIRX_HWSTATE_HDMI5V_LOW;
						hdmirx_set_hpd(rx.port, 0);
						sig_unstable_reset_hpd_cnt = 0;
						hdmirx_print("[HDMIRX State] unstable->hpd_low\n");
						break;
					}
				}
				hdmirx_print("[HDMIRX State] stable->timingchange\n");
			}
		}
		break;
	case HDMIRX_HWSTATE_SIG_READY:
		if(rx.current_port_tx_5v_status == 0){
			rx.no_signal = true;
			hdmirx_print("[HDMIRX State] ready->HPD_LOW\n");
			hdmirx_audio_enable(0);
			hdmirx_audio_fifo_rst();
			#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
			hdmirx_audiopll_control(0);
			//if(reset_sw){
				//hdmirx_module_control(0);
			//}
			#endif
			rx.state = HDMIRX_HWSTATE_INIT;
			rx.pre_state = HDMIRX_HWSTATE_SIG_READY;
			break;
		}
		if(hdmirx_tmds_pll_lock() == false) {
		    if (sig_lost_lock_cnt++ >= sig_lost_lock_max) {
		        rx.state = HDMIRX_HWSTATE_HPD_READY;
				rx.pre_state = HDMIRX_HWSTATE_SIG_READY;
				audio_sample_rate = 0;
				#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
				if(reset_sw){
					hdmirx_module_control(0);
				}
				hdmirx_audiopll_control(0);
				#endif
				hdmirx_audio_enable(0);
				hdmirx_audio_fifo_rst();
				if(hdmirx_log_flag & VIDEO_LOG_ENABLE){
		        	hdmirx_print("[hdmi] pll unlock !! TMDS clock = %d, Pixel clock = %d\n", hdmirx_get_tmds_clock(), hdmirx_get_pixel_clock());
		        	hdmirx_print("[HDMIRX State] pll unlock ready->HPD-ready\n");
				}
				break;
		    }
		} else {
		    sig_lost_lock_cnt = 0;
		}

	    hdmirx_get_video_info(&rx.ctrl, &rx.cur_video_params);
		rgb_quant_range = rx.cur_video_params.rgb_quant_range;
		yuv_quant_range = rx.cur_video_params.yuv_quant_range;
		it_content = rx.cur_video_params.it_content;
	    /* video info change */
	    if ((!is_timing_stable(&rx.pre_video_params, &rx.cur_video_params))){
			//is_vic_change() ||
	        //is_frame_rate_change(&rx.pre_video_params, &rx.cur_video_params)) {
	        //memcpy(&rx.video_params, &rx.pre_video_params,sizeof(struct hdmi_rx_ctrl_video));
	        if (sig_unready_cnt++ > sig_unready_max) {
	            sig_lost_lock_cnt = 0;
	            sig_unready_cnt = 0;
				audio_sample_rate = 0;
				#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
				if(reset_sw){
					hdmirx_module_control(0);
				}
				hdmirx_audiopll_control(0);
				#endif
				hdmirx_audio_enable(0);
				hdmirx_audio_fifo_rst();
	            rx.state = HDMIRX_HWSTATE_HPD_READY;
				rx.pre_state = HDMIRX_HWSTATE_SIG_READY;
				memcpy(&rx.pre_video_params, &rx.cur_video_params,sizeof(struct hdmi_rx_ctrl_video));
	            memset(&rx.vendor_specific_info, 0, sizeof(struct vendor_specific_info_s));
	            printk("[HDMIRX State] ready->Hpd ready:unstable\n");
	            break;
	        }
	    } else if(is_packetinfo_change(&rx.pre_video_params, &rx.cur_video_params) ||
	    	is_frame_rate_change(&rx.pre_video_params, &rx.cur_video_params)) {
	        //memcpy(&rx.video_params, &rx.pre_video_params,sizeof(struct hdmi_rx_ctrl_video));
	        if (sig_unready_cnt++ > sig_unready_max<<3) {
	            sig_lost_lock_cnt = 0;
	            sig_unready_cnt = 0;
				audio_sample_rate = 0;
				#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
				if(reset_sw){
					hdmirx_module_control(0);
				}
				hdmirx_audiopll_control(0);
				#endif
				hdmirx_audio_enable(0);
				hdmirx_audio_fifo_rst();
	            rx.state = HDMIRX_HWSTATE_HPD_READY;
				rx.pre_state = HDMIRX_HWSTATE_SIG_READY;
				memcpy(&rx.pre_video_params, &rx.cur_video_params,sizeof(struct hdmi_rx_ctrl_video));
	            memset(&rx.vendor_specific_info, 0, sizeof(struct vendor_specific_info_s));
	            hdmirx_print("[HDMIRX State] hpd-->ready:colorspace\n");
	            break;
		        }
	    }  else if(is_hdcp_state_error(&rx.cur_video_params)) {
			if (sig_unready_cnt++ > sig_unready_max*100) {
	            sig_lost_lock_cnt = 0;
	            sig_unready_cnt = 0;
				audio_sample_rate = 0;
				hdmirx_wr_top (HDMIRX_TOP_SW_RESET , 0x02);
				hdmirx_wr_top (HDMIRX_TOP_SW_RESET , 0x00);
				hdmirx_audiopll_control(0);
				hdmirx_audio_enable(0);
				hdmirx_audio_fifo_rst();
				hdmirx_set_hpd(rx.port, 0);
				hdmirx_hw_config();
	            rx.state = HDMIRX_HWSTATE_HDMI5V_LOW;
				rx.pre_state = HDMIRX_HWSTATE_SIG_READY;
				memcpy(&rx.pre_video_params, &rx.cur_video_params,sizeof(struct hdmi_rx_ctrl_video));
	            memset(&rx.vendor_specific_info, 0, sizeof(struct vendor_specific_info_s));
	            hdmirx_print("[HDMIRX State] hpd-->ready:ddc error\n");
	            break;
	        }
	    }
			else {
		        sig_unready_cnt = 0;
				if(enable_hpd_reset)
					sig_unstable_reset_hpd_cnt = 0;
		    }

			if ((audio_enable)&&(rx.pre_video_params.sw_dvi==0))
			{
			    pre_sample_rate = rx.aud_info.real_sample_rate;
				hdmirx_read_audio_info(&rx.aud_info);
				if (force_audio_sample_rate == 0)
				    rx.aud_info.real_sample_rate = get_real_sample_rate();
				else
				    rx.aud_info.real_sample_rate = force_audio_sample_rate;
				if((rx.aud_info.real_sample_rate<=31000)&&(rx.aud_info.real_sample_rate>=193000)
				   && (abs((signed int)rx.aud_info.real_sample_rate-(signed int)pre_sample_rate)>sample_rate_change_th)
                   ) {
				    if(hdmirx_log_flag&AUDIO_LOG_ENABLE) {
				        dump_audio_info(1);
				    }
				}

				if(!is_sample_rate_stable(pre_sample_rate, rx.aud_info.real_sample_rate)){
					//set_hdmi_audio_source_gate(0);
				    if(hdmirx_log_flag&AUDIO_LOG_ENABLE) {
			        hdmirx_print("[hdmirx-audio]:sample_rate_chg,pre:%d,cur:%d\n", pre_sample_rate, rx.aud_info.real_sample_rate);
			        //dump_audio_info(1);
				}
				rx.audio_sample_rate_stable_count = 0;
			} else {
					if(rx.audio_sample_rate_stable_count < audio_sample_rate_stable_count_th){
					    if(hdmirx_log_flag&AUDIO_LOG_ENABLE) {
					        hdmirx_print("[hdmirx-audio]:sample_rate_stable_count:%d\n", rx.audio_sample_rate_stable_count);
					    }
						rx.audio_sample_rate_stable_count++;
						if(rx.audio_sample_rate_stable_count==audio_sample_rate_stable_count_th){
						sig_lost_lock_max = 3;
						sig_unready_max = 6;
						//dump_state(0x2);
							printk("[hdmirx-audio]:----audio stable\n");
							#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
							hdmirx_audiopll_control(1);
							#endif
							//hdmirx_config_audio();
							hdmirx_audio_enable(1);
							hdmirx_audio_fifo_rst();

							audio_sample_rate = rx.aud_info.real_sample_rate;
							audio_coding_type = rx.aud_info.coding_type;
							audio_channel_count = rx.aud_info.channel_count;
							//rx.audio_wait_time = audio_stable_time;
							#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
							if(hdmirx_get_audio_clock()<100000){
								printk("update audio\n");
								hdmirx_wr_top(HDMIRX_TOP_ACR_CNTL_STAT,hdmirx_rd_top(HDMIRX_TOP_ACR_CNTL_STAT)|(1<<11));
							}
							#endif
						}
					}else{

					}
				}
				auds_rcv_sts = rx.aud_info.audio_samples_packet_received;
		}
		break;
	default:
		break;
	}

	if(force_state&0x10){
		rx.state = force_state&0xf;
	    if((force_state&0x20)==0){
	        force_state = 0;
	     }
    }
}

/*
* EDID & hdcp
*/
struct hdmi_rx_ctrl_hdcp init_hdcp_data;
#define MAX_KEY_BUF_SIZE 512

static char key_buf[MAX_KEY_BUF_SIZE];
static int key_size = 0;

#define MAX_EDID_BUF_SIZE 1024
static char edid_buf[MAX_EDID_BUF_SIZE];
static int edid_size = 0;
extern unsigned char *pEdid_buffer;

#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9TV)
static unsigned char amlogic_hdmirx_edid_port1[] =
{
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0xAC, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00,
0x0A, 0x18, 0x01, 0x03, 0x80, 0x73, 0x41, 0x78, 0x0A, 0xCF, 0x74, 0xA3, 0x57, 0x4C, 0xB0, 0x23,
0x09, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0xFF, 0x01, 0xFF, 0xFF, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x20, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
0x58, 0x2C, 0x45, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x41,
0x4D, 0x4C, 0x20, 0x38, 0x36, 0x36, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x46, 0x1F, 0x8C, 0x3C, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x27,
0x02, 0x03, 0x37, 0xF0, 0x5B, 0x5F, 0x1F, 0x10, 0x14, 0x05, 0x13, 0x04, 0x20, 0x22, 0x3C, 0x3E,
0x12, 0x16, 0x03, 0x07, 0x11, 0x15, 0x06, 0x02, 0x01, 0x5D, 0x60, 0x61, 0x64, 0x65, 0x66, 0x62,
0x23, 0x09, 0x07, 0x01, 0x83, 0x01, 0x00, 0x00, 0x6E, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x88, 0x3C,
0x20, 0x80, 0x80, 0x01, 0x02, 0x03, 0x04, 0x02, 0x3A, 0x80, 0xD0, 0x72, 0x38, 0x2D, 0x40, 0x10,
0x2C, 0x45, 0x80, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E,
0x20, 0xB8, 0x28, 0x55, 0x40, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x01, 0x1D, 0x80, 0xD0, 0x72,
0x1C, 0x16, 0x20, 0x10, 0x2C, 0x25, 0x80, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x9F, 0x8C, 0x0A, 0xD0,
0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x47
};

static unsigned char amlogic_hdmirx_edid_port2[] =
{
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0xAC, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00,
0x0A, 0x18, 0x01, 0x03, 0x80, 0x73, 0x41, 0x78, 0x0A, 0xCF, 0x74, 0xA3, 0x57, 0x4C, 0xB0, 0x23,
0x09, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0xFF, 0x01, 0xFF, 0xFF, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x20, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
0x58, 0x2C, 0x45, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x41,
0x4D, 0x4C, 0x20, 0x38, 0x36, 0x36, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x46, 0x1F, 0x8C, 0x3C, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x27,
0x02, 0x03, 0x37, 0xF0, 0x5B, 0x5F, 0x1F, 0x10, 0x14, 0x05, 0x13, 0x04, 0x20, 0x22, 0x3C, 0x3E,
0x12, 0x16, 0x03, 0x07, 0x11, 0x15, 0x06, 0x02, 0x01, 0x5D, 0x60, 0x61, 0x64, 0x65, 0x66, 0x62,
0x23, 0x09, 0x07, 0x01, 0x83, 0x01, 0x00, 0x00, 0x6E, 0x03, 0x0C, 0x00, 0x20, 0x00, 0x88, 0x3C,
0x20, 0x80, 0x80, 0x01, 0x02, 0x03, 0x04, 0x02, 0x3A, 0x80, 0xD0, 0x72, 0x38, 0x2D, 0x40, 0x10,
0x2C, 0x45, 0x80, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E,
0x20, 0xB8, 0x28, 0x55, 0x40, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x01, 0x1D, 0x80, 0xD0, 0x72,
0x1C, 0x16, 0x20, 0x10, 0x2C, 0x25, 0x80, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x9F, 0x8C, 0x0A, 0xD0,
0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x37
};

static unsigned char amlogic_hdmirx_edid_port3[] =
{
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0xAC, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00,
0x0A, 0x18, 0x01, 0x03, 0x80, 0x73, 0x41, 0x78, 0x0A, 0xCF, 0x74, 0xA3, 0x57, 0x4C, 0xB0, 0x23,
0x09, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0xFF, 0x01, 0xFF, 0xFF, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x20, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
0x58, 0x2C, 0x45, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x41,
0x4D, 0x4C, 0x20, 0x38, 0x36, 0x36, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x46, 0x1F, 0x8C, 0x3C, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x27,
0x02, 0x03, 0x37, 0xF0, 0x5B, 0x5F, 0x1F, 0x10, 0x14, 0x05, 0x13, 0x04, 0x20, 0x22, 0x3C, 0x3E,
0x12, 0x16, 0x03, 0x07, 0x11, 0x15, 0x06, 0x02, 0x01, 0x5D, 0x60, 0x61, 0x64, 0x65, 0x66, 0x62,
0x23, 0x09, 0x07, 0x01, 0x83, 0x01, 0x00, 0x00, 0x6E, 0x03, 0x0C, 0x00, 0x30, 0x00, 0x88, 0x3C,
0x20, 0x80, 0x80, 0x01, 0x02, 0x03, 0x04, 0x02, 0x3A, 0x80, 0xD0, 0x72, 0x38, 0x2D, 0x40, 0x10,
0x2C, 0x45, 0x80, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E,
0x20, 0xB8, 0x28, 0x55, 0x40, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x01, 0x1D, 0x80, 0xD0, 0x72,
0x1C, 0x16, 0x20, 0x10, 0x2C, 0x25, 0x80, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x9F, 0x8C, 0x0A, 0xD0,
0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x27
};
#else
static unsigned char amlogic_hdmirx_edid_port1[] =
{
0x00 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0x00 ,0x05 ,0xAC ,0x30 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,
0x0A ,0x18 ,0x01 ,0x03 ,0x80 ,0x73 ,0x41 ,0x78 ,0x0A ,0xCF ,0x74 ,0xA3 ,0x57 ,0x4C ,0xB0 ,0x23 ,
0x09 ,0x48 ,0x4C ,0x00 ,0x00 ,0x00 ,0x01 ,0x01 ,0x01 ,0xFF ,0x01 ,0xFF ,0xFF ,0x01 ,0x01 ,0x01 ,
0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x20 ,0x02 ,0x3A ,0x80 ,0x18 ,0x71 ,0x38 ,0x2D ,0x40 ,0x58 ,0x2C ,
0x45 ,0x00 ,0x20 ,0x52 ,0x31 ,0x00 ,0x00 ,0x1E ,0x02 ,0x3A ,0x80 ,0x18 ,0x71 ,0x38 ,0x2D ,0x40 ,
0x58 ,0x2C ,0x45 ,0x00 ,0x20 ,0xC2 ,0x31 ,0x00 ,0x00 ,0x1E ,0x00 ,0x00 ,0x00 ,0xFC ,0x00 ,0x41 ,
0x4D ,0x4C ,0x20 ,0x38 ,0x36 ,0x36 ,0x0A ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x00 ,0x00 ,0x00 ,0xFD ,
0x00 ,0x3B ,0x46 ,0x1F ,0x8C ,0x3C ,0x00 ,0x0A ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x01 ,0x5A ,
0x02 ,0x03 ,0x37 ,0xF0 ,0x5B ,0x10 ,0x1F ,0x5F ,0x14 ,0x05 ,0x13 ,0x04 ,0x20 ,0x22 ,0x3C ,0x3E ,
0x12 ,0x16 ,0x03 ,0x07 ,0x11 ,0x15 ,0x06 ,0x02 ,0x01 ,0x5D ,0x60 ,0x61 ,0x64 ,0x65 ,0x66 ,0x62 ,
0x23 ,0x09 ,0x07 ,0x01 ,0x83 ,0x01 ,0x00 ,0x00 ,0x6E ,0x03 ,0x0C ,0x00 ,0x10 ,0x00 ,0x88 ,0x3C ,
0x20 ,0x80 ,0x80 ,0x01 ,0x02 ,0x03 ,0x04 ,0x02 ,0x3A ,0x80 ,0xD0 ,0x72 ,0x38 ,0x2D ,0x40 ,0x10 ,
0x2C ,0x45 ,0x80 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x1F ,0x01 ,0x1D ,0x00 ,0xBC ,0x52 ,0xD0 ,0x1E ,
0x20 ,0xB8 ,0x28 ,0x55 ,0x40 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x1F ,0x01 ,0x1D ,0x80 ,0xD0 ,0x72 ,
0x1C ,0x16 ,0x20 ,0x10 ,0x2C ,0x25 ,0x80 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x9F ,0x8C ,0x0A ,0xD0 ,
0x8A ,0x20 ,0xE0 ,0x2D ,0x10 ,0x10 ,0x3E ,0x96 ,0x00 ,0x13 ,0x8E ,0x21 ,0x00 ,0x00 ,0x18 ,0x47
};

static unsigned char amlogic_hdmirx_edid_port2[] =
{
0x00 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0x00 ,0x05 ,0xAC ,0x30 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,
0x0A ,0x18 ,0x01 ,0x03 ,0x80 ,0x73 ,0x41 ,0x78 ,0x0A ,0xCF ,0x74 ,0xA3 ,0x57 ,0x4C ,0xB0 ,0x23 ,
0x09 ,0x48 ,0x4C ,0x00 ,0x00 ,0x00 ,0x01 ,0x01 ,0x01 ,0xFF ,0x01 ,0xFF ,0xFF ,0x01 ,0x01 ,0x01 ,
0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x20 ,0x02 ,0x3A ,0x80 ,0x18 ,0x71 ,0x38 ,0x2D ,0x40 ,0x58 ,0x2C ,
0x45 ,0x00 ,0x20 ,0x52 ,0x31 ,0x00 ,0x00 ,0x1E ,0x02 ,0x3A ,0x80 ,0x18 ,0x71 ,0x38 ,0x2D ,0x40 ,
0x58 ,0x2C ,0x45 ,0x00 ,0x20 ,0xC2 ,0x31 ,0x00 ,0x00 ,0x1E ,0x00 ,0x00 ,0x00 ,0xFC ,0x00 ,0x41 ,
0x4D ,0x4C ,0x20 ,0x38 ,0x36 ,0x36 ,0x0A ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x00 ,0x00 ,0x00 ,0xFD ,
0x00 ,0x3B ,0x46 ,0x1F ,0x8C ,0x3C ,0x00 ,0x0A ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x01 ,0x5A ,
0x02 ,0x03 ,0x37 ,0xF0 ,0x5B ,0x10 ,0x1F ,0x5F ,0x14 ,0x05 ,0x13 ,0x04 ,0x20 ,0x22 ,0x3C ,0x3E ,
0x12 ,0x16 ,0x03 ,0x07 ,0x11 ,0x15 ,0x06 ,0x02 ,0x01 ,0x5D ,0x60 ,0x61 ,0x64 ,0x65 ,0x66 ,0x62 ,
0x23 ,0x09 ,0x07 ,0x01 ,0x83 ,0x01 ,0x00 ,0x00 ,0x6E ,0x03 ,0x0C ,0x00 ,0x20 ,0x00 ,0x88 ,0x3C ,
0x20 ,0x80 ,0x80 ,0x01 ,0x02 ,0x03 ,0x04 ,0x02 ,0x3A ,0x80 ,0xD0 ,0x72 ,0x38 ,0x2D ,0x40 ,0x10 ,
0x2C ,0x45 ,0x80 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x1F ,0x01 ,0x1D ,0x00 ,0xBC ,0x52 ,0xD0 ,0x1E ,
0x20 ,0xB8 ,0x28 ,0x55 ,0x40 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x1F ,0x01 ,0x1D ,0x80 ,0xD0 ,0x72 ,
0x1C ,0x16 ,0x20 ,0x10 ,0x2C ,0x25 ,0x80 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x9F ,0x8C ,0x0A ,0xD0 ,
0x8A ,0x20 ,0xE0 ,0x2D ,0x10 ,0x10 ,0x3E ,0x96 ,0x00 ,0x13 ,0x8E ,0x21 ,0x00 ,0x00 ,0x18 ,0x37
};

static unsigned char amlogic_hdmirx_edid_port3[] =
{
0x00 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0x00 ,0x05 ,0xAC ,0x30 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,
0x0A ,0x18 ,0x01 ,0x03 ,0x80 ,0x73 ,0x41 ,0x78 ,0x0A ,0xCF ,0x74 ,0xA3 ,0x57 ,0x4C ,0xB0 ,0x23 ,
0x09 ,0x48 ,0x4C ,0x00 ,0x00 ,0x00 ,0x01 ,0x01 ,0x01 ,0xFF ,0x01 ,0xFF ,0xFF ,0x01 ,0x01 ,0x01 ,
0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x20 ,0x02 ,0x3A ,0x80 ,0x18 ,0x71 ,0x38 ,0x2D ,0x40 ,0x58 ,0x2C ,
0x45 ,0x00 ,0x20 ,0x52 ,0x31 ,0x00 ,0x00 ,0x1E ,0x02 ,0x3A ,0x80 ,0x18 ,0x71 ,0x38 ,0x2D ,0x40 ,
0x58 ,0x2C ,0x45 ,0x00 ,0x20 ,0xC2 ,0x31 ,0x00 ,0x00 ,0x1E ,0x00 ,0x00 ,0x00 ,0xFC ,0x00 ,0x41 ,
0x4D ,0x4C ,0x20 ,0x38 ,0x36 ,0x36 ,0x0A ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x00 ,0x00 ,0x00 ,0xFD ,
0x00 ,0x3B ,0x46 ,0x1F ,0x8C ,0x3C ,0x00 ,0x0A ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x01 ,0x5A ,
0x02 ,0x03 ,0x37 ,0xF0 ,0x5B ,0x10 ,0x1F ,0x5F ,0x14 ,0x05 ,0x13 ,0x04 ,0x20 ,0x22 ,0x3C ,0x3E ,
0x12 ,0x16 ,0x03 ,0x07 ,0x11 ,0x15 ,0x06 ,0x02 ,0x01 ,0x5D ,0x60 ,0x61 ,0x64 ,0x65 ,0x66 ,0x62 ,
0x23 ,0x09 ,0x07 ,0x01 ,0x83 ,0x01 ,0x00 ,0x00 ,0x6E ,0x03 ,0x0C ,0x00 ,0x30 ,0x00 ,0x88 ,0x3C ,
0x20 ,0x80 ,0x80 ,0x01 ,0x02 ,0x03 ,0x04 ,0x02 ,0x3A ,0x80 ,0xD0 ,0x72 ,0x38 ,0x2D ,0x40 ,0x10 ,
0x2C ,0x45 ,0x80 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x1F ,0x01 ,0x1D ,0x00 ,0xBC ,0x52 ,0xD0 ,0x1E ,
0x20 ,0xB8 ,0x28 ,0x55 ,0x40 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x1F ,0x01 ,0x1D ,0x80 ,0xD0 ,0x72 ,
0x1C ,0x16 ,0x20 ,0x10 ,0x2C ,0x25 ,0x80 ,0x30 ,0xEB ,0x52 ,0x00 ,0x00 ,0x9F ,0x8C ,0x0A ,0xD0 ,
0x8A ,0x20 ,0xE0 ,0x2D ,0x10 ,0x10 ,0x3E ,0x96 ,0x00 ,0x13 ,0x8E ,0x21 ,0x00 ,0x00 ,0x18 ,0x27
};
#endif
static unsigned char hdmirx_8bit_3d_edid_port1[] =
{
//8 bit only with 3D
0x00 ,0xff ,0xff ,0xff ,0xff ,0xff ,0xff ,0x00 ,0x4d ,0x77 ,0x02 ,0x2c ,0x01 ,0x01 ,0x01 ,0x01,
0x01 ,0x15 ,0x01 ,0x03 ,0x80 ,0x85 ,0x4b ,0x78 ,0x0a ,0x0d ,0xc9 ,0xa0 ,0x57 ,0x47 ,0x98 ,0x27,
0x12 ,0x48 ,0x4c ,0x21 ,0x08 ,0x00 ,0x81 ,0x80 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01,
0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x02 ,0x3a ,0x80 ,0x18 ,0x71 ,0x38 ,0x2d ,0x40 ,0x58 ,0x2c,
0x45 ,0x00 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x01 ,0x1d ,0x00 ,0x72 ,0x51 ,0xd0 ,0x1e ,0x20,
0x6e ,0x28 ,0x55 ,0x00 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x00 ,0x00 ,0x00 ,0xfc ,0x00 ,0x53,
0x6b ,0x79 ,0x77 ,0x6f ,0x72 ,0x74 ,0x68 ,0x20 ,0x54 ,0x56 ,0x0a ,0x20 ,0x00 ,0x00 ,0x00 ,0xfd,
0x00 ,0x30 ,0x3e ,0x0e ,0x46 ,0x0f ,0x00 ,0x0a ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x01 ,0xdc,
0x02 ,0x03 ,0x38 ,0xf0 ,0x53 ,0x1f ,0x10 ,0x14 ,0x05 ,0x13 ,0x04 ,0x20 ,0x22 ,0x3c ,0x3e ,0x12,
0x16 ,0x03 ,0x07 ,0x11 ,0x15 ,0x02 ,0x06 ,0x01 ,0x23 ,0x09 ,0x07 ,0x01 ,0x83 ,0x01 ,0x00 ,0x00,
0x74 ,0x03 ,0x0c ,0x00 ,0x10 ,0x00 ,0x88 ,0x2d ,0x2f ,0xd0 ,0x0a ,0x01 ,0x40 ,0x00 ,0x7f ,0x20,
0x30 ,0x70 ,0x80 ,0x90 ,0x76 ,0xe2 ,0x00 ,0xfb ,0x02 ,0x3a ,0x80 ,0xd0 ,0x72 ,0x38 ,0x2d ,0x40,
0x10 ,0x2c ,0x45 ,0x80 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x01 ,0x1d ,0x00 ,0xbc ,0x52 ,0xd0,
0x1e ,0x20 ,0xb8 ,0x28 ,0x55 ,0x40 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x01 ,0x1d ,0x80 ,0xd0,
0x72 ,0x1c ,0x16 ,0x20 ,0x10 ,0x2c ,0x25 ,0x80 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x9e ,0x00 ,0x00,
0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x8e
};
static unsigned char hdmirx_12bit_3d_edid_port1 [] =
{
0x00 ,0xff ,0xff ,0xff ,0xff ,0xff ,0xff ,0x00 ,0x4d ,0xd9 ,0x02 ,0x2c ,0x01 ,0x01 ,0x01 ,0x01,
0x01 ,0x15 ,0x01 ,0x03 ,0x80 ,0x85 ,0x4b ,0x78 ,0x0a ,0x0d ,0xc9 ,0xa0 ,0x57 ,0x47 ,0x98 ,0x27,
0x12 ,0x48 ,0x4c ,0x21 ,0x08 ,0x00 ,0x81 ,0x80 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01,
0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x02 ,0x3a ,0x80 ,0x18 ,0x71 ,0x38 ,0x2d ,0x40 ,0x58 ,0x2c,
0x45 ,0x00 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x01 ,0x1d ,0x00 ,0x72 ,0x51 ,0xd0 ,0x1e ,0x20,
0x6e ,0x28 ,0x55 ,0x00 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x00 ,0x00 ,0x00 ,0xfc ,0x00 ,0x53,
0x4f ,0x4e ,0x59 ,0x20 ,0x54 ,0x56 ,0x0a ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x00 ,0x00 ,0x00 ,0xfd,
0x00 ,0x30 ,0x3e ,0x0e ,0x46 ,0x0f ,0x00 ,0x0a ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x01 ,0x1c,
0x02 ,0x03 ,0x3b ,0xf0 ,0x53 ,0x1f ,0x10 ,0x14 ,0x05 ,0x13 ,0x04 ,0x20 ,0x22 ,0x3c ,0x3e ,0x12,
0x16 ,0x03 ,0x07 ,0x11 ,0x15 ,0x02 ,0x06 ,0x01 ,0x26 ,0x09 ,0x07 ,0x07 ,0x15 ,0x07 ,0x50 ,0x83,
0x01 ,0x00 ,0x00 ,0x74 ,0x03 ,0x0c ,0x00 ,0x20 ,0x00 ,0xb8 ,0x2d ,0x2f ,0xd0 ,0x0a ,0x01 ,0x40,
0x00 ,0x7f ,0x20 ,0x30 ,0x70 ,0x80 ,0x90 ,0x76 ,0xe2 ,0x00 ,0xfb ,0x02 ,0x3a ,0x80 ,0xd0 ,0x72,
0x38 ,0x2d ,0x40 ,0x10 ,0x2c ,0x45 ,0x80 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x01 ,0x1d ,0x00,
0xbc ,0x52 ,0xd0 ,0x1e ,0x20 ,0xb8 ,0x28 ,0x55 ,0x40 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00 ,0x1e ,0x01,
0x1d ,0x80 ,0xd0 ,0x72 ,0x1c ,0x16 ,0x20 ,0x10 ,0x2c ,0x25 ,0x80 ,0x30 ,0xeb ,0x52 ,0x00 ,0x00,
0x9e ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xd9,
};

/* OLD EDID before 2013.08*/
static unsigned char hdmirx_test_edid_port_Old [] =
{
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0xAC, 0x02, 0x2C, 0x01, 0x01, 0x01, 0x01,
0x26, 0x18, 0x01, 0x03, 0x80, 0x85, 0x4B, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x08, 0xE8, 0x00, 0xA0, 0xF5, 0x70, 0x5A, 0x80,
0xB0, 0x58, 0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x04, 0x74, 0x00, 0x30, 0xf2, 0x70,
0x5a, 0x80, 0xb0, 0x58, 0x8a, 0x00, 0x20, 0xc2, 0x31, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xFc,
0x00, 0x41, 0x4d, 0x4c, 0x20, 0x46, 0x42, 0x43, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xc2,
0x02, 0x03, 0x1F, 0xF0, 0x43, 0x5F, 0x61, 0x5E, 0x23, 0x09, 0x07, 0x01, 0x83, 0x01, 0x00, 0x00,
0x6E, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x00, 0x3C, 0x20, 0xA0, 0x42, 0x03, 0x02, 0x81, 0x7F, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03
};

/* test EDID SP*/
static unsigned char hdmirx_test_edid_port_SP [] =
{
0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x4D,0x10,0xDB,0x10,0x00,0x00,0x00,0x00,
0xFF,0x17,0x01,0x03,0x80,0x85,0x4B,0x78,0x2A,0x1B,0xBE,0xA2,0x55,0x34,0xB3,0x26,
0x14,0x4A,0x52,0xAF,0xCE,0x00,0x90,0x40,0x81,0x80,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x3A,0x80,0xD0,0x72,0x38,0x2D,0x40,0x10,0x2C,
0x45,0x80,0x31,0xEB,0x52,0x00,0x00,0x1E,0x66,0x21,0x50,0xB0,0x51,0x00,0x1B,0x30,
0x40,0x70,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x00,0x00,0x00,0xFC,0x00,0x53,
0x48,0x41,0x52,0x50,0x20,0x48,0x44,0x4D,0x49,0x0A,0x20,0x20,0x00,0x00,0x00,0xFD,
0x00,0x17,0x4C,0x0E,0x44,0x0F,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xE2,
0x02,0x03,0x28,0x72,0x50,0x9F,0x90,0x20,0x14,0x05,0x13,0x04,0x12,0x03,0x11,0x02,
0x16,0x07,0x15,0x06,0x01,0x23,0x09,0x07,0x07,0x83,0x01,0x00,0x00,0x67,0x03,0x0C,
0x00,0x10,0x00,0x80,0x1E,0xE2,0x00,0x3F,0x02,0x3A,0x80,0x18,0x71,0x38,0x2D,0x40,
0x58,0x2C,0x45,0x00,0x31,0xEB,0x52,0x00,0x00,0x1E,0x01,0x1D,0x80,0xD0,0x72,0x1C,
0x16,0x20,0x10,0x2C,0x25,0x80,0x31,0xEB,0x52,0x00,0x00,0x9E,0x01,0x1D,0x80,0x18,
0x71,0x1C,0x16,0x20,0x58,0x2C,0x25,0x00,0x31,0xEB,0x52,0x00,0x00,0x9E,0x01,0x1D,
0x00,0xBC,0x52,0xD0,0x1E,0x20,0xB8,0x28,0x55,0x40,0x31,0xEB,0x52,0x00,0x00,0x1E,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x67
};

/* test EDID ATSC*/
static unsigned char hdmirx_test_edid_port_ATSC [] =
{
0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x08,0x59,0x32,0x00,0x01,0x00,0x00,0x00,
0x1E,0x16,0x01,0x03,0x80,0x29,0x17,0x78,0x0A,0xD0,0xDD,0xA9,0x53,0x49,0x9D,0x23,
0x11,0x47,0x4A,0xA3,0x08,0x00,0x81,0xC0,0x81,0x00,0x81,0x0F,0x81,0x40,0x81,0x80,
0x95,0x00,0xB3,0x00,0x01,0x01,0x66,0x21,0x50,0xB0,0x51,0x00,0x1B,0x30,0x00,0x70,
0x26,0x44,0xE4,0x10,0x11,0x00,0x00,0x1E,0x46,0x20,0x00,0xA4,0x51,0x00,0x2A,0x30,
0x50,0x80,0x37,0x00,0x80,0x80,0x21,0x00,0x00,0x1C,0x00,0x00,0x00,0xFC,0x00,0x4E,
0x53,0x2D,0x31,0x39,0x45,0x33,0x31,0x30,0x41,0x31,0x33,0x0A,0x00,0x00,0x00,0xFD,
0x00,0x37,0x4C,0x1E,0x50,0x11,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xFC,
0x02,0x03,0x20,0x73,0x48,0x85,0x04,0x03,0x02,0x01,0x06,0x07,0x10,0x26,0x09,0x07,
0x07,0x15,0x07,0x50,0x83,0x01,0x00,0x00,0x67,0x03,0x0C,0x00,0x40,0x00,0xB8,0x2D,
0x01,0x1D,0x00,0x72,0x51,0xD0,0x1E,0x20,0x6E,0x28,0x55,0x00,0xC4,0x8E,0x21,0x00,
0x00,0x1E,0x8C,0x0A,0xD0,0x8A,0x20,0xE0,0x2D,0x10,0x10,0x3E,0x96,0x00,0x13,0x8E,
0x21,0x00,0x00,0x18,0x01,0x1D,0x80,0x18,0x71,0x1C,0x16,0x20,0x58,0x2C,0x25,0x00,
0xC4,0x8E,0x21,0x00,0x00,0x9E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x23

};
/* test EDID skyworth mst/mtk */
static unsigned char hdmirx_test_edid_port_skyworth_mstmtk [] =
{
0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x4D,0x77,0x01,0x00,0x01,0x00,0x00,0x00,
0x1C,0x16,0x01,0x03,0x80,0x3C,0x22,0x78,0x0A,0x0D,0xC9,0xA0,0x57,0x47,0x98,0x27,
0x12,0x48,0x4C,0xBF,0xEF,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x1D,0x00,0x72,0x51,0xD0,0x1E,0x20,0x6E,0x28,
0x55,0x00,0xC4,0x8E,0x21,0x00,0x00,0x1E,0x01,0x1D,0x80,0x18,0x71,0x1C,0x16,0x20,
0x58,0x2C,0x25,0x00,0xC4,0x8E,0x21,0x00,0x00,0x9E,0x00,0x00,0x00,0xFC,0x00,0x20,
0x38,0x4B,0x35,0x35,0x20,0x20,0x20,0x20,0x4C,0x45,0x44,0x0A,0x00,0x00,0x00,0xFD,
0x00,0x31,0x4C,0x0F,0x50,0x0E,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xBB,
0x02,0x03,0x29,0x74,0x4B,0x84,0x10,0x1F,0x05,0x13,0x14,0x01,0x02,0x11,0x06,0x15,
0x26,0x09,0x7F,0x03,0x11,0x7F,0x18,0x83,0x01,0x00,0x00,0x6D,0x03,0x0C,0x00,0x10,
0x00,0xB8,0x3C,0x2F,0x80,0x60,0x01,0x02,0x03,0x01,0x1D,0x00,0xBC,0x52,0xD0,0x1E,
0x20,0xB8,0x28,0x55,0x40,0xC4,0x8E,0x21,0x00,0x00,0x1E,0x01,0x1D,0x80,0xD0,0x72,
0x1C,0x16,0x20,0x10,0x2C,0x25,0x80,0xC4,0x8E,0x21,0x00,0x00,0x9E,0x8C,0x0A,0xD0,
0x8A,0x20,0xE0,0x2D,0x10,0x10,0x3E,0x96,0x00,0x13,0x8E,0x21,0x00,0x00,0x18,0x8C,
0x0A,0xD0,0x90,0x20,0x40,0x31,0x20,0x0C,0x40,0x55,0x00,0x13,0x8E,0x21,0x00,0x00,
0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22
};
/* test EDID skyworth mst/mtk */
static unsigned char hdmirx_test_edid_enable_rgb_range_block [] =
{
0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x05,0xAC,0x02,0x2C,0x01,0x01,0x01,0x01,
0x01,0x15,0x01,0x03,0x80,0x85,0x4B,0x78,0x0A,0x0D,0xC9,0xA0,0x57,0x47,0x98,0x27,
0x12,0x48,0x4C,0x21,0x08,0x00,0x81,0x80,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x3A,0x80,0x18,0x71,0x38,0x2D,0x40,0x58,0x2C,
0x45,0x00,0x30,0xEB,0x52,0x00,0x00,0x1E,0x01,0x1D,0x00,0x72,0x51,0xD0,0x1E,0x20,
0x6E,0x28,0x55,0x00,0x30,0xEB,0x52,0x00,0x00,0x1E,0x00,0x00,0x00,0xFC,0x00,0x41,
0x4D,0x4C,0x4F,0x47,0x49,0x43,0x20,0x54,0x56,0x0A,0x20,0x20,0x00,0x00,0x00,0xFD,
0x00,0x30,0x3E,0x0E,0x46,0x0F,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0x3E,
0x02,0x03,0x32,0xF0,0x53,0x1F,0x10,0x14,0x05,0x13,0x04,0x20,0x22,0x3C,0x3E,0x12,
0x16,0x03,0x07,0x11,0x15,0x02,0x06,0x01,0x23,0x09,0x07,0x01,0x83,0x01,0x00,0x00,
0x6E,0x03,0x0C,0x00,0x10,0x00,0x88,0x3C,0x20,0x80,0x80,0x01,0x02,0x03,0x04,0xE2,
0x00,0x40,0x02,0x3A,0x80,0xD0,0x72,0x38,0x2D,0x40,0x10,0x2C,0x45,0x80,0x30,0xEB,
0x52,0x00,0x00,0x1E,0x01,0x1D,0x00,0xBC,0x52,0xD0,0x1E,0x20,0xB8,0x28,0x55,0x40,
0x30,0xEB,0x52,0x00,0x00,0x1E,0x01,0x1D,0x80,0xD0,0x72,0x1C,0x16,0x20,0x10,0x2C,
0x25,0x80,0x30,0xEB,0x52,0x00,0x00,0x9E,0x8C,0x0A,0xD0,0x8A,0x20,0xE0,0x2D,0x10,
0x10,0x3E,0x96,0x00,0x13,0x8E,0x21,0x00,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x43
};
/* test EDID skyworth mst/mtk */
static unsigned char hdmirx_test_edid_4K_MSD [] =
{
0x00 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0x00 ,0x36 ,0x74 ,0x30 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00,
0x0A ,0x18 ,0x01 ,0x03 ,0x80 ,0x73 ,0x41 ,0x78 ,0x0A ,0xCF ,0x74 ,0xA3 ,0x57 ,0x4C ,0xB0 ,0x23,
0x09 ,0x48 ,0x4C ,0x00 ,0x00 ,0x00 ,0x01 ,0x01 ,0x01 ,0xFF ,0x01 ,0xFF ,0xFF ,0x01 ,0x01 ,0x01,
0x01 ,0x01 ,0x01 ,0x01 ,0x01 ,0x20 ,0x08 ,0xE8 ,0x00 ,0x30 ,0xF2 ,0x70 ,0x5A ,0x80 ,0xB0 ,0x58,
0x8A ,0x00 ,0xC4 ,0x8E ,0x21 ,0x00 ,0x00 ,0x1E ,0x02 ,0x3A ,0x80 ,0x18 ,0x71 ,0x38 ,0x2D ,0x40,
0x58 ,0x2C ,0x45 ,0x00 ,0xC4 ,0x8E ,0x21 ,0x00 ,0x00 ,0x1E ,0x00 ,0x00 ,0x00 ,0xFC ,0x00 ,0x4D,
0x53 ,0x74 ,0x61 ,0x72 ,0x20 ,0x44 ,0x65 ,0x6D ,0x6F ,0x0A ,0x20 ,0x20 ,0x00 ,0x00 ,0x00 ,0xFD,
0x00 ,0x3B ,0x46 ,0x1F ,0x8C ,0x3C ,0x00 ,0x0A ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x01 ,0x68,
0x02 ,0x03 ,0x4C ,0xF2 ,0x5B ,0x05 ,0x84 ,0x03 ,0x01 ,0x12 ,0x13 ,0x14 ,0x16 ,0x07 ,0x90 ,0x1F,
0x20 ,0x22 ,0x5D ,0x5F ,0x60 ,0x61 ,0x62 ,0x64 ,0x65 ,0x66 ,0x5E ,0x63 ,0x02 ,0x06 ,0x11 ,0x15,
0x26 ,0x09 ,0x07 ,0x07 ,0x11 ,0x07 ,0x06 ,0x83 ,0x01 ,0x00 ,0x00 ,0x6E ,0x03 ,0x0C ,0x00 ,0x10,
0x00 ,0x78 ,0x44 ,0x20 ,0x00 ,0x80 ,0x01 ,0x02 ,0x03 ,0x04 ,0x67 ,0xD8 ,0x5D ,0xC4 ,0x01 ,0x78,
0xC8 ,0x07 ,0xE3 ,0x05 ,0x03 ,0x01 ,0xE5 ,0x0F ,0x00 ,0x80 ,0x19 ,0x00 ,0x8C ,0x0A ,0xD0 ,0x8A,
0x20 ,0xE0 ,0x2D ,0x10 ,0x10 ,0x3E ,0x96 ,0x00 ,0xC4 ,0x8E ,0x21 ,0x00 ,0x00 ,0x18 ,0x8C ,0x0A,
0xA0 ,0x14 ,0x51 ,0xF0 ,0x16 ,0x00 ,0x26 ,0x7C ,0x43 ,0x00 ,0xC4 ,0x8E ,0x21 ,0x00 ,0x00 ,0x98,
0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x71
};

/* AML EDID JIASHI 15.06.30 */
static unsigned char hdmirx_aml_edid_domy_port1 [] =
{
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x11, 0xB9, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00,
0x20, 0x19, 0x01, 0x03, 0x80, 0x73, 0x41, 0x78, 0x0A, 0xCF, 0x74, 0xA3, 0x57, 0x4C, 0xB0, 0x23,
0x09, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
0x58, 0x2C, 0x45, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x44,
0x6F, 0x6D, 0x79, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x46, 0x1F, 0x8C, 0x3C, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x4B,
0x02, 0x03, 0x36, 0xF0, 0x5B, 0x5F, 0x10, 0x1F, 0x14, 0x05, 0x13, 0x04, 0x20, 0x22, 0x3C, 0x3E,
0x12, 0x16, 0x03, 0x07, 0x11, 0x15, 0x02, 0x06, 0x01, 0x61, 0x5D, 0x64, 0x65, 0x66, 0x62, 0x60,
0x29, 0x09, 0x07, 0x01, 0x15, 0x07, 0x00, 0x57, 0x06, 0x00, 0x83, 0x01, 0x00, 0x00, 0x67, 0x03,
0x0C, 0x00, 0x10, 0x00, 0x88, 0x3C, 0x02, 0x3A, 0x80, 0xD0, 0x72, 0x38, 0x2D, 0x40, 0x10, 0x2C,
0x45, 0x80, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E, 0x20,
0xB8, 0x28, 0x55, 0x40, 0x30, 0xEB, 0x52, 0x00, 0x00, 0x1F, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0,
0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19
};

int hdmi_rx_ctrl_edid_update(void)
{
	int i, ram_addr, byte_num;
	unsigned char value = 0;
	//unsigned int data32 = 0;
	unsigned char check_sum = 0;
	unsigned char phy_addr_offset = 0;
	//unsigned long tmp32;
	//printk("HDMI_OTHER_CTRL2=%x\n", hdmi_rd_reg(OTHER_BASE_ADDR+HDMI_OTHER_CTRL2));
	if(((edid_mode&0x100)==0) && edid_size>4 && edid_buf[0]=='E' && edid_buf[1]=='D' && edid_buf[2]=='I' && edid_buf[3]=='D'){
		hdmirx_print("edid: use Top edid\n");
		for (i = 0; i < (edid_size-4); i++)
		{
			value = edid_buf[i+4];
			if(value == 0x03) {
				if((0x0c == edid_buf[i+5]) &&
					(0x00 == edid_buf[i+6]) &&
					(0x00 == edid_buf[i+8])) {
					edid_buf[i+7] = 0x10;
					phy_addr_offset = i+7-4;
				}
			}
			if((i >= 128) && (i < 255)){
				check_sum += value;
				check_sum &= 0xff;
			}
			if(i == 255) {
				value = (0x100-check_sum)&0xff;
				check_sum = 0;
		 	}
			ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
		    hdmirx_wr_top(ram_addr, value);
			hdmirx_wr_top(ram_addr+0x100, value);
		}
		if(multi_port_edid_enable) {
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR1, phy_addr_offset | (0x0f<<16));
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR2, (0x100+phy_addr_offset) | (0x0f<<16));
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR1_DATA, 0x10|0x20<<8|0x30<<16);
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR2_DATA, 0x10|0x20<<8|0x30<<16);
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0, 0xff | (0x0f<<16));
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3, 0x1ff | (0x0f<<16));
			if(value >= 0x20){
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0_DATA, value|(value-0x10)<<8|(value-0x20)<<16);
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3_DATA, value|(value-0x10)<<8|(value-0x20)<<16);
			}
			else if(value >= 0x10){
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0_DATA, value|(value-0x10)<<8|(0x100+value-0x20)<<16);
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3_DATA, value|(value-0x10)<<8|(0x100+value-0x20)<<16);
			}
			else{
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0_DATA, value|(0x100+value-0x10)<<8|(0x100+value-0x20)<<16);
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3_DATA, value|(0x100+value-0x10)<<8|(0x100+value-0x20)<<16);
			}
		}
	} else {
		if((edid_mode&0xf) == 0){
			unsigned char * p_edid_array;
			byte_num = sizeof(amlogic_hdmirx_edid_port1)/sizeof(unsigned char);
			p_edid_array =  amlogic_hdmirx_edid_port1;
			if(multi_port_edid_enable == 0){
				if(rx.port == 1){
					byte_num = sizeof(amlogic_hdmirx_edid_port2)/sizeof(unsigned char);
					p_edid_array =  amlogic_hdmirx_edid_port2;
				} else if (rx.port == 2) {
					byte_num = sizeof(amlogic_hdmirx_edid_port3)/sizeof(unsigned char);
					p_edid_array =  amlogic_hdmirx_edid_port3;
				} else if (rx.port == 3) {
					byte_num = sizeof(amlogic_hdmirx_edid_port3)/sizeof(unsigned char);
					p_edid_array =  amlogic_hdmirx_edid_port3;
				}
			}
			for (i = 0; i < byte_num; i++){
				value = p_edid_array[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
				hdmirx_wr_top(ram_addr+0x100, value);
			}
			if(multi_port_edid_enable) {
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR1, 0xAC | (0x0f<<16));
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR1_DATA, 0x10|0x20<<8|0x30<<16);
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0, 0xff | (0x0f<<16));
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0_DATA, 0x47|0x37<<8|0x27<<16);

				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR2, 0x1AC | (0x0f<<16));
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR2_DATA, 0x10|0x20<<8|0x30<<16);
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3, 0x1ff | (0x0f<<16));
				hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3_DATA, 0x47|0x37<<8|0x27<<16);
			}
		}else if((edid_mode&0xf) == 1){
			byte_num = sizeof(hdmirx_12bit_3d_edid_port1)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_12bit_3d_edid_port1[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 2){
			byte_num = sizeof(hdmirx_8bit_3d_edid_port1)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_8bit_3d_edid_port1[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 3){
			byte_num = sizeof(hdmirx_test_edid_port_Old)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_test_edid_port_Old[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 4){
			byte_num = sizeof(hdmirx_test_edid_port_SP)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_test_edid_port_SP[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 5){
			byte_num = sizeof(hdmirx_test_edid_port_ATSC)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_test_edid_port_ATSC[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 6){
			byte_num = sizeof(hdmirx_test_edid_port_skyworth_mstmtk)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_test_edid_port_skyworth_mstmtk[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 7){
			byte_num = sizeof(hdmirx_test_edid_enable_rgb_range_block)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_test_edid_enable_rgb_range_block[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 8){
			byte_num = sizeof(hdmirx_test_edid_4K_MSD)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_test_edid_4K_MSD[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
		}else if((edid_mode&0xf) == 9){
			byte_num = sizeof(hdmirx_aml_edid_domy_port1)/sizeof(unsigned char);
			for (i = 0; i < byte_num; i++){
				value = hdmirx_aml_edid_domy_port1[i];
				ram_addr = HDMIRX_TOP_EDID_OFFSET+i;
				hdmirx_wr_top(ram_addr, value);
			}
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR1, 0xB2 | (0x0f<<16));
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR1_DATA, 0x10|0x20<<8|0x30<<16);
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0, 0xff | (0x0f<<16));
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR0_DATA, 0x19|0x09<<8|0xF9<<16);

			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR2, 0x1B2 | (0x0f<<16));
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR2_DATA, 0x10|0x20<<8|0x30<<16);
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3, 0x1ff | (0x0f<<16));
			hdmirx_wr_top( HDMIRX_TOP_EDID_RAM_OVR3_DATA, 0x19|0x09<<8|0xF9<<16);
		}
	}



	return 0;
}

static void set_hdcp(struct hdmi_rx_ctrl_hdcp *hdcp, const unsigned char* b_key)
{
    int i,j;
    memset(&init_hdcp_data, 0, sizeof(struct hdmi_rx_ctrl_hdcp));
    for(i=0,j=0; i<80; i+=2,j+=7){
        hdcp->keys[i+1] = b_key[j]|(b_key[j+1]<<8)|(b_key[j+2]<<16)|(b_key[j+3]<<24);
        hdcp->keys[i+0] = b_key[j+4]|(b_key[j+5]<<8)|(b_key[j+6]<<16);
    }
    hdcp->bksv[1] = b_key[j]|(b_key[j+1]<<8)|(b_key[j+2]<<16)|(b_key[j+3]<<24);
    hdcp->bksv[0] = b_key[j+4];

}

int hdmirx_read_key_buf(char* buf, int max_size)
{
	if(key_size > max_size){
		pr_err("Error: %s, key size %d is larger than the buf size of %d\n", __func__,  key_size, max_size);
		return 0;
	}
	memcpy(buf, key_buf, key_size);
	pr_info("HDMIRX: read key buf\n");
	return key_size;
}

void hdmirx_fill_key_buf(const char* buf, int size)
{
	if(size > MAX_KEY_BUF_SIZE){
		pr_err("Error: %s, key size %d is larger than the max size of %d\n", __func__,  size, MAX_KEY_BUF_SIZE);
		return;
	}
	if(buf[0]=='k' && buf[1]=='e' && buf[2]=='y'){
	    set_hdcp(&init_hdcp_data, buf+3);
	}
	else{
		memcpy(key_buf, buf, size);
		key_size = size;
		pr_info("HDMIRX: fill key buf, size %d\n", size);
	}
}

int hdmirx_read_edid_buf(char* buf, int max_size)
{
	if(edid_size > max_size){
		pr_err("Error: %s, edid size %d is larger than the buf size of %d\n", __func__,  edid_size, max_size);
		return 0;
	}
	memcpy(buf, edid_buf, edid_size);
	pr_info("HDMIRX: read edid buf\n");
	return edid_size;
}

void hdmirx_fill_edid_buf(const char* buf, int size)
{
	if(size > MAX_EDID_BUF_SIZE){
		pr_err("Error: %s, edid size %d is larger than the max size of %d\n", __func__,  size, MAX_EDID_BUF_SIZE);
		return;
	}
	memcpy(edid_buf, buf, size);
	edid_size = size;
	pr_info("HDMIRX: fill edid buf, size %d\n", size);
}

/********************
    debug functions
*********************/
int hdmirx_hw_dump_reg(unsigned char* buf, int size)
{
	return 0;
}

static void dump_state(unsigned char enable)
{
	int error = 0;
	//int i = 0;
	struct hdmi_rx_ctrl_video v;
  	static struct aud_info_s a;
	struct vendor_specific_info_s vsi;
  	if(enable&1){
    		hdmirx_get_video_info(&rx.ctrl, &v);
      		printk("[HDMI info]error %d video_format %d VIC %d dvi %d interlace %d\nhtotal %d vtotal %d hactive %d vactive %d pixel_repetition %d\npixel_clk %ld deep_color %d refresh_rate %d\n",
        		error,
        		v.video_format, v.video_mode, v.dvi, v.interlaced,
        		v.htotal, v.vtotal, v.hactive, v.vactive, v.pixel_repetition,
        		v.pixel_clk, v.deep_color_mode, v.refresh_rate);
     		hdmirx_read_vendor_specific_info_frame(&vsi);
      		printk("Vendor Specific Info ID=%x, hdmi_video_format=%x, 3d_struct=%x, 3d_ext=%x\n",
       		vsi.identifier, vsi.hdmi_video_format, vsi._3d_structure, vsi._3d_ext_data);
  	}
  	if(enable&2){
      		hdmirx_read_audio_info(&a);
    		printk("AudioInfo: CT=%u CC=%u SF=%u SS=%u CA=%u",
    			a.coding_type, a.channel_count, a.sample_frequency,
    			a.sample_size, a.channel_allocation);

      		printk("CTS=%d, N=%d, recovery clock is %d\n", a.cts, a.n, a.audio_recovery_clock);
  	}
 	printk("TMDS clock = %d, Pixel clock = %d\n", hdmirx_get_tmds_clock(), hdmirx_get_pixel_clock());

  	printk("rx.no_signal=%d, rx.state=%d, fmt=0x%x, sw_vic:%d, sw_dvi:%d, sw_fp:%d,, sw_alternative:%d\n",
  	        rx.no_signal, rx.state, hdmirx_hw_get_fmt(),rx.pre_video_params.sw_vic,rx.pre_video_params.sw_dvi,rx.pre_video_params.sw_fp\
  	      ,rx.pre_video_params.sw_alternative);

  	printk("HDCP debug value=0x%x\n", hdmirx_rd_dwc(HDMIRX_DWC_HDCP_DBG));
	printk("HDCP encrypted state:%d\n", v.hdcp_enc_state);
}

static void dump_audio_info(unsigned char enable)
{
    static struct aud_info_s a;

    if(enable){
        hdmirx_read_audio_info(&a);
        printk("AudioInfo: CT=%u CC=%u SF=%u SS=%u CA=%u",
                a.coding_type, a.channel_count, a.sample_frequency,
                a.sample_size, a.channel_allocation);
        printk("[hdmirx]CTS=%d, N=%d, recovery clock is %d\n", a.cts, a.n, a.audio_recovery_clock);
    }
}

void dump_reg(void)
{
	int i = 0;

    printk("\n\n*************Top registers***********\n");
    printk("[addr ]  addr + 0x0,  addr + 0x1,  addr + 0x2,  addr + 0x3\n\n");
    for(i = 0; i <= 0x24; ){
        printk("[0x%-3x]  0x%-8x,  0x%-8x,  0x%-8x,  0x%-8x\n",i,(unsigned int)hdmirx_rd_top(i),(unsigned int)hdmirx_rd_top(i+1),(unsigned int)hdmirx_rd_top(i+2),(unsigned int)hdmirx_rd_top(i+3));
        i = i + 4;
    }
	printk("[0x%-3x]  0x%-8x,  0x%-8x,  0x%-8x\n",i,(unsigned int)hdmirx_rd_top(i),(unsigned int)hdmirx_rd_top(i+1),(unsigned int)hdmirx_rd_top(i+2));
	printk("0x60 : [0x%-3x]  \n",(unsigned int)hdmirx_rd_top(0x60));
    printk("\n\n*************EDID data***************\n");
    printk("[addr ]  addr + 0x0,  addr + 0x1,  addr + 0x2,  addr + 0x3\n\n");
    for(i = 0; i < 256; ){
        printk("[0x%-3x]  0x%-8x,  0x%-8x,  0x%-8x,  0x%-8x\n",i,
                (unsigned int)hdmirx_rd_top(HDMIRX_TOP_EDID_OFFSET+i),(unsigned int)hdmirx_rd_top(HDMIRX_TOP_EDID_OFFSET+i+1),
                (unsigned int)hdmirx_rd_top(HDMIRX_TOP_EDID_OFFSET+i+2),(unsigned int)hdmirx_rd_top(HDMIRX_TOP_EDID_OFFSET+i+3));
        i = i + 4;
    }
	printk("\n\n***********Controller registers***********\n");
	printk("[addr ]  addr + 0x0,  addr + 0x4,  addr + 0x8,  addr + 0xc\n\n");
	for(i = 0; i <= 0xffc; ){
		printk("[0x%-3x]  0x%-8x,  0x%-8x,  0x%-8x,  0x%-8x\n",i,hdmirx_rd_dwc(i),hdmirx_rd_dwc(i+4),hdmirx_rd_dwc(i+8),hdmirx_rd_dwc(i+12));
		i = i + 16;
	}
	printk("\n\n*******PHY registers********\n");
	printk("[addr ]  addr + 0x0,  addr + 0x1,  addr + 0x2,  addr + 0x3\n\n");
	for(i = 0; i <= 0x9a; ){
		printk("[0x%-3x]  0x%-8x,  0x%-8x,  0x%-8x,  0x%-8x\n",i,hdmirx_rd_phy(i),hdmirx_rd_phy(i+1),hdmirx_rd_phy(i+2),hdmirx_rd_phy(i+3));
		i = i + 4;
	}
}

void dump_hdcp_data(void)
{
	int i = 0;
	printk("\n*************HDCP***************");
	printk("\n hdcp-seed = %d ",rx.hdcp.seed);
	//KSV CONFIDENTIAL
	printk("\n hdcp-ksv = %x---%x",rx.hdcp.bksv[0],rx.hdcp.bksv[1]);
	printk("\n hdcp-key:");
	for(i=0; i<HDCP_KEYS_SIZE; i+=4){
		printk("\n%x    %x    %x    %x",rx.hdcp.keys[i],rx.hdcp.keys[i+1],rx.hdcp.keys[i+2],rx.hdcp.keys[i+3]);
	}
}
void dump_edid_reg(void)
{
	int i = 0;
	int j = 0;
	printk("\n*************************************************************\n");
	printk("echo 0x107 > edid_mode ---- enable rgb range block\n");
	printk("echo 0x106 > edid_mode ---- skyworth mst_or_mtk edid\n");
	printk("echo 0x105 > edid_mode ---- mst sharp porduction edid\n");
	printk("echo 0x104 > edid_mode ---- mst ATSC production edid\n");
	printk("echo 0x103 > edid_mode ---- amlogic old edid, 4k*2k unsupported\n");
	printk("**************************************************************\n");
	/* 1024 = 64*16 */
	for (i = 0; i < 16; i++) {
		printk("[%2d] ", i);
		for (j = 0; j < 16; j++) {
			printk("0x%02lx, ", hdmirx_rd_top(HDMIRX_TOP_EDID_OFFSET + (i*16 + j)));
		}
		printk("\n");
	}
}
void timer_state(void)
{
	switch (rx.state) {
		case HDMIRX_HWSTATE_INIT:
			printk("timer state: HDMIRX_HWSTATE_INIT\n");
		break;
		case HDMIRX_HWSTATE_HDMI5V_LOW:
			printk("timer state: HDMIRX_HWSTATE_HDMI5V_LOW\n");
		break;
		case HDMIRX_HWSTATE_TIMINGCHANGE:
			printk("timer state: HDMIRX_HWSTATE_TIMINGCHANGE\n");
		break;
		case HDMIRX_HWSTATE_SIG_UNSTABLE:
			printk("timer state: HDMIRX_HWSTATE_SIG_UNSTABLE\n");
		break;
		case HDMIRX_HWSTATE_SIG_STABLE:
			printk("timer state: HDMIRX_HWSTATE_SIG_STABLE\n");
		break;
		case HDMIRX_HWSTATE_SIG_READY:
			printk("timer state: HDMIRX_HWSTATE_SIG_READY\n");
		break;
	}
}

ssize_t hdmirx_cable_status_buf(char* buf)
{
	return sprintf(buf, "%x\n", (unsigned int)(hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V) >> 20));
}

ssize_t hdmirx_signal_status_buf(char* buf)
{
	return sprintf(buf, "%x\n", rx.no_signal);
}

int hdmirx_debug(const char* buf, int size)
{
	char tmpbuf[128];
	int i = 0;
	unsigned int adr;
	unsigned int value = 0;

	while((buf[i]) && (buf[i] != ',') && (buf[i] != ' ')) {
		tmpbuf[i]=buf[i];
		i++;
	}
	tmpbuf[i] = 0;
	if(strncmp(tmpbuf, "hpd", 3)==0){
        hdmirx_set_hpd(rx.port, tmpbuf[3]=='0'?0:1);
	}
	else if(strncmp(tmpbuf, "cable_status", 12) == 0){
		size = hdmirx_rd_top(HDMIRX_TOP_HPD_PWR5V) >> 20;
		printk("cable_status = %x\n",size);
	}
	else if(strncmp(tmpbuf, "signal_status", 13) == 0){
		size = rx.no_signal;
		printk("signal_status = %d\n",size);
	}
	else if(strncmp(tmpbuf, "input_mode", 10) == 0){
		size = rx.pre_video_params.sw_vic;
		printk("input_mode = %d",size);
	}
	#ifdef CEC_FUNC_ENABLE
	else if(strncmp(tmpbuf, "cec", 3)==0){
		if(tmpbuf[3] == '0') {
			cec_state(0);
		}else if(tmpbuf[3] == '1'){
			cec_state(1);
		}else if(tmpbuf[3] == '2'){
			//clean_cec_message();
		}else if(tmpbuf[3] == '3'){
			dump_cec_message(tmpbuf[4]-'0');
		}else if(tmpbuf[3] == '5'){
			cec_dump_dev_map();
		}
	}
	#endif
	else if(strncmp(tmpbuf, "reset", 5) == 0){
		if(tmpbuf[5] == '0') {
		    printk(" hdmirx hw config \n");
			memcpy(&rx.hdcp, &init_hdcp_data, sizeof(struct hdmi_rx_ctrl_hdcp));
		    hdmirx_hw_config();
		    //hdmi_rx_ctrl_edid_update();
		    //hdmirx_config_video(&rx.video_params);
			//hdmirx_config_audio();
		}else if(tmpbuf[5] == '1') {
		    printk(" hdmirx phy init 8bit\n");
		    hdmirx_phy_init(rx.port, 0);
		}else if(tmpbuf[5] == '4') {
		    printk(" edid update \n");
		    hdmi_rx_ctrl_edid_update();
		}else if(tmpbuf[5] == '2') {
			printk(" hdmirx phy init 10bit\n");
		    hdmirx_phy_init(rx.port, 1);
		}else if(tmpbuf[5] == '3') {
		    printk(" hdmirx phy init 12bit\n");
		    hdmirx_phy_init(rx.port, 2);
		}else if(strncmp(tmpbuf+5, "_on", 3) == 0){
			reset_sw = 1;
			printk("reset on!\n");
		}else if(strncmp(tmpbuf+5, "_off", 4) == 0){
			reset_sw = 0;
			printk(" reset off!\n");
		}
	}else if (strncmp(tmpbuf, "state", 5) == 0){
		dump_state(0xff);
	}else if (strncmp(tmpbuf, "pause", 5) == 0){
		sm_pause = simple_strtoul(tmpbuf+5, NULL, 10);
		printk("%s the state machine\n", sm_pause?"pause":"enable");
	}else if (strncmp(tmpbuf, "reg", 3) == 0){
		dump_reg();
	}else if (strncmp(tmpbuf, "edid", 4) == 0){
		dump_edid_reg();
	}else if (strncmp(tmpbuf, "hdcp123", 7) == 0){
		dump_hdcp_data();
	}else if (strncmp(tmpbuf, "loadkey", 7) == 0){
		printk("load hdcp key\n");
		memcpy(&rx.hdcp, &init_hdcp_data, sizeof(struct hdmi_rx_ctrl_hdcp));
		hdmirx_hw_config();
	}else if (strncmp(tmpbuf, "timer_state", 11) == 0){
		timer_state();
	}else if (strncmp(tmpbuf, "debug1", 6) == 0){

	}else if (strncmp(tmpbuf, "debug0", 6) == 0){

	}else if (strncmp(tmpbuf, "debug2", 6) == 0){

	}else if (strncmp(tmpbuf, "pinmux_on", strlen("pinmux_on")) == 0){
	     //hdmirx_set_pinmux();
		 printk("[hdmi]%s:hdmi pinmux is on no use \n",__func__);
	}else if (strncmp(tmpbuf, "clock", 5) == 0){
		value = simple_strtoul(tmpbuf + 5, NULL, 10);
		printk("clock[%d] = %d\n", value, hdmirx_get_clock(value));
	}else if (strncmp(tmpbuf, "sample_rate", 11) == 0){
			//nothing
	}else if (strncmp(tmpbuf, "prbs", 4) == 0) {
	  //turn_on_prbs_mode(simple_strtoul(tmpbuf+4, NULL, 10));
	}else if (tmpbuf[0] == 'w'){
		adr = simple_strtoul(tmpbuf + 2, NULL, 16);
		value = simple_strtoul(buf + i + 1, NULL, 16);
		if(buf[1] == 'h'){
    		adr = simple_strtoul(tmpbuf + 3, NULL, 16);
			if(buf[2] == 't'){
		    	hdmirx_wr_top(adr, value);
				pr_info("write %x to hdmirx TOP reg[%x]\n",value,adr);
			}else if (buf[2] == 'd'){
		    	hdmirx_wr_dwc(adr, value);
				pr_info("write %x to hdmirx DWC reg[%x]\n",value,adr);
		    }else if(buf[2] == 'p'){
		    	hdmirx_wr_phy(adr, value);
				pr_info("write %x to hdmirx PHY reg[%x]\n",value,adr);
		    }
		}else if (buf[1] == 'c'){
			WRITE_MPEG_REG(adr, value);
			pr_info("write %x to CBUS reg[%x]\n",value,adr);
		}else if (buf[1] == 'p'){
			WRITE_APB_REG(adr, value);
			pr_info("write %x to APB reg[%x]\n",value,adr);
		}else if (buf[1] == 'l'){
			WRITE_MPEG_REG(MDB_CTRL, 2);
			WRITE_MPEG_REG(MDB_ADDR_REG, adr);
			WRITE_MPEG_REG(MDB_DATA_REG, value);
			pr_info("write %x to LMEM[%x]\n",value,adr);
		}else if(buf[1] == 'r'){
			WRITE_MPEG_REG(MDB_CTRL, 1);
			WRITE_MPEG_REG(MDB_ADDR_REG, adr);
			WRITE_MPEG_REG(MDB_DATA_REG, value);
			pr_info("write %x to amrisc reg [%x]\n",value,adr);
		}
	}else if(tmpbuf[0] == 'r'){
		adr = simple_strtoul(tmpbuf + 2, NULL, 16);
		if(buf[1] == 'h'){
		  adr = simple_strtoul(tmpbuf + 3, NULL, 16);
			if(buf[2] == 't'){
				value = hdmirx_rd_top(adr);
				pr_info("hdmirx TOP reg[%x]=%x\n",adr, value);
			}else if(buf[2] == 'd'){
				value = hdmirx_rd_dwc(adr);
				pr_info("hdmirx DWC reg[%x]=%x\n",adr, value);
			}else if(buf[2] == 'p'){
				value = hdmirx_rd_phy(adr);
				pr_info("hdmirx PHY reg[%x]=%x\n",adr, value);
			}
		}
		else if(buf[1] == 'c'){
		    value = READ_MPEG_REG(adr);
		    pr_info("CBUS reg[%x]=%x\n", adr, value);
		}else if(buf[1] == 'p'){
		    value = READ_APB_REG(adr);
		    pr_info("APB reg[%x]=%x\n", adr, value);
		}else if(buf[1] == 'l'){
		    WRITE_MPEG_REG(MDB_CTRL, 2);
		    WRITE_MPEG_REG(MDB_ADDR_REG, adr);
		    value = READ_MPEG_REG(MDB_DATA_REG);
		    pr_info("LMEM[%x]=%x\n", adr, value);
		}else if(buf[1]=='r'){
		    WRITE_MPEG_REG(MDB_CTRL, 1);
		    WRITE_MPEG_REG(MDB_ADDR_REG, adr);
		    value = READ_MPEG_REG(MDB_DATA_REG);
		    pr_info("amrisc reg[%x]=%x\n", adr, value);
		}
	}else if(tmpbuf[0] == 'v'){
		printk("------------------\n");
		printk("MESON_CPU_TYPE = %x\n",MESON_CPU_TYPE);
		printk("Hdmirx driver version: %s\n", HDMIRX_VER);
		printk("------------------\n");
	}
	return 0;
}

void to_init_state(void)
{
    if(sm_pause){
        return;
    }
}

#if 0
void hdmirx_hw_init2(void)
{
  if(sm_pause){
    return;
  }

    memset(&rx, 0, sizeof(struct rx));
    memset(&rx.pre_video_params, 0, sizeof(struct hdmi_rx_ctrl_video));
    memcpy(&rx.hdcp, &init_hdcp_data, sizeof(struct hdmi_rx_ctrl_hdcp));

    rx.phy.cfg_clk = cfg_clk;
    rx.phy.lock_thres = lock_thres;
    rx.phy.fast_switching = fast_switching;
    rx.phy.fsm_enhancement = fsm_enhancement;
    rx.phy.port_select_ovr_en = port_select_ovr_en;
    rx.phy.phy_cmu_config_force_val = phy_cmu_config_force_val;
    rx.phy.phy_system_config_force_val = phy_system_config_force_val;
    rx.ctrl.md_clk = 24000;
    rx.ctrl.tmds_clk = 0;
    rx.ctrl.tmds_clk2 = 0;
    rx.ctrl.acr_mode = acr_mode;



    hdmirx_set_pinmux();
    //hdmirx_set_hpd(rx.port, 0);

    hdmirx_print("%s %d\n", __func__, rx.port);

}
#endif

/***********************
    hdmirx_hw_init
    hdmirx_hw_uninit
    hdmirx_hw_enable
    hdmirx_hw_disable
    hdmirx_irq_init
*************************/
void hdmirx_hw_init(tvin_port_t port)
{
  if(sm_pause){
    return;
  }

    memset(&rx, 0, sizeof(struct rx));
    //memset(rx.pow5v_state, 0, sizeof(rx.pow5v_state));
    memset(&rx.pre_video_params, 0, sizeof(struct hdmi_rx_ctrl_video));

  	memcpy(&rx.hdcp, &init_hdcp_data, sizeof(struct hdmi_rx_ctrl_hdcp));


    rx.phy.cfg_clk = cfg_clk;
    rx.phy.lock_thres = lock_thres;
    rx.phy.fast_switching = fast_switching;
    rx.phy.fsm_enhancement = fsm_enhancement;
    rx.phy.port_select_ovr_en = port_select_ovr_en;
    rx.phy.phy_cmu_config_force_val = phy_cmu_config_force_val;
    rx.phy.phy_system_config_force_val = phy_system_config_force_val;
    rx.ctrl.md_clk = 24000;
    rx.ctrl.tmds_clk = 0;
    rx.ctrl.tmds_clk2 = 0;
    rx.ctrl.acr_mode = acr_mode;

    rx.port = (port_map>>((port - TVIN_PORT_HDMI0)<<2))&0xf;

	rx.portA_pow5v_state = 0;
	rx.portB_pow5v_state = 0;
	rx.portC_pow5v_state = 0;
	rx.portA_pow5v_state_pre = 0;
	rx.portB_pow5v_state_pre = 0;
	rx.portC_pow5v_state_pre = 0;
    //hdmirx_set_pinmux();
	//if(multi_port_edid_enable){
		//hdmirx_hw_config();
		//hdmi_rx_ctrl_edid_update();
	//}
	//hdmirx_set_pinmux();
	//hdmirx_default_hpd(1);
	//mdelay(30);
    hdmirx_print("%s %d\n", __func__, rx.port);

}


void hdmirx_hw_uninit(void)
{
    if(sm_pause){
        return;
    }
    /* set all hpd low  */
    //WRITE_CBUS_REG(PREG_PAD_GPIO5_O, READ_CBUS_REG(PREG_PAD_GPIO5_O) |
              //((1<<1)|(1<<5)|(1<<9)|(1<<13)));

	if(!multi_port_edid_enable){
		hdmirx_set_hpd(rx.port, 0);
	}

#ifndef CEC_FUNC_ENABLE
    hdmirx_wr_top(HDMIRX_TOP_INTR_MASKN, 0);
    hdmirx_interrupts_cfg(false);
#endif
    audio_status_init();

    rx.ctrl.status = 0;
    rx.ctrl.tmds_clk = 0;
    //ctx->bsp_reset(true);

    hdmirx_phy_reset(true);
    hdmirx_phy_pddq(1);
}

void hdmirx_hw_enable(void)
{
	//hdmirx_set_pinmux();
	hdmirx_hw_probe();
	//mdelay(150);
	//hdmirx_default_hpd(1);
}

void hdmirx_hw_disable(unsigned char flag)
{
}


void hdmirx_default_hpd(bool high)
{
#if(MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV)
    if (!high)
        WRITE_CBUS_REG(PREG_PAD_GPIO5_O, READ_CBUS_REG(PREG_PAD_GPIO5_O) |
                ((1<<5)|(1<<9)|(1<<13)));
    else
        WRITE_CBUS_REG(PREG_PAD_GPIO5_O, READ_CBUS_REG(PREG_PAD_GPIO5_O) &
                (~((1<<5)|(1<<9)|(1<<13))));
#endif
#if(MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9TV)
	if (!high)
        WRITE_CBUS_REG(PREG_PAD_GPIO0_O, READ_CBUS_REG(PREG_PAD_GPIO0_O) |
                ((1<<5)|(1<<9)|(1<<13)));
    else
        WRITE_CBUS_REG(PREG_PAD_GPIO0_O, READ_CBUS_REG(PREG_PAD_GPIO0_O) &
                (~((1<<5)|(1<<9)|(1<<13))));
#endif
#if(MESON_CPU_TYPE == MESON_CPU_TYPE_MESONG9BB)
	if (high)
        WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) |
                ((1<<5)|(1<<9)|(1<<13)));
    else
        WRITE_CBUS_REG(PREG_PAD_GPIO0_EN_N, READ_CBUS_REG(PREG_PAD_GPIO0_EN_N) &
                (~((1<<5)|(1<<9)|(1<<13))));

#endif

}

void hdmirx_irq_init(void)
{
    if(request_irq(INT_HDMI_RX, &irq_handler, IRQF_SHARED, "hdmirx", (void *)&rx)){
    	hdmirx_print(__func__, "RX IRQ request");
    }
}

