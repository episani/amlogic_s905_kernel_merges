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
#include <linux/spinlock.h>
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
#include "hdmirx_cec.h"

#define CEC_ONE_TOUCH_PLAY_FUNC_SUPPORT				1	// TV, CEC switches
#define CEC_ROUTING_CONTROL_FUNC_SUPPORT			1	// TV, CEC switches
#define CEC_STANDBY_FUNC_SUPPORT					1	// All
#define CEC_ONE_TOUCH_RECORD_FUNC_SUPPORT			0 	// Recording devices
#define CEC_TIMER_PROGRAMMER_FUNC_SUPPORT			0 	// optional
#define CEC_SYSTEM_INFORMATION_FUNC_SUPPORT			1 	// All
#define CEC_DECK_CONTROL_FUNC_SUPPORT				0	// optional
#define CEC_TUNER_CONTROL_FUNC_SUPPORT				0	// optional
#define CEC_VENDOR_SPECIFIC_FUNC_SUPPORT			0 	// All
#define CEC_OSD_DISPLAY_FUNC_SUPPORT				0	// optional
#define CEC_DEVICE_OSD_NAME_TRANSFER_FUNC_SUPPORT	1	// optional
#define CEC_DEVICE_MENU_CONTROL_FUNC_SUPPORT		1	// optional
#define CEC_REMOTE_CONTROL_FUNC_PASSTHROUGH_SUPPORT	0	// optional	IR pass through
#define CEC_POWER_STATUS_FUNC_SUPPORT				1	// All except CEC switches
#define CEC_GENERAL_PROTOCAL_FUNC_SUPPORT			1 	// All
#define CEC_SYSTEM_AUDIO_CONTROL_FUNC_SUPPORT		1	// optional ARC authentication
#define CEC_AUDIO_RETURN_CHANNEL_FUNC_SUPPORT		1	// optiongal ARC authentication


#define LOGIC_ADDR_EXIST	0x88
#define LOGIC_ADDR_NULL		0x00
struct _cec_dev_map cec_map[16];

static int cec_enable = 1;
static int arc_enable = 1;
static int cec_log = 0;  // 1: error log  2: info log  3 all
static enum _cec_map_status cec_map_status = E_CEC_MAP_UPDATE_START;
static int cec_retry_cnt = 0;
static int cec_dev_index = 1;

static bool get_ack_flag = false;   //just used to ping dev on the cec line
static int cec_wait_for_ack_cnt = 30;
static int cec_ctrl_wait_times = 3;
static int ping_dev_cnt = 0;	// if = 0, start ping dev (1--14)
								// if = 140, ping dev completed, statt post msg get phyaddr and name
								// if = 141, post msg over,do nothing

_cec_msg_queue  rx_queue;
_cec_msg_queue  tx_queue;

int cec_handler(bool get_msg, bool get_ack)
{
	//int wait_cnt = 0;

	if(hdmirx_rd_dwc(HDMIRX_DWC_CEC_CTRL) != 0x00000002)
	{
		//mdelay(1);
		//if(wait_cnt++ > cec_ctrl_wait_times){
		//clear current lock state, messge is wrong
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_LOCK, 0);
		if(cec_log)
			printk("\n\n cec error --- clr current tx info\n\n");
		return 0;
		//}
	}
	if(get_msg){
		rx_queue.cec_msg[rx_queue.wr_index].msg_len = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_CNT);
		rx_queue.cec_msg[rx_queue.wr_index].addr = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA0);
		rx_queue.cec_msg[rx_queue.wr_index].cmd = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA1);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[0] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA2);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[1] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA3);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[2] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA4);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[3] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA5);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[4] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA6);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[5] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA7);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[6] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA8);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[7] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA9);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[8] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA10);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[9] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA11);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[10] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA12);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[11] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA13);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[12] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA14);
		rx_queue.cec_msg[rx_queue.wr_index].msg_data[13] = hdmirx_rd_dwc(HDMIRX_DWC_CEC_RX_DATA15);
		rx_queue.wr_index = (rx_queue.wr_index+1) % CEC_MSG_QUEUE_SIZE;
		//clr CEC lock bit
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_LOCK, 0);
	}
	if(get_ack){
		get_ack_flag = true;
	}
	return 0;
}

#if (MESON_CPU_TYPE < MESON_CPU_TYPE_MESONG9TV)
#define HHI_CLK_32K_CNTL         0x105a
#define Wr_reg_bits(reg, val, start, len) \
  WRITE_MPEG_REG(reg, (READ_MPEG_REG(reg) & ~(((1L<<(len))-1)<<(start)))|((unsigned int)(val) << (start)))
#endif
int cec_init(void)
{
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESONG9TV)
	unsigned int data32;
	//set cec clk 32768k
    data32  = 0;
    data32 |= 0         << 16;  // [17:16] clk_sel: 0=oscin; 1=slow_oscin; 2=fclk_div3; 3=fclk_div5.
    data32 |= 1         << 15;  // [   15] clk_en
    data32 |= (732-1)   << 0;   // [13: 0] clk_div
    WRITE_MPEG_REG(HHI_32K_CLK_CNTL,    data32);

#else
	Wr_reg_bits(HHI_CLK_32K_CNTL, 1, 16, 2);
	Wr_reg_bits(HHI_CLK_32K_CNTL, 1, 18, 1);
#endif
	//cec_clk_en
	hdmirx_wr_top(HDMIRX_TOP_CLK_CNTL,hdmirx_rd_top(HDMIRX_TOP_CLK_CNTL)| _BIT(2));
	//set logic addr
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_ADDR_L, 0x00000001);
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_ADDR_H, 0x00000000);

	//enable cec eom irq & ack irq
	hdmirx_wr_dwc(HDMIRX_DWC_AUD_CLK_IEN_SET, (3<<16));
	return 0;
}

void cec_state(bool cec_rx)
{
    printk("\n\n ****************CEC***************");
    printk("\nCEC_CTRL = %2x",hdmirx_rd_dwc(0x1f00)&0xff);
	printk("\nCEC_MASK = %2x",hdmirx_rd_dwc(0x1f08)&0xff);
	printk("\nCEC_ADDR_L = %2x",hdmirx_rd_dwc(0x1f14)&0xff);
	printk("\nCEC_ADDR_H = %2x",hdmirx_rd_dwc(0x1f18)&0xff);
	printk("\nCEC_TX_CNT = %2x",hdmirx_rd_dwc(0x1f1c)&0xff);
	printk("\nCEC_RX_CNT = %2x",hdmirx_rd_dwc(0x1f20)&0xff);
	printk("\nCEC_LOCK = %2x",hdmirx_rd_dwc(0x1fc0)&0xff);
	printk("\nCEC_WKUPCTRL = %2x",hdmirx_rd_dwc(0x1fc4)&0xff);

    if(cec_rx){
	printk("\nRX_DATA0 = %2x",hdmirx_rd_dwc(0x1f80)&0xff);
	printk("\nRX_DATA1 = %2x",hdmirx_rd_dwc(0x1f84)&0xff);
	printk("\nRX_DATA2 = %2x",hdmirx_rd_dwc(0x1f88)&0xff);
	printk("\nRX_DATA3 = %2x",hdmirx_rd_dwc(0x1f8C)&0xff);
	printk("\nRX_DATA4 = %2x",hdmirx_rd_dwc(0x1f90)&0xff);
	printk("\nRX_DATA5 = %2x",hdmirx_rd_dwc(0x1f94)&0xff);
	printk("\nRX_DATA6 = %2x",hdmirx_rd_dwc(0x1f98)&0xff);
	printk("\nRX_DATA7 = %2x",hdmirx_rd_dwc(0x1f9C)&0xff);
	printk("\nRX_DATA8 = %2x",hdmirx_rd_dwc(0x1fA0)&0xff);
	printk("\nRX_DATA9 = %2x",hdmirx_rd_dwc(0x1fA4)&0xff);
	printk("\nRX_DATA10 = %2x",hdmirx_rd_dwc(0x1fA8)&0xff);
	printk("\nRX_DATA11 = %2x",hdmirx_rd_dwc(0x1fAC)&0xff);
	printk("\nRX_DATA12 = %2x",hdmirx_rd_dwc(0x1fB0)&0xff);
	printk("\nRX_DATA13 = %2x",hdmirx_rd_dwc(0x1fB4)&0xff);
	printk("\nRX_DATA14 = %2x",hdmirx_rd_dwc(0x1fB8)&0xff);
	printk("\nRX_DATA15 = %2x",hdmirx_rd_dwc(0x1fBC)&0xff);
    }else{
    printk("\nTX_DATA0 = %2x",hdmirx_rd_dwc(0x1f40)&0xff);
	printk("\nTX_DATA1 = %2x",hdmirx_rd_dwc(0x1f44)&0xff);
	printk("\nTX_DATA2 = %2x",hdmirx_rd_dwc(0x1f48)&0xff);
	printk("\nTX_DATA3 = %2x",hdmirx_rd_dwc(0x1f4C)&0xff);
	printk("\nTX_DATA4 = %2x",hdmirx_rd_dwc(0x1f50)&0xff);
	printk("\nTX_DATA5 = %2x",hdmirx_rd_dwc(0x1f54)&0xff);
	printk("\nTX_DATA6 = %2x",hdmirx_rd_dwc(0x1f58)&0xff);
	printk("\nTX_DATA7 = %2x",hdmirx_rd_dwc(0x1f5C)&0xff);
	printk("\nTX_DATA8 = %2x",hdmirx_rd_dwc(0x1f60)&0xff);
	printk("\nTX_DATA9 = %2x",hdmirx_rd_dwc(0x1f64)&0xff);
	printk("\nTX_DATA10 = %2x",hdmirx_rd_dwc(0x1f68)&0xff);
	printk("\nTX_DATA11 = %2x",hdmirx_rd_dwc(0x1f6C)&0xff);
	printk("\nTX_DATA12 = %2x",hdmirx_rd_dwc(0x1f70)&0xff);
	printk("\nTX_DATA13 = %2x",hdmirx_rd_dwc(0x1f74)&0xff);
	printk("\nTX_DATA14 = %2x",hdmirx_rd_dwc(0x1f78)&0xff);
	printk("\nTX_DATA15 = %2x",hdmirx_rd_dwc(0x1f7C)&0xff);
    }
    printk("\n****************CEC***************\n\n");
}


void dump_cec_message(int all)
{
	#if 0
	int i = 0;
	for(i=0; i<CEC_MSG_QUEUE_SIZE; i++){
		if((queue.msg[i].info == MSG_RX) && (all==1)){
			printk("\n------ buffer %d start --------\n",i);
			printk("      %d      ",queue.msg[i].info);
			printk("len=%d,  dev_addr=%x,  opcode=%x\n",queue.msg[i].msg_len,queue.msg[i].addr,queue.msg[i].opcode);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[0],queue.msg[i].msg_data[1],queue.msg[i].msg_data[2],queue.msg[i].msg_data[3]);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[4],queue.msg[i].msg_data[5],queue.msg[i].msg_data[6],queue.msg[i].msg_data[7]);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[8],queue.msg[i].msg_data[9],queue.msg[i].msg_data[10],queue.msg[i].msg_data[11]);
			printk("\n------ buffer %d end --------\n",i);
		}else if((queue.msg[i].info == MSG_TX) && (all==2)){
			printk("\n------ buffer %d start --------\n",i);
			printk("      %d      ",queue.msg[i].info);
			printk("len=%d,  dev_addr=%x,  opcode=%x\n",queue.msg[i].msg_len,queue.msg[i].addr,queue.msg[i].opcode);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[0],queue.msg[i].msg_data[1],queue.msg[i].msg_data[2],queue.msg[i].msg_data[3]);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[4],queue.msg[i].msg_data[5],queue.msg[i].msg_data[6],queue.msg[i].msg_data[7]);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[8],queue.msg[i].msg_data[9],queue.msg[i].msg_data[10],queue.msg[i].msg_data[11]);
			printk("\n------ buffer %d end --------\n",i);
		}if(all==3){
			printk("\n------ buffer %d start --------\n",i);
			printk("      %d      ",queue.msg[i].info);
			printk("len=%d,  dev_addr=%x,  opcode=%x\n",queue.msg[i].msg_len,queue.msg[i].addr,queue.msg[i].opcode);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[0],queue.msg[i].msg_data[1],queue.msg[i].msg_data[2],queue.msg[i].msg_data[3]);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[4],queue.msg[i].msg_data[5],queue.msg[i].msg_data[6],queue.msg[i].msg_data[7]);
			printk("%x,  %x,  %x,  %x\n",queue.msg[i].msg_data[8],queue.msg[i].msg_data[9],queue.msg[i].msg_data[10],queue.msg[i].msg_data[11]);
			printk("\n------ buffer %d end --------\n",i);
		}
	}
	#endif
}
void cec_dump_dev_map(void)
{
	int i = 0;
	for (i=1; i<=E_LA_MAX; i++){
		if(cec_map[i].logic_addr == LOGIC_ADDR_NULL)
			continue;
		printk("\n ************************\n");
		printk("logicaddr=%d,  phyaddr=%x,  type=%d\n",i,cec_map[i].phy_addr,cec_map[i].cec_dev_type);
		printk("devname = %s\n",cec_map[i].cec_dev_name);
		printk("************************\n");
	}
}
/*
static bool cec_dev_is_exist(_cec_dev_logic_addr_ logic_addr)
{
	if(cec_map[logic_addr].cec_dev_logicaddr == LOGIC_ADDR_EXIST)
		return true;
	else
		return false;
}
*/
void cec_add_dev(_cec_logic_addr logic_addr,unsigned int physical_addr,_cec_dev_type dev_type)
{
	cec_map[logic_addr].logic_addr		= LOGIC_ADDR_EXIST;
	cec_map[logic_addr].phy_addr		= physical_addr;
	cec_map[logic_addr].cec_dev_type 	= dev_type;
	cec_map[logic_addr].cec_dev_name[0] = '\0';
}


bool cec_ping_logic_addr(void)
{
	if(get_ack_flag){
		cec_map[cec_dev_index].logic_addr = LOGIC_ADDR_EXIST;
		cec_dev_index++;
		get_ack_flag = false;
		if(cec_log){
			printk("ping dev %d success\n",cec_dev_index);
		}
	}
	if(cec_retry_cnt >= 5){
		cec_map[cec_dev_index].logic_addr = LOGIC_ADDR_NULL;
		cec_retry_cnt = 0;
		cec_dev_index++;
	}
	if(cec_dev_index < 15) {
	    hdmirx_wr_dwc(HDMIRX_DWC_CEC_CTRL, 0x00000002);
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA0, cec_dev_index);
		if(cec_log)
			printk("\nPing dev %d\n",cec_dev_index);
		cec_retry_cnt++;
		get_ack_flag = false;
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_CNT, 0x00000001);
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_CTRL, 0x00000003);
		return false;
	}
	cec_dev_index = 1;
	return true;
}
bool cec_update_cec_parameters(_cec_cmd cmd)
{
	if(get_ack_flag){
		cec_dev_index++;
		cec_retry_cnt = 0;
		get_ack_flag = false;
	}
	if(cec_retry_cnt++ >= 5){
		cec_dev_index++;
		cec_retry_cnt = 0;
	}
	if(cec_dev_index >= 15)
		return true;
	if(cec_map[cec_dev_index].logic_addr == LOGIC_ADDR_EXIST)
		cec_post_msg(cec_dev_index, cmd, 0, 2);
	else{
		cec_dev_index++;
		cec_retry_cnt = 0;
	}
	return false;
}

void cec_update_cec_map(void)
{
	static int wait_cnt = 0;
	if(cec_map_status == E_CEC_MAP_UPDATE_COMPLETED)
		return;
	if(wait_cnt++ < 4)
		return;
	wait_cnt = 0;
	switch(cec_map_status){
	case E_CEC_MAP_UPDATE_START:
		cec_map_status = E_CEC_MAP_POLLLING;
		break;
	case E_CEC_MAP_POLLLING:
		if(cec_ping_logic_addr()){
			cec_map_status = E_CEC_MAP_GET_VENDOR_ID;
			if(cec_log)
				printk("-> E_CEC_MAP_GET_VENDOR_ID\n");
			cec_dev_index = 1;
		}
		break;
	case E_CEC_MAP_GET_VENDOR_ID:
		if(cec_update_cec_parameters(E_MSG_GIVE_DEVICE_VENDOR_ID)){
			cec_map_status = E_CEC_MAP_GET_PHYSICAL_ADDR;
			if(cec_log)
				printk("-> E_CEC_MAP_GET_PHYSICAL_ADDR\n");
			cec_dev_index = 1;
		}
		break;
	case E_CEC_MAP_GET_PHYSICAL_ADDR:
		if(cec_update_cec_parameters(E_MSG_REPORT_PHYSICAL_ADDRESS)){
			cec_map_status = E_CEC_MAP_GET_OSD_NAME;
			if(cec_log)
				printk("-> E_CEC_MAP_GET_OSD_NAME\n");
			cec_dev_index = 1;
		}
		break;
	case E_CEC_MAP_GET_OSD_NAME:
		if(cec_update_cec_parameters(E_MSG_OSDNT_GIVE_OSD_NAME)){
			cec_map_status = E_CEC_MAP_GET_CEC_VERSION;
			if(cec_log)
				printk("-> E_CEC_MAP_GET_CEC_VERSION\n");
			cec_dev_index = 1;
		}
		break;
	case E_CEC_MAP_GET_CEC_VERSION:
		if(cec_update_cec_parameters(E_MSG_CEC_VERSION)){
			cec_map_status = E_CEC_MAP_UPDATE_COMPLETED;
			if(cec_log)
				printk("-> E_CEC_MAP_UPDATE_COMPLETED\n");
			cec_dev_index = 1;
		}
		break;
	default:
		break;

	}
}
void cec_post_givephyaddr(_cec_logic_addr logic_addr)
{
	#if 0
	int i=0;
	for(i=0; i<CEC_MSG_QUEUE_SIZE; i++){
		if(queue.msg[i].info == MSG_NULL){
			break;
		}
	}
	if((i>=CEC_MSG_QUEUE_SIZE)&&(cec_log&(1<<0)))
		printk("\n tx overflow %s \n",__FUNCTION__);
	queue.msg[i].info = MSG_TX;
	queue.msg[i].msg_len = 2;
	queue.msg[i].opcode = E_MSG_GIVE_PHYSICAL_ADDRESS;
	queue.msg[i].addr = logic_addr;
	if(cec_log&(1<<1))
		printk("\n tx %d post msg %s \n",logic_addr,__FUNCTION__);
	#endif
}

//--------------------------- IO_CTL ---------------------------------------//
void hdmirx_cec_fun_onoff(bool enable)
{
	if(enable){
		cec_enable = 1;
		//cec eom irq enable
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_CLK_IEN_SET, (1<<17));
	}else{
		cec_enable = 0;
		//cec eom irq enable
		hdmirx_wr_dwc(HDMIRX_DWC_AUD_CLK_IEN_SET, (0<<17));
	}
	if(cec_log)
		printk("cec_status: %d\n",enable);

}
void hdmirx_cec_arc_onoff(bool enable)
{
	if(enable)
		arc_enable = 1;
	else
		arc_enable = 0;
	if(cec_log)
		printk("arc_status: %d\n",enable);
}

void cec_post_msg_to_buf(struct _cec_msg *msg)
{
	memcpy(&tx_queue.cec_msg[tx_queue.wr_index], msg, sizeof(_cec_msg));
	tx_queue.wr_index = (tx_queue.wr_index+1) % CEC_MSG_QUEUE_SIZE;
}

void cec_post_msg(_cec_logic_addr addr, _cec_cmd cmd, int* data, int length)
{
	int i=0;
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_CTRL, 0x00000002);
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA0, addr);
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA1, cmd);
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_CNT, length);
	if(length <=2){
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_CTRL, 0x00000003);
	}else{
		for(; i<length-2; i++){
			hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA2+i*4, data[i]);
		}
	}
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_CTRL, 0x00000003);
}

int hdmirx_get_cec_msg_cnt(void)
{
	if(rx_queue.wr_index >= rx_queue.rd_index)
		return (rx_queue.wr_index - rx_queue.rd_index);
	else
		return (CEC_MSG_QUEUE_SIZE + rx_queue.wr_index - rx_queue.rd_index);
}

void hdmirx_cec_clear_rx_buffer(void)
{
	rx_queue.rd_index = rx_queue.wr_index = 0;
}

struct _cec_msg *hdmirx_get_rx_msg(void)
{
	struct _cec_msg *msg;
	if(rx_queue.wr_index == rx_queue.rd_index){
		printk("error,no rx msg,ioctl_get\n");
		return 0;
	}
	msg = &rx_queue.cec_msg[rx_queue.rd_index];
	//memcpy(msg ,rx_queue.cec_msg[rx_queue.rd_index], sizeof(struct _cec_msg_));

	rx_queue.rd_index = (rx_queue.rd_index+1) % CEC_MSG_QUEUE_SIZE;
	return msg;
}
void cec_post_giveosdname(_cec_logic_addr logic_addr)
{
	#if 0
	int i=0;
	for(i=0; i<CEC_MSG_QUEUE_SIZE; i++){
		if(queue.msg[i].info == MSG_NULL){
			break;
		}
	}
	if((i>=CEC_MSG_QUEUE_SIZE)&&(cec_log&(1<<0)))
		printk("\n tx overflow %s \n",__FUNCTION__);
	queue.msg[i].info = MSG_TX;
	queue.msg[i].msg_len = 2;
	queue.msg[i].opcode = E_MSG_OSDNT_GIVE_OSD_NAME;
	queue.msg[i].addr = logic_addr;
	if(cec_log&(1<<1))
		printk("\n tx %d post msg %s \n",logic_addr,__FUNCTION__);
	#endif
}
void cec_dbg_post_cmd(int command,int value)
{
	//int i=0;
	tx_queue.cec_msg[tx_queue.wr_index].msg_len = 3;
	tx_queue.cec_msg[tx_queue.wr_index].cmd = command;
	tx_queue.cec_msg[tx_queue.wr_index].addr = 0x04;
	tx_queue.cec_msg[tx_queue.wr_index].msg_data[0] = value;
	tx_queue.wr_index = (tx_queue.wr_index+1) % CEC_MSG_QUEUE_SIZE;
	if(cec_log&(1<<1))
		printk("\n tx post msg command %x vaule %x \n",command,value);

}

int hdmirx_cec_tx_monitor(void)
{
	//int i = 0;
	if(cec_map_status <= E_CEC_MAP_POLLLING)
		return 0;
	if(hdmirx_rd_dwc(HDMIRX_DWC_CEC_CTRL) & (1<<0))
		return 0;
	if(tx_queue.rd_index == tx_queue.wr_index)
		return 0;
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_CTRL, 0x00000002);
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA0, tx_queue.cec_msg[tx_queue.rd_index].addr);
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA1, tx_queue.cec_msg[tx_queue.rd_index].cmd);
	if(cec_log&(1<<1))
		printk("\n tx handle opcpde %x\n",tx_queue.cec_msg[tx_queue.rd_index].cmd);
	hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_CNT, tx_queue.cec_msg[tx_queue.rd_index].msg_len);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>2)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA2, tx_queue.cec_msg[tx_queue.rd_index].msg_data[0]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>3)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA3, tx_queue.cec_msg[tx_queue.rd_index].msg_data[1]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>4)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA4, tx_queue.cec_msg[tx_queue.rd_index].msg_data[2]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>5)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA5, tx_queue.cec_msg[tx_queue.rd_index].msg_data[3]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>6)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA6, tx_queue.cec_msg[tx_queue.rd_index].msg_data[4]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>7)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA7, tx_queue.cec_msg[tx_queue.rd_index].msg_data[5]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>8)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA8, tx_queue.cec_msg[tx_queue.rd_index].msg_data[6]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>9)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA9, tx_queue.cec_msg[tx_queue.rd_index].msg_data[7]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>10)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA10, tx_queue.cec_msg[tx_queue.rd_index].msg_data[8]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>11)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA11, tx_queue.cec_msg[tx_queue.rd_index].msg_data[9]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>12)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA12, tx_queue.cec_msg[tx_queue.rd_index].msg_data[10]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>13)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA13, tx_queue.cec_msg[tx_queue.rd_index].msg_data[11]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>14)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA14, tx_queue.cec_msg[tx_queue.rd_index].msg_data[12]);
	if(tx_queue.cec_msg[tx_queue.rd_index].msg_len>15)
		hdmirx_wr_dwc(HDMIRX_DWC_CEC_TX_DATA15, tx_queue.cec_msg[tx_queue.rd_index].msg_data[13]);

	hdmirx_wr_dwc(HDMIRX_DWC_CEC_CTRL, 0x00000003);
	tx_queue.rd_index = (tx_queue.rd_index+1) % CEC_MSG_QUEUE_SIZE;

	return 0;
}

int hdmirx_cec_rx_monitor(void)
{
	//int i = 0;
	//static int ping_dev_cnt = 0;
	//int initiator_addr = rx_queue.cec_msg[queue.head].addr >> 4;
	//int dest_addr = rx_queue.cec_msg[queue.head].addr & 0xf;
	/*
	if(cec_ping_logic_addr(ping_dev_cnt) == PING_DEV_SUCCESS){
		ping_dev_cnt = (ping_dev_cnt/10 + 1)*10;
	}

	if(ping_dev_cnt < 140)
		ping_dev_cnt++;
	else if(ping_dev_cnt == 140){
		for(i=1; i<15; i++){
			if(cec_dev_is_exist(i)){
				cec_post_givephyaddr(i);
				cec_post_giveosdname(i);
			}
		}
		ping_dev_cnt++;
	}
	*/
	if(cec_map_status <= E_CEC_MAP_POLLLING)
		return 0;
	if(rx_queue.wr_index == rx_queue.rd_index)
		return 0;
	if(cec_log&(1<<1)){
		printk("\n rx msg %x",rx_queue.cec_msg[rx_queue.rd_index].cmd);
		printk("\n %x",rx_queue.cec_msg[rx_queue.rd_index].addr);
		printk("\n %x",rx_queue.cec_msg[rx_queue.rd_index].msg_data[0]);
		printk("\n %x",rx_queue.cec_msg[rx_queue.rd_index].msg_data[1]);
		printk("\n %x",rx_queue.cec_msg[rx_queue.rd_index].msg_data[2]);
	}
	switch(rx_queue.cec_msg[rx_queue.rd_index].cmd){
	case CMD_TCL_WIFI:
	case CMD_TCL_ETHERNET:
	case CMD_TCL_3D:
	case CMD_TCL_PANEL_REVERSE:
	case CMD_VPU_INIT:
	case CMD_VPU_ENABLE:
	case CMD_VPU_BYPASS:
	case CMD_VPU_OUTPUT_MUX:
	case CMD_VPU_TIMING:
	case CMD_VPU_SOURCE:
	case CMD_TCL_BRIDGE_SW_VER:
	//case CMD_G9_MAINCODE_VER:
	case CMD_TCL_DEVICE_ID:
	//case CMD_G9_BOOTCODE_VER:
	case CMD_TCL_CLIENT_TYPE:
	//case CMD_INFO_G9_TO_FBC:
	case CMD_TCL_DEVICE_NUM:
	//case CMD_INFO_FBC_TO_G9:
	case CMD_TCL_ACTIVE_KEY:
	//case CMD_TIME_SYNC:
	case CMD_TCL_ACTIVE_STATUS:
	//case CMD_KEY_TRANSLATION:
	case CMD_DBG_RD_REGISTER_ACCESS:
	case CMD_DBG_RD_MEMORY_ACCESS:
	case CMD_DBG_RD_SPI_ACCESS:
	case CMD_DBG_RD_VPU_MEMORY_ACCESS:
	case CMD_DBG_RD_MEMORY_TRANSFER:
	case CMD_DBG_INPUT_KEY_DOWN:
	case CMD_DBG_INPUT_KEY_UP:
	case CMD_DBG_INPUT_REBOOT:
	case CMD_DBG_WR_REGISTER_ACCESS:
	case CMD_DBG_WR_MEMORY_ACCESS:
	case CMD_DBG_WR_SPI_ACCESS:
	case CMD_DBG_WR_VPU_MEMORY_ACCESS:
	case CMD_DBG_WR_MEMORY_TRANSFER:
	case CMD_NATURE_LIGHT_ONOFF:
	case CMD_USR_BACKLIGHT_ONOFF:
	case CMD_USR_BRIGHTNESS:
	case CMD_USR_CONTRAST:
	case CMD_USR_BACKLIGHT:
	case CMD_USR_SATURATION:
	case CMD_USR_DYNAMIC_CONTRAST:
	case CMD_USR_PICTURE_MODE:
	case CMD_TEST_PATTERN_ONOFF:
	case CMD_TEST_PATTERN_SELECT:
	case CMD_USR_GAMMA:
	case CMD_DEF_SOUND_MODE:
	case CMD_DEF_COLOR_TEMPERATURE:
	case CMD_DEF_BRIGHTNESS:
	case CMD_DEF_CONTRAST:
	case CMD_DEF_COLOR:
	case CMD_DEF_BACKLIGHT:
	case CMD_AUTO_LUMA_ONOFF:
	case CMD_HISTOGRAM:
	case CMD_BLEND:
	case CMD_DEMULA:
	case CMD_COLORSPACE_CONVERSION:
	case CMD_CM2:
	case CMD_DEF_RED_GAIN:
	case CMD_DEF_GREEN_GAIN:
	case CMD_DEF_BLUE_GAIN:
	case CMD_DEF_RED_OFFSET:
	case CMD_DEF_GREEN_OFFSET:
	case CMD_DEF_BLUE_OFFSET:
	case CMD_DEF_PRE_RED_OFFSET:
	case CMD_DEF_PRE_GREEN_OFFSET:
	case CMD_DEF_PRE_BLUE_OFFSET:
	case CMD_WHITEBLANCE:
	case CMD_SET_SW_VERSION:
	case CMD_3D:
	default:
		break;
	}
	rx_queue.rd_index = (rx_queue.rd_index+1) % CEC_MSG_QUEUE_SIZE;
	return 0;
}


MODULE_PARM_DESC(cec_wait_for_ack_cnt, "\n cec_wait_for_ack_cnt \n");
module_param(cec_wait_for_ack_cnt, int, 0664);

MODULE_PARM_DESC(ping_dev_cnt, "\n ping_dev_cnt \n");
module_param(ping_dev_cnt, int, 0664);

MODULE_PARM_DESC(cec_log, "\n cec_log \n");
module_param(cec_log, int, 0664);

MODULE_PARM_DESC(cec_ctrl_wait_times, "\n cec_ctrl_wait_times \n");
module_param(cec_ctrl_wait_times, int, 0664);

MODULE_PARM_DESC(cec_enable, "\n cec_enable \n");
module_param(cec_enable, int, 0664);

MODULE_PARM_DESC(arc_enable, "\n arc_enable \n");
module_param(arc_enable, int, 0664);
