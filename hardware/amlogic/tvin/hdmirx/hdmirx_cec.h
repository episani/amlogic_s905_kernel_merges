/*
 * hdmirx_drv.h for HDMI device driver, and declare IO function,
 * structure, enum, used in TVIN AFE sub-module processing
 *
 * Copyright (C) 2013 AMLOGIC, INC. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the smems of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#ifndef _HDMICEC_H
#define _HDMICEC_H

#define CEC_MSG_QUEUE_SIZE 20

typedef enum _cec_logic_addr
{
    E_LA_TV              =0,
    E_LA_RECORDER1       =1,
    E_LA_RECORDER2       =2,
    E_LA_TUNER1          =3,
    E_LA_PLAYBACK1       =4,
    E_LA_AUDIO_SYS       =5,
    E_LA_TUNER2          =6,
    E_LA_TUNER3          =7,
    E_LA_PLAYBACK2       =8,
    E_LA_RECORER3        =9,
    E_LA_TUNER4          =10,
    E_LA_PLYBACK3        =11,
    RESERVED_1           =12,
    RESERVED_2           =13,
    E_LA_FREE_USE        =14,
    E_LA_UNREGISTERED    =15,
    E_LA_BROADCAST       =15,
    E_LA_MAX = 15,
} _cec_logic_addr;

typedef enum _cec_dev_type
{
    E_DEVICE_TYPE_TV                    =0,
    E_DEVICE_TYPE_RECORDING_DEVICE      =1,
    E_DEVICE_TYPE_RESERVED              =2,
    E_DEVICE_TYPE_TUNER                 =3,
    E_DEVICE_TYPE_PLAYBACK_DEVICE       =4,
    E_DEVICE_TYPE_AUDIO_SYSTEM          =5,
    E_DEVICE_TYPE_PURE_CEC_SWITCH		=6,
    E_DEVICE_TYPE_VIDEO_PROCESSOR		=7
} _cec_dev_type;

typedef enum _cec_cmd
{
//----- One Touch Play ----------------------------
    E_MSG_ACTIVE_SOURCE                         = 0x82,
    E_MSG_IMAGE_VIEW_ON                     	= 0x04,
    E_MSG_TEXT_VIEW_ON                      	= 0x0D,
//----- Routing Control ---------------------------
    //E_MSG_RC_ACTIVE_SOURCE                    = 0x82,
    E_MSG_INACTIVE_SOURCE						= 0x9D,
    E_MSG_REQUEST_ACTIVE_SOURCE					= 0x85,
    E_MSG_ROUTING_CHANGE						= 0x80,
    E_MSG_ROUTING_INFO							= 0x81,
    E_MSG_SET_STREM_PATH						= 0x86,
//----- Standby Command ---------------------------
    E_MSG_STANDBY                               = 0x36,
//----- One Touch Record---------------------------
    E_MSG_RECORD_ON								= 0x09,
    E_MSG_RECORD_OFF							= 0x0B,
    E_MSG_RECORD_STATUS							= 0x0A,
    E_MSG_RECORD_TV_SCREEN						= 0x0F,
//----- Timer programmer -------------------------- CEC1.3a
    E_MSG_CLEAR_ANALOG_TIMER					= 0x33,
    E_MSG_CLEAR_DIGITAL_TIMER					= 0x99,
    E_MSG_CLEAR_EXT_TIMER						= 0xA1,
    E_MSG_SET_ANALOG_TIMER						= 0x34,
    E_MSG_SET_DIGITAL_TIMER						= 0x97,
    E_MSG_SET_EXT_TIMER							= 0xA2,
    E_MSG_SET_TIMER_PROGRAM_TITLE				= 0x67,
    E_MSG_TIMER_CLEARD_STATUS					= 0x43,
    E_MSG_TIMER_STATUS							= 0x35,
//----- System Information ------------------------
    E_MSG_CEC_VERSION                        	= 0x9E,			//1.3a
    E_MSG_GET_CEC_VERSION                   	= 0x9F,			//1.3a
    E_MSG_GIVE_PHYSICAL_ADDRESS              	= 0x83,
    E_MSG_REPORT_PHYSICAL_ADDRESS				= 0x84,
    E_MSG_GET_MENU_LANGUAGE						= 0x91,
    E_MSG_SET_MENU_LANGUAGE						= 0x32,
    //E_MSG_POLLING_MESSAGE						= ?,
    //E_MSG_REC_TYPE_PRESET						= 0x00,			//parameter   ?
    //E_MSG_REC_TYPE_OWNSRC						= 0x01,  		//parameter   ?
//----- Deck Control Feature-----------------------
    E_MSG_DECK_CTRL								= 0x42,
    E_MSG_DECK_STATUS                        	= 0x1B,
    E_MSG_GIVE_DECK_STATUS                   	= 0x1A,
    E_MSG_PLAY									= 0x41,
//----- Tuner Control ------------------------------
    E_MSG_GIVE_TUNER_STATUS                  	= 0x08,
    E_MSG_SEL_ANALOG_SERVICE                 	= 0x92,
    E_MSG_SEL_DIGITAL_SERVICE                	= 0x93,
    E_MSG_TUNER_DEVICE_STATUS                	= 0x07,
    E_MSG_TUNER_STEP_DEC                     	= 0x06,
    E_MSG_TUNER_STEP_INC                     	= 0x05,
//---------Vendor Specific -------------------------
    //E_MSG_CEC_VERSION                      	= 0x9E,       	//1.3a
    //E_MSG_GET_CEC_VERSION                    	= 0x9F,       	//1.3a
    E_MSG_DEVICE_VENDOR_ID                   	= 0x87,
    E_MSG_GIVE_DEVICE_VENDOR_ID              	= 0x8C,
    E_MSG_VENDOR_COMMAND                     	= 0x89,
    E_MSG_VENDOR_COMMAND_WITH_ID             	= 0xA0,      	//1.3a
    E_MSG_VENDOR_RC_BUT_DOWN                 	= 0x8A,
    E_MSG_VENDOR_RC_BUT_UP                   	= 0x8B,
//----- OSD Display --------------------------------
    E_MSG_SET_OSD_STRING                        = 0x64,
//----- Device OSD Name Transfer  -------------------------
    E_MSG_OSDNT_GIVE_OSD_NAME                   = 0x46,
    E_MSG_OSDNT_SET_OSD_NAME                    = 0x47,
//----- Device Menu Control ------------------------
    E_MSG_DMC_MENU_REQUEST                      = 0x8D,
    E_MSG_DMC_MENU_STATUS                       = 0x8E,
    E_MSG_UI_PRESS                              = 0x44,
    E_MSG_UI_RELEASE                            = 0x45,
//----- Remote Control Passthrough ----------------
	//E_MSG_UI_PRESS							= 0x44,
    //E_MSG_UI_RELEASE							= 0x45,
//----- Power Status  ------------------------------
    E_MSG_GIVE_DEVICE_POWER_STATUS           	= 0x8F,
    E_MSG_REPORT_POWER_STATUS                	= 0x90,
//----- General Protocal Message ------------------
    E_MSG_ABORT_MESSAGE                         = 0xFF,			//Abort msg
    E_MSG_FEATURE_ABORT                         = 0x00,			//Feature Abort
//----- System Audio Control ----------------------
    E_MSG_ARC_GIVE_AUDIO_STATUS                 = 0x71,
    E_MSG_ARC_GIVE_SYSTEM_AUDIO_MODE_STATUS     = 0x7D,
    E_MSG_ARC_REPORT_AUDIO_STATUS               = 0x7A,
    E_MSG_ARC_SET_SYSTEM_AUDIO_MODE             = 0x72,
    E_MSG_ARC_SYSTEM_AUDIO_MODE_REQUEST         = 0x70,
    E_MSG_ARC_SYSTEM_AUDIO_MODE_STATUS          = 0x7E,
    E_MSG_ARC_SET_AUDIO_RATE                    = 0x9A,
//----- Audio Return Channel  Control -------------
    E_MSG_ARC_INITIATE_ARC                      = 0xC0,
    E_MSG_ARC_REPORT_ARC_INITIATED              = 0xC1,
    E_MSG_ARC_REPORT_ARC_TERMINATED             = 0xC2,
    E_MSG_ARC_REQUEST_ARC_INITATION             = 0xC3,
    E_MSG_ARC_REQUEST_ARC_TERMINATION           = 0xC4,
    E_MSG_ARC_TERMINATED_ARC                    = 0xC5,

	E_MSG_CDC_MESSAGE                           = 0xF8,
	//amlogic cmd
	//TCL
	CMD_TCL_WIFI						= 0x01,
	CMD_TCL_ETHERNET					= 0x02,
	CMD_TCL_3D							= 0x03,
	CMD_TCL_PANEL_REVERSE				= 0x04,
	CMD_RESERVE1						= 0x05,
	CMD_RESERVE2						= 0x06,
	CMD_RESERVE3						= 0x07,
	//VPU
	CMD_VPU_INIT						= 0x08,
	CMD_VPU_ENABLE						= 0x09,
	CMD_VPU_BYPASS						= 0x0a,
	CMD_VPU_OUTPUT_MUX					= 0x0b,
	CMD_VPU_TIMING						= 0x0c,
	CMD_VPU_SOURCE						= 0x0d,
	CMD_RESERVE4						= 0x0e,
	CMD_RESERVE5						= 0x0f,
	//TCL || AML
	CMD_TCL_BRIDGE_SW_VER				= 0x10,
	//CMD_G9_MAINCODE_VER					= 0x10,
	CMD_TCL_DEVICE_ID					= 0x11,
	//CMD_G9_BOOTCODE_VER					= 0x11,
	CMD_TCL_CLIENT_TYPE					= 0x12,
	//CMD_INFO_G9_TO_FBC					= 0x12,
	CMD_TCL_DEVICE_NUM					= 0x13,
	//CMD_INFO_FBC_TO_G9					= 0x13,
	CMD_TCL_ACTIVE_KEY					= 0x14,
	//CMD_TIME_SYNC						= 0x14,
	CMD_TCL_ACTIVE_STATUS				= 0x15,
	//CMD_KEY_TRANSLATION					= 0x15,
	CMD_RESERVE6						= 0x16,
	CMD_RESERVE7						= 0x17,
	//DEBUG READ
	CMD_DBG_RD_REGISTER_ACCESS			= 0x18,
	CMD_DBG_RD_MEMORY_ACCESS			= 0x19,
	CMD_DBG_RD_SPI_ACCESS				= 0x1a,
	CMD_DBG_RD_VPU_MEMORY_ACCESS		= 0x1b,
	CMD_DBG_RD_MEMORY_TRANSFER			= 0x1c,
	CMD_DBG_INPUT_KEY_DOWN				= 0x1d,
	CMD_DBG_INPUT_KEY_UP				= 0x1e,
	CMD_DBG_INPUT_REBOOT				= 0x1f,
	//DEBUG WRITE
	CMD_DBG_WR_REGISTER_ACCESS			= 0x98,
	CMD_DBG_WR_MEMORY_ACCESS			= 0x99,
	CMD_DBG_WR_SPI_ACCESS				= 0x9a,
	CMD_DBG_WR_VPU_MEMORY_ACCESS		= 0x9b,
	CMD_DBG_WR_MEMORY_TRANSFER			= 0x9c,
	//USER
	CMD_NATURE_LIGHT_ONOFF				= 0x20,
	CMD_USR_BACKLIGHT_ONOFF				= 0x21,
	CMD_USR_BRIGHTNESS					= 0x22,
	CMD_USR_CONTRAST					= 0x23,
	CMD_USR_BACKLIGHT					= 0x24,
	CMD_RESERVE25						= 0x25,
	CMD_USR_SATURATION					= 0x26,
	CMD_USR_DYNAMIC_CONTRAST			= 0x27,
	CMD_USR_PICTURE_MODE				= 0x28,
	CMD_TEST_PATTERN_ONOFF				= 0x29,
	CMD_TEST_PATTERN_SELECT				= 0x2a,
	CMD_RESERVE8						= 0x2b,
	CMD_RESERVE9						= 0x2c,
	CMD_RESERVE10						= 0x2d,
	CMD_RESERVE11						= 0x2e,
	CMD_USR_GAMMA						= 0x2f,
	//FACTORY
	CMD_DEF_SOUND_MODE					= 0x30,
	CMD_DEF_COLOR_TEMPERATURE			= 0x31,
	CMD_DEF_BRIGHTNESS					= 0x32,
	CMD_DEF_CONTRAST					= 0x33,
	CMD_DEF_COLOR						= 0x34,
	CMD_RESERVE12						= 0x35,
	CMD_DEF_BACKLIGHT					= 0x36,
	CMD_RESERVE13						= 0x37,
	CMD_AUTO_LUMA_ONOFF					= 0x38,
	CMD_HISTOGRAM						= 0x39,
	CMD_BLEND							= 0x3a,
	CMD_DEMULA							= 0x3b,
	CMD_COLORSPACE_CONVERSION			= 0x3c,
	CMD_CM2								= 0x3d,
	CMD_RESERVE14						= 0x3e,
	CMD_RESERVE15						= 0x3f,
	// GAIN & OFFSET & WHITEBLANCE
	CMD_DEF_RED_GAIN					= 0x40,
	CMD_DEF_GREEN_GAIN					= 0x41,
	CMD_DEF_BLUE_GAIN					= 0x42,
	CMD_DEF_RED_OFFSET					= 0x43,
	CMD_DEF_GREEN_OFFSET				= 0x44,
	CMD_DEF_BLUE_OFFSET					= 0x45,
	CMD_DEF_PRE_RED_OFFSET				= 0x46,
	CMD_DEF_PRE_GREEN_OFFSET			= 0x47,
	CMD_DEF_PRE_BLUE_OFFSET				= 0x48,
	CMD_RESERVE16						= 0x49,
	CMD_WHITEBLANCE						= 0x4a,

	CMD_SET_SW_VERSION					= 0x57,
	CMD_3D								= 0xd0,

	CMD_MAX								= 0xff
} _cec_cmd;

typedef struct _cec_msg
{
    unsigned char addr; //refer as enum _cec_logic_addr
    unsigned char cmd;  //refer as enum _cec_cmd
    unsigned char msg_data[14];
    unsigned char msg_len;
} _cec_msg;

typedef union tagCECMsgStream{
    unsigned char buf[17];
    struct _cec_msg msg;
} CECMsgStream;

typedef struct _cec_dev_map
{
	enum _cec_logic_addr logic_addr;
	unsigned int phy_addr;
	int cec_dev_type;
	char cec_dev_name[14];
}_cec_dev_map;

typedef struct _cec_msg_queue
{
    struct _cec_msg cec_msg[CEC_MSG_QUEUE_SIZE];
    int wr_index;
    int rd_index;
} _cec_msg_queue;



typedef enum _cec_status
{
    E_CEC_FEATURE_ABORT = 0x00,
    E_CEC_RX_SUCCESS    = 0x01,
    E_CEC_TX_SUCCESS    = 0x02,
    E_CEC_RF            = 0x04,
    E_CEC_LOST_ABT      = 0x08,
    E_CEC_BIT_SHORT     = 0x10,
    E_CEC_BIT_LONG      = 0x20,
    E_CEC_NACK          = 0x40,
    E_CEC_SYSTEM_BUSY   = 0x80,
} _cec_status;


typedef enum _cec_map_status
{
	E_CEC_MAP_UPDATE_START		= 0,
	E_CEC_MAP_POLLLING 			= 1,
	E_CEC_MAP_GET_VENDOR_ID		= 2,
	E_CEC_MAP_GET_PHYSICAL_ADDR	= 3,
	E_CEC_MAP_GET_OSD_NAME		= 4,
	E_CEC_MAP_GET_CEC_VERSION	= 5,
	E_CEC_MAP_UPDATE_COMPLETED	= 6,
}_cec_map_status;

//#define HDMI_IOC_MAGIC 'H'
//#define HDMI_IOC_HDCP_GET_KSV          	_IOR(HDMI_IOC_MAGIC, 0x09, struct _hdcp_ksv)
#define HDMI_IOC_CEC_ON					_IO(HDMI_IOC_MAGIC, 0x01)
#define HDMI_IOC_CEC_OFF				_IO(HDMI_IOC_MAGIC, 0x02)
#define HDMI_IOC_CEC_ARC_ON				_IO(HDMI_IOC_MAGIC, 0x03)
#define HDMI_IOC_CEC_ARC_OFF			_IO(HDMI_IOC_MAGIC, 0x04)
#define HDMI_IOC_CEC_CLEAR_BUFFER		_IO(HDMI_IOC_MAGIC, 0x05)
#define HDMI_IOC_CEC_GET_MSG_CNT		_IOR(HDMI_IOC_MAGIC, 0x06, int)
#define HDMI_IOC_CEC_GET_MSG			_IOR(HDMI_IOC_MAGIC, 0x07, struct _cec_msg)
#define HDMI_IOC_CEC_SENT_MSG        	_IOW(HDMI_IOC_MAGIC, 0x08, struct _cec_msg)


//#include <linux/tvin/tvin.h>
//#include "../tvin_global.h"
//#include "../tvin_format_table.h"

extern void dump_cec_message(int all);
extern void cec_dump_dev_map(void);
extern void clean_cec_message(void);
extern int cec_init(void);
extern void cec_state(bool cec_rx);
extern int cec_handler(bool get_msg, bool get_ack);
extern int hdmirx_cec_rx_monitor(void);
extern int hdmirx_cec_tx_monitor(void);
extern void cec_update_cec_map(void);
extern void cec_post_msg(_cec_logic_addr addr, _cec_cmd cmd, int* data, int length);

//extern void cec_enable_eom_irq(void);
//tcl
extern void hdmirx_cec_fun_onoff(bool enable);
extern void hdmirx_cec_arc_onoff(bool enable);
extern void hdmirx_cec_clear_rx_buffer(void);
extern void cec_post_msg_to_buf(struct _cec_msg *msg);
extern int hdmirx_get_cec_msg_cnt(void);
extern void clean_cec_message(void);
struct _cec_msg *hdmirx_get_rx_msg(void);

#endif
