/*
 * Copyright (c) 2008, Atheros Communications Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _IEEE80211_DEFINES_H_
#define _IEEE80211_DEFINES_H_

/*
 * Public defines for Atheros Upper MAC Layer
 */

/**
 * @brief Opaque handle of 802.11 protocal layer.
 */
struct ieee80211com;
typedef struct ieee80211com *wlan_dev_t; 

/**
 * @brief Opaque handle to App IE module.
*/
struct wlan_mlme_app_ie;
typedef struct wlan_mlme_app_ie *wlan_mlme_app_ie_t; 

/**
 * @brief Opaque handle of network instance (vap) in 802.11 protocal layer.
*/
struct ieee80211vap;
typedef struct ieee80211vap *wlan_if_t; 

/**
 * @brief Opaque handle of a node in the wifi network.
 */
struct ieee80211_node;
typedef struct ieee80211_node *wlan_node_t;

/**
 * @brief Opaque handle of OS interface (ifp in the case of unix ).
 */
struct _os_if_t;
typedef struct _os_if_t *os_if_t; 

/**
 *
 * @brief Opaque handle.
 */
typedef void *os_handle_t; 

/**
 * @brief Opaque handle of a channel.
 */
struct ieee80211_channel;
typedef struct ieee80211_channel *wlan_chan_t;

/**
 * @brief Opaque handle scan_entry.
 */
struct ieee80211_scan_entry; 
typedef struct ieee80211_scan_entry *wlan_scan_entry_t;

#define IEEE80211_NWID_LEN                  32
#define IEEE80211_ISO_COUNTRY_LENGTH        3       /* length of 11d ISO country string */

typedef struct _ieee80211_ssid {
    int         len;
    u_int8_t    ssid[IEEE80211_NWID_LEN];
} ieee80211_ssid;

typedef struct ieee80211_tx_status {
    int         ts_flags;
#define IEEE80211_TX_ERROR          0x01
#define IEEE80211_TX_XRETRY         0x02

    int         ts_retries;     /* number of retries to successfully transmit this frame */
#ifdef ATH_SUPPORT_TxBF
    u_int8_t    ts_txbfstatus;
#define	AR_BW_Mismatch      0x1
#define	AR_Stream_Miss      0x2
#define	AR_CV_Missed        0x4
#define AR_Dest_Miss        0x8
#define AR_Expired          0x10
#define AR_TxBF_Valid_Status    (AR_BW_Mismatch|AR_Stream_Miss|AR_CV_Missed|AR_Dest_Miss|AR_Expired)
#endif
} ieee80211_xmit_status;

#ifndef EXTERNAL_USE_ONLY
typedef struct ieee80211_rx_status {
    int         rs_flags;
#define IEEE80211_RX_FCS_ERROR      0x01
#define IEEE80211_RX_MIC_ERROR      0x02
#define IEEE80211_RX_DECRYPT_ERROR  0x04
/* holes in flags here between, ATH_RX_XXXX to IEEE80211_RX_XXX */
#define IEEE80211_RX_KEYMISS        0x200
    int         rs_rssi;       /* RSSI (noise floor ajusted) */
    int         rs_abs_rssi;   /* absolute RSSI */
    int         rs_datarate;   /* data rate received */
    int         rs_rateieee;
    int         rs_ratephy;

#define IEEE80211_MAX_ANTENNA       3                /* Keep the same as ATH_MAX_ANTENNA */
    u_int8_t    rs_rssictl[IEEE80211_MAX_ANTENNA];   /* RSSI (noise floor ajusted) */
    u_int8_t    rs_rssiextn[IEEE80211_MAX_ANTENNA];  /* RSSI (noise floor ajusted) */    

    enum ieee80211_phymode rs_phymode;
    int         rs_freq;
    
    union {
        u_int8_t            data[8];
        u_int64_t           tsf;
    } rs_tstamp;

#if ATH_SUPPORT_AOW
    u_int16_t   rs_rxseq;      /* WLAN Sequence number */
#endif  
} ieee80211_recv_status;
#endif /* EXTERNAL_USE_ONLY */

/*
 * flags to be passed to ieee80211_vap_create function .
 */
#define IEEE80211_CLONE_BSSID           0x0001  /* allocate unique mac/bssid */
#define IEEE80211_CLONE_NOBEACONS       0x0002  /* don't setup beacon timers */
#define IEEE80211_CLONE_WDS             0x0004  /* enable WDS processing */
#define IEEE80211_CLONE_WDSLEGACY       0x0008  /* legacy WDS operation */
#define IEEE80211_PRIMARY_VAP           0x0010  /* primary vap */
#define IEEE80211_P2PDEV_VAP            0x0020  /* p2pdev vap */
#define IEEE80211_P2PGO_VAP             0x0040  /* p2p-go vap */
#define IEEE80211_P2PCLI_VAP            0x0080  /* p2p-client vap */

/*
 * For the new multi-vap scan feature, there is a set of default priority tables
 * for each OpMode.
 * The following are the default list of the VAP Scan Priority Mapping based on OpModes.
 * NOTE: the following are only used when "#if ATH_SUPPORT_MULTIPLE_SCANS" is true.
 */
/* For IBSS opmode */
#define DEF_VAP_SCAN_PRI_MAP_OPMODE_IBSS_BASE               0
/* For STA opmode */
#define DEF_VAP_SCAN_PRI_MAP_OPMODE_STA_BASE                0
#define DEF_VAP_SCAN_PRI_MAP_OPMODE_STA_P2P_CLIENT          1
/* For HostAp opmode */
#define DEF_VAP_SCAN_PRI_MAP_OPMODE_AP_BASE                 0
#define DEF_VAP_SCAN_PRI_MAP_OPMODE_AP_P2P_GO               1
#define DEF_VAP_SCAN_PRI_MAP_OPMODE_AP_P2P_DEVICE           2
/* For BTAmp opmode */
#define DEF_VAP_SCAN_PRI_MAP_OPMODE_BTAMP_BASE              0

typedef enum _ieee80211_dev_vap_event {
    IEEE80211_VAP_CREATED = 1,
    IEEE80211_VAP_STOPPED,
    IEEE80211_VAP_DELETED
} ieee80211_dev_vap_event;

typedef struct _wlan_dev_event_handler_table {
    void (*wlan_dev_vap_event) (void *event_arg, wlan_dev_t, os_if_t, ieee80211_dev_vap_event);  /* callback to receive vap events*/
#if ATH_SUPPORT_SPECTRAL
    void (*wlan_dev_spectral_indicate)(void*, void*, u_int32_t);
#endif
} wlan_dev_event_handler_table;

typedef int IEEE80211_REASON_CODE;
typedef int IEEE80211_STATUS;

#ifndef EXTERNAL_USE_ONLY
typedef struct _wlan_event_handler_table {
    void (*wlan_receive) (os_if_t osif, wbuf_t wbuf,
                          u_int16_t type, u_int16_t subtype,
                          ieee80211_recv_status *rs);                       /* callback to receive all the packets */
    int  (*wlan_receive_filter_80211) (os_if_t osif, wbuf_t wbuf,
                                       u_int16_t type, u_int16_t subtype,
                                       ieee80211_recv_status *rs);          /* callback to receive and filter all the 802.11 frames*/
    void (*wlan_receive_monitor_80211) (os_if_t osif, wbuf_t wbuf,
                                        ieee80211_recv_status *rs);         /* recieve 802.11 frames for monitor mode only*/
    int  (*wlan_dev_xmit_queue) (os_if_t osif, wbuf_t wbuf);                /* queue packet to the device for transmit */
    void (*wlan_vap_xmit_queue) (os_if_t osif, wbuf_t wbuf);                /* queue packet to the interface for transmit */
    void (*wlan_xmit_update_status)(os_if_t osif, wbuf_t wbuf,
                                    ieee80211_xmit_status *ts);             /* callback to indicate tx completion.
                                                                             * NB: this is to allow IHV to update certain statistics,
                                                                             * it should NOT consume the wbuf */
} wlan_event_handler_table;

typedef struct _wlan_mlme_event_handler_table {
    /* MLME confirmation handler */
    void (*mlme_join_complete_infra)(os_handle_t, IEEE80211_STATUS);
    void (*mlme_join_complete_adhoc)(os_handle_t, IEEE80211_STATUS);
    void (*mlme_auth_complete)(os_handle_t, IEEE80211_STATUS);
    void (*mlme_assoc_req)(os_handle_t, wbuf_t wbuf);
    void (*mlme_assoc_complete)(os_handle_t, IEEE80211_STATUS, u_int16_t aid, wbuf_t wbuf);
    void (*mlme_reassoc_complete)(os_handle_t, IEEE80211_STATUS, u_int16_t aid, wbuf_t wbuf);
    void (*mlme_deauth_complete)(os_handle_t, IEEE80211_STATUS);
    void (*mlme_disassoc_complete)(os_handle_t, u_int8_t *, u_int16_t, IEEE80211_STATUS);

    /* MLME indication handler */
    void (*mlme_auth_indication)(os_handle_t, u_int8_t *macaddr, u_int16_t result);
    void (*mlme_deauth_indication)(os_handle_t, u_int8_t *macaddr, u_int16_t reason_code);
    void (*mlme_assoc_indication)(os_handle_t, u_int8_t *macaddr, u_int16_t result, wbuf_t wbuf, wbuf_t resp_wbuf);
    void (*mlme_reassoc_indication)(os_handle_t, u_int8_t *macaddr, u_int16_t result, wbuf_t wbuf, wbuf_t resp_wbuf);
    void (*mlme_disassoc_indication)(os_handle_t, u_int8_t *macaddr, u_int16_t reason_code);
    void (*mlme_ibss_merge_start_indication)(os_handle_t, u_int8_t *bssid);
    void (*mlme_ibss_merge_completion_indication)(os_handle_t, u_int8_t *bssid);
    void (*wlan_radar_detected) (os_if_t, u_int32_t csa_delay);
} wlan_mlme_event_handler_table;

typedef struct _wlan_misc_event_handler_table {
    void (*wlan_channel_change) (os_handle_t, wlan_chan_t chan);
    void (*wlan_country_changed) (os_handle_t, char *country);
    void (*wlan_linkspeed)(os_handle_t, u_int32_t rxlinkspeed, u_int32_t txlinkspeed);
    void (*wlan_michael_failure_indication)(os_handle_t, const u_int8_t *frm, u_int keyix);
    void (*wlan_replay_failure_indication)(os_handle_t, const u_int8_t *frm, u_int keyix);
    void (*wlan_beacon_miss_indication)(os_handle_t);
    void (*wlan_device_error_indication)(os_handle_t);
    void (*wlan_sta_clonemac_indication)(os_handle_t);
    void (*wlan_sta_scan_entry_update)(os_handle_t, wlan_scan_entry_t scan_entry);
} wlan_misc_event_handler_table;

typedef struct _wlan_ccx_handler_table {
    void (*wlan_ccx_trigger_roam) (os_if_t,  u_int16_t reason);
    void (*wlan_ccx_set_vperf) (os_if_t, u_int8_t);
    void (*wlan_ccx_fill_tsrsie)(os_if_t, u_int8_t tid, u_int32_t rate, u_int8_t *frm, u_int8_t *len);
    bool (*wlan_ccx_is_mfp)(os_if_t);
    int  (*wlan_ccx_dc_match)(os_if_t, wlan_scan_entry_t);
    void (*wlan_ccx_bss_adjutil)(os_if_t, wlan_scan_entry_t, int, int, u_int32_t*);
    void (*wlan_ccx_process_qos)(os_if_t, u_int8_t, u_int16_t);
    bool (*wlan_ccx_check_msdu_life)(os_if_t, u_int16_t);
    void (*wlan_ccx_vperf_pause)(os_if_t, bool);
} wlan_ccx_handler_table;

/* action frame complete callback*/
typedef void (*wlan_action_frame_complete_handler)(wlan_if_t vaphandle, wbuf_t wbuf, void *arg, u_int8_t *dst_addr, u_int8_t *src_addr, u_int8_t *bssid, ieee80211_xmit_status *ts);

/* 
 *                        
 *  wlan_dev_xmit      : the OS dependent layer needs to register a callback function.
 *                       umac layer uses this callback to xmit a packet. the implementaion
 *                       of this  call back function would implement OS specific handling of 
 *                       the wbuf and at the end it if it decides to xmit the wbuf it should 
 *                       call the ath_tx_send (the ath layer xmit funtion). the implementation
 *                       should return 0 on success and -ve value on failure.
 *
 *  wlan_vap_xmit      : the OS dependent layer needs to register a callback function.
 *                       umac layer uses this callback to xmit a packet. the implementaion
 *                       of this  call back function would implement send the packet on to the 
 *                       transmit queue of the vap. if IEEE80211_DELIVER_80211 param is set the 
 *                       frame is 802.11 frame else  it is ethernet (802.3) frame.
 * 
 *  wlan_receive       : handler to receive all the packets icluding data,mgmt,control.
 *                       UMAC will deliver standard 802.11 frames (with qos control removed)
 *                       if IEEE80211_DELIVER_80211 param is set. 
 *                       UMAC will deliver ethernet frames (with 802.11 header decapped)
 *                       if IEEE80211_DELIVER_80211 param is not set. 
 *                       the handler should cosume the wbuf (release it eventually). 
 *
 *  wlan_receive_filter_80211 : handler to receive all the frames with 802.11 header unstripped but the FCS stripped at the end.
 *                              implementation of the handler should make a copy of the wbuf and should not
 *                              release the wbuf passed via the handler. implementation should return non zero if the packet
 *                              should not be processed further by umac.
 *
 *  wlan_receive_monitor_80211 : handler to receive the  all the frames with the 802.11 header including the FCS.
 *                               the handler should cosume the wbuf (release it eventually). 
 *                               this function is exclusively used for monitor mode.
 *
 *  
 */
#endif /* EXTERNAL_USE_ONLY */
/*
 * scan API related structs.
 */
typedef enum _ieee80211_scan_type {
    IEEE80211_SCAN_BACKGROUND,
    IEEE80211_SCAN_FOREGROUND,
    IEEE80211_SCAN_SPECTRAL,
    IEEE80211_SCAN_REPEATER_BACKGROUND,
    IEEE80211_SCAN_REPEATER_EXT_BACKGROUND,
    IEEE80211_SCAN_RADIO_MEASUREMENTS,
} ieee80211_scan_type;

/*
 * Priority numbers must be sequential, starting with 0.
 */
typedef enum ieee80211_scan_priority_t {
    IEEE80211_SCAN_PRIORITY_LOW    = 0,
    IEEE80211_SCAN_PRIORITY_MEDIUM,
    IEEE80211_SCAN_PRIORITY_HIGH,

    IEEE80211_SCAN_PRIORITY_COUNT   /* number of priorities supported */
} IEEE80211_SCAN_PRIORITY;

typedef u_int16_t    IEEE80211_SCAN_REQUESTOR;
typedef u_int32_t    IEEE80211_SCAN_ID;

#define IEEE80211_SCAN_ID_NONE                    0

/* All P2P scans currently use medium priority */
#define IEEE80211_P2P_DEFAULT_SCAN_PRIORITY       IEEE80211_SCAN_PRIORITY_MEDIUM

/* Masks identifying types/ID of scans */
#define IEEE80211_SPECIFIC_SCAN       0x00000000
#define IEEE80211_VAP_SCAN            0x01000000
#define IEEE80211_ALL_SCANS           0x04000000

#define IEEE80211_SCAN_CLASS_MASK     0xFF000000

#define IEEE80211_SCAN_PASSIVE            0x0001 /* passively scan all the channels */
#define IEEE80211_SCAN_ACTIVE             0x0002 /* actively  scan all the channels (regdomain rules still apply) */
#define IEEE80211_SCAN_2GHZ               0x0004 /* scan 2GHz band */
#define IEEE80211_SCAN_5GHZ               0x0008 /* scan 5GHz band */
#define IEEE80211_SCAN_ALLBANDS           (IEEE80211_SCAN_5GHZ | IEEE80211_SCAN_2GHZ)
#define IEEE80211_SCAN_CONTINUOUS         0x0010 /* keep scanning until maxscantime expires */
#define IEEE80211_SCAN_FORCED             0x0020 /* forced scan (OS request) - should proceed even in the presence of data traffic */
#define IEEE80211_SCAN_NOW                0x0040 /* scan now (User request)  - should proceed even in the presence of data traffic */
#define IEEE80211_SCAN_ADD_BCAST_PROBE    0x0080 /* add wildcard ssid and broadcast probe request if there is none */
#define IEEE80211_SCAN_EXTERNAL           0x0100 /* scan requested by OS */

#define IEEE80211_SCAN_PARAMS_MAX_SSID     10
#define IEEE80211_SCAN_PARAMS_MAX_BSSID    10


/* flag definitions passed to scan_cancel API */

#define IEEE80211_SCAN_CANCEL_ASYNC 0x0 /* asynchronouly wait for scan SM to complete cancel */ 
#define IEEE80211_SCAN_CANCEL_WAIT  0x1 /* wait for scan SM to complete cancel */ 
#define IEEE80211_SCAN_CANCEL_SYNC  0x2 /* synchronously execute cancel scan */ 

#ifndef EXTERNAL_USE_ONLY
typedef bool (*ieee80211_scan_termination_check) (void *arg);

typedef struct _ieee80211_scan_params {
    ieee80211_scan_type                 type;
    int                                 min_dwell_time_active;  /* min time in msec on active channels */
    int                                 max_dwell_time_active;  /* max time in msec on active channels (if no response) */
    int                                 min_dwell_time_passive; /* min time in msec on passive channels */
    int                                 max_dwell_time_passive; /* max time in msec on passive channels (if no response) */
    int                                 min_rest_time;          /* min time in msec on the BSS channel, only valid for BG scan */
    int                                 max_rest_time;          /* max time in msec on the BSS channel, only valid for BG scan */
    int                                 repeat_probe_time;      /* time before sending second probe request */
    int                                 idle_time;              /* time in msec on bss channel before switching channel */
    int                                 max_scan_time;          /* maximum time in msec allowed for scan  */
    int                                 probe_delay;            /* delay in msec before sending probe request */
    int                                 offchan_retry_delay;    /* delay in msec before retrying off-channel switch */
    int                                 min_beacon_count;       /* number of home AP beacons to receive before leaving the home channel */
    int                                 max_offchan_retries;    /* maximum number of times to retry off-channel switch */
    int                                 beacon_timeout;         /* maximum time to wait for beacons */
    int                                 flags;                  /* scan flags */
    int                                 num_channels;           /* number of channels to scan */
    bool                                multiple_ports_active;  /* driver has multiple ports active in the home channel */
    bool                                restricted_scan;        /* Perform restricted scan */
    u_int32_t                           *chan_list;             /* array of ieee channels (or) frequencies to scan */
    int                                 num_ssid;               /* number of desired ssids */
    ieee80211_ssid                      ssid_list[IEEE80211_SCAN_PARAMS_MAX_SSID];
    int                                 num_bssid;              /* number of desired bssids */
    u_int8_t                            bssid_list[IEEE80211_SCAN_PARAMS_MAX_BSSID][IEEE80211_ADDR_LEN];
    struct ieee80211_node               *bss_node;              /* BSS node */
    int                                 ie_len;                 /* length of the ie data to be added to probe req */ 
    u_int8_t                            *ie_data;               /* pointer to ie data */
    ieee80211_scan_termination_check    check_termination_function;  /* function checking for termination condition */
    void                                *check_termination_context;  /* context passed to function above */
} ieee80211_scan_params;

/* Data types used to specify scan priorities */
typedef u_int32_t IEEE80211_PRIORITY_MAPPING[IEEE80211_SCAN_PRIORITY_COUNT];

/**************************************
 * Called before attempting to roam.  Modifies the rssiAdder of a BSS
 * based on the preferred status of a BSS.
 *
 * According to CCX spec, AP in the neighbor list is not meant for giving extra
 * weightage in roaming. By doing so, roaming becomes sticky. See bug 21220.
 * Change the weightage to 0. Cisco may ask in future for a user control of
 * this weightage.
 */
#define PREFERRED_BSS_RANK                20
#define NEIGHBOR_BSS_RANK                  0    /* must be less than preferred BSS rank */

/*
 * The utility of the BSS is the metric used in the selection
 * of a BSS. The Utility of the BSS is reduced if we just left the BSS.
 * The Utility of the BSS is not reduced if we have left the
 * BSS for 8 seconds (8000ms) or more.
 * 2^13 milliseconds is a close approximation to avoid expensive division
 */
#define LAST_ASSOC_TIME_DELTA_REQUIREMENT (1 << 13) // 8192

#define QBSS_SCALE_MAX                   255  /* Qbss channel load Max value */
#define QBSS_SCALE_DOWN_FACTOR             2  /* scale factor to reduce Qbss channel load */
#define QBSS_HYST_ADJ                     60  /* Qbss Weightage factor for the current AP */

/*
 * Flags used to set field APState
 */
#define AP_STATE_GOOD    0x00
#define AP_STATE_BAD     0x01
#define AP_STATE_RETRY   0x10
#define BAD_AP_TIMEOUT   6000   // In milli seconds
/*
 * To disable BAD_AP status check on any scan entry
 */
#define BAD_AP_TIMEOUT_DISABLED             0

/*
 * BAD_AP timeout specified in seconds
 */
#define BAD_AP_TIMEOUT_IN_SECONDS           10

/*
 * State values used to represent our assoc_state with ap (discrete, not bitmasks)
 */
#define AP_ASSOC_STATE_NONE     0
#define AP_ASSOC_STATE_AUTH     1
#define AP_ASSOC_STATE_ASSOC    2

/*
 * Entries in the scan list are considered obsolete after 75 seconds.
 */
#define IEEE80211_SCAN_ENTRY_EXPIRE_TIME           75000

/*
 * idle time is only valid for scan type IEEE80211_SCAN_BACKGROUND.
 * if idle time is set then the scanner would change channel from BSS
 * channel to foreign channel only if both resttime is expired and 
 * the theres was not traffic for idletime msec on the bss channel.
 * value of 0 for idletime would cause the channel to switch from BSS
 * channel to foreign channel as soon  as the resttime is expired.
 *
 * if maxscantime is nonzero and if the scanner can not complete the 
 * scan in maxscantime msec then the scanner will cancel the scan and
 * post IEEE80211_SCAN_COMPLETED event with reason SCAN_TIMEDOUT.
 *
 */  

/*
 * chanlist can be either ieee channels (or) frequencies.
 * if a value is less than 1000 implementation assumes it 
 * as ieee channel # otherwise implementation assumes it 
 * as frequency in Mhz.
 */ 

typedef enum _ieee80211_scan_event_type {
    IEEE80211_SCAN_STARTED,
    IEEE80211_SCAN_COMPLETED,
    IEEE80211_SCAN_RADIO_MEASUREMENT_START,
    IEEE80211_SCAN_RADIO_MEASUREMENT_END,
    IEEE80211_SCAN_RESTARTED,
    IEEE80211_SCAN_HOME_CHANNEL,
    IEEE80211_SCAN_FOREIGN_CHANNEL,
    IEEE80211_SCAN_BSSID_MATCH,
    IEEE80211_SCAN_FOREIGN_CHANNEL_GET_NF,
    IEEE80211_SCAN_DEQUEUED,
    IEEE80211_SCAN_PREEMPTED,

    IEEE80211_SCAN_EVENT_COUNT
} ieee80211_scan_event_type;

typedef enum ieee80211_scan_completion_reason {
    IEEE80211_REASON_NONE,
    IEEE80211_REASON_COMPLETED,
    IEEE80211_REASON_CANCELLED,
    IEEE80211_REASON_TIMEDOUT,
    IEEE80211_REASON_TERMINATION_FUNCTION,
    IEEE80211_REASON_MAX_OFFCHAN_RETRIES,
    IEEE80211_REASON_PREEMPTED,
    IEEE80211_REASON_RUN_FAILED,
    IEEE80211_REASON_INTERNAL_STOP,

    IEEE80211_REASON_COUNT
} ieee80211_scan_completion_reason;

typedef struct _ieee80211_scan_event {
    ieee80211_scan_event_type           type;
    ieee80211_scan_completion_reason    reason;
    wlan_chan_t                         chan;
    IEEE80211_SCAN_REQUESTOR            requestor; /* Requestor ID passed to the scan_start function */
    IEEE80211_SCAN_ID                   scan_id;   /* Specific ID of the scan reporting the event */
} ieee80211_scan_event;

typedef enum _ieee80211_scan_request_status {
    IEEE80211_SCAN_STATUS_QUEUED,
    IEEE80211_SCAN_STATUS_RUNNING,
    IEEE80211_SCAN_STATUS_PREEMPTED,
    IEEE80211_SCAN_STATUS_COMPLETED
} ieee80211_scan_request_status;

/*
 * the sentry field of tht ieee80211_scan_event is only valid if the
 * event type is IEEE80211_SCAN_BSSID_MATCH.
 */

typedef void (*ieee80211_scan_event_handler) (wlan_if_t vaphandle, ieee80211_scan_event *event, void *arg);  

typedef struct _ieee80211_scan_info {
    ieee80211_scan_type                type;
    IEEE80211_SCAN_REQUESTOR           requestor;                   /* Originator ID passed to the scan_start function */
    IEEE80211_SCAN_ID                  scan_id;                     /* Specific ID of the scan reporting the event */
    IEEE80211_SCAN_PRIORITY            priority;                    /* Requested priority level (low/medium/high) */
    ieee80211_scan_request_status      scheduling_status;           /* Queued/running/preempted/completed */
    int                                min_dwell_time_active;       /* min time in msec on active channels */
    int                                max_dwell_time_active;       /* max time in msec on active channel (if no response) */
    int                                min_dwell_time_passive;      /* min time in msec on passive channels */
    int                                max_dwell_time_passive;      /* max time in msec on passive channel*/
    int                                min_rest_time;               /* min time in msec on the BSS channel, only valid for BG scan */
    int                                max_rest_time;               /* max time in msec on the BSS channel, only valid for BG scan */
    int                                repeat_probe_time;           /* time before sending second probe request */
    int                                min_beacon_count;            /* number of home AP beacons to receive before leaving the home channel */
    int                                flags;                       /* scan flags */
    systime_t                          scan_start_time;             /* system time when last scani started */ 
    int                                scanned_channels;            /* number of scanned channels */
    int                                default_channel_list_length; /* number of channels in the default channel list */
    int                                channel_list_length;         /* number of channels in the channel list used for the current scan */
    int                                in_progress            : 1,  /* if the scan is in progress */
                                       cancelled              : 1,  /* if the scan is cancelled */
                                       preempted              : 1,  /* if the scan is preempted */
                                       restricted             : 1;  /* if the scan is restricted */
} ieee80211_scan_info;

typedef struct _ieee80211_scan_request_info {
    wlan_if_t                        vaphandle;
    IEEE80211_SCAN_REQUESTOR         requestor;
    IEEE80211_SCAN_PRIORITY          requested_priority;
    IEEE80211_SCAN_PRIORITY          absolute_priority;
    IEEE80211_SCAN_ID                scan_id;
    ieee80211_scan_request_status    scheduling_status;
    ieee80211_scan_params            params;
    systime_t                        request_timestamp;
    u_int32_t                        maximum_duration;
} ieee80211_scan_request_info;

#endif /* EXTERNAL_USE_ONLY */

#ifndef EXTERNAL_USE_ONLY
typedef void (*ieee80211_acs_event_handler) (void *arg, wlan_chan_t channel);  
#endif /* EXTERNAL_USE_ONLY */

#define MAX_CHAINS 3

typedef struct _wlan_rssi_info {
    int8_t      avg_rssi;     /* average rssi */
    u_int8_t    valid_mask;   /* bitmap of valid elements in rssi_ctrl/ext array */
    int8_t      rssi_ctrl[MAX_CHAINS];
    int8_t      rssi_ext[MAX_CHAINS];
} wlan_rssi_info;

typedef enum _wlan_rssi_type {
    WLAN_RSSI_TX,
    WLAN_RSSI_RX,
    WLAN_RSSI_BEACON  /* rssi of the beacon, only valid for STA/IBSS vap */
} wlan_rssi_type;

typedef enum _ieee80211_rate_type {
    IEEE80211_RATE_TYPE_LEGACY,
    IEEE80211_RATE_TYPE_MCS,
} ieee80211_rate_type;

typedef struct _ieee80211_rate_info {
    ieee80211_rate_type    type;
    u_int32_t              rate;     /* average rate in kbps */
    u_int32_t              lastrate; /* last packet rate in kbps */
    u_int8_t               mcs;      /* mcs index . is valid if rate type is MCS20 or MCS40 */
} ieee80211_rate_info;

typedef enum _ieee80211_node_param_type {
    IEEE80211_NODE_PARAM_TX_POWER,
    IEEE80211_NODE_PARAM_ASSOCID,
    IEEE80211_NODE_PARAM_INACT,     /* inactivity timer value */
    IEEE80211_NODE_PARAM_AUTH_MODE, /* auth mode */
    IEEE80211_NODE_PARAM_CAP_INFO,  /* auth mode */
} ieee80211_node_param_type;

/*
 * Per/node (station) statistics available when operating as an AP.
 */
struct ieee80211_nodestats {
    u_int32_t    ns_rx_data;             /* rx data frames */
    u_int32_t    ns_rx_mgmt;             /* rx management frames */
    u_int32_t    ns_rx_ctrl;             /* rx control frames */
    u_int32_t    ns_rx_ucast;            /* rx unicast frames */
    u_int32_t    ns_rx_mcast;            /* rx multi/broadcast frames */
    u_int64_t    ns_rx_bytes;            /* rx data count (bytes) */
    u_int64_t    ns_rx_beacons;          /* rx beacon frames */
    u_int32_t    ns_rx_proberesp;        /* rx probe response frames */

    u_int32_t    ns_rx_dup;              /* rx discard 'cuz dup */
    u_int32_t    ns_rx_noprivacy;        /* rx w/ wep but privacy off */
    u_int32_t    ns_rx_wepfail;          /* rx wep processing failed */
    u_int32_t    ns_rx_demicfail;        /* rx demic failed */
    u_int32_t    ns_rx_decap;            /* rx decapsulation failed */
    u_int32_t    ns_rx_defrag;           /* rx defragmentation failed */
    u_int32_t    ns_rx_disassoc;         /* rx disassociation */
    u_int32_t    ns_rx_deauth;           /* rx deauthentication */
    u_int32_t    ns_rx_action;           /* rx action */
    u_int32_t    ns_rx_decryptcrc;       /* rx decrypt failed on crc */
    u_int32_t    ns_rx_unauth;           /* rx on unauthorized port */
    u_int32_t    ns_rx_unencrypted;      /* rx unecrypted w/ privacy */

    u_int32_t    ns_tx_data;             /* tx data frames */
    u_int32_t    ns_tx_mgmt;             /* tx management frames */
    u_int32_t    ns_tx_ucast;            /* tx unicast frames */
    u_int32_t    ns_tx_mcast;            /* tx multi/broadcast frames */
    u_int64_t    ns_tx_bytes;            /* tx data count (bytes) */
    u_int32_t    ns_tx_probereq;         /* tx probe request frames */
    u_int32_t    ns_tx_uapsd;            /* tx on uapsd queue */

    u_int32_t    ns_tx_novlantag;        /* tx discard 'cuz no tag */
    u_int32_t    ns_tx_vlanmismatch;     /* tx discard 'cuz bad tag */

    u_int32_t    ns_tx_eosplost;         /* uapsd EOSP retried out */

    u_int32_t    ns_ps_discard;          /* ps discard 'cuz of age */

    u_int32_t    ns_uapsd_triggers;      /* uapsd triggers */
    u_int32_t    ns_uapsd_duptriggers;    /* uapsd duplicate triggers */
    u_int32_t    ns_uapsd_ignoretriggers; /* uapsd duplicate triggers */
    u_int32_t    ns_uapsd_active;         /* uapsd duplicate triggers */
    u_int32_t    ns_uapsd_triggerenabled; /* uapsd duplicate triggers */


    /* MIB-related state */
    u_int32_t    ns_tx_assoc;            /* [re]associations */
    u_int32_t    ns_tx_assoc_fail;       /* [re]association failures */
    u_int32_t    ns_tx_auth;             /* [re]authentications */
    u_int32_t    ns_tx_auth_fail;        /* [re]authentication failures*/
    u_int32_t    ns_tx_deauth;           /* deauthentications */
    u_int32_t    ns_tx_deauth_code;      /* last deauth reason */
    u_int32_t    ns_tx_disassoc;         /* disassociations */
    u_int32_t    ns_tx_disassoc_code;    /* last disassociation reason */
    u_int32_t    ns_psq_drops;           /* power save queue drops */
    
    /* IQUE-HBR related state */
#ifdef ATH_SUPPORT_IQUE
	u_int32_t	ns_tx_dropblock;	/* tx discard 'cuz headline block */
#endif
};

/*
 * station power save mode.
 */
typedef enum ieee80211_psmode {
    IEEE80211_PWRSAVE_NONE = 0,          /* no power save */
    IEEE80211_PWRSAVE_LOW,
    IEEE80211_PWRSAVE_NORMAL,
    IEEE80211_PWRSAVE_MAXIMUM
} ieee80211_pwrsave_mode;

/*
 * apps power save state.
 */
typedef enum {
     APPS_AWAKE = 0,
     APPS_PENDING_SLEEP,
     APPS_SLEEP,
     APPS_FAKE_SLEEP,           /* Pending blocking sleep */
     APPS_FAKING_SLEEP,         /* Blocking sleep */
     APPS_UNKNOWN_PWRSAVE,
} ieee80211_apps_pwrsave_state;

typedef enum _iee80211_mimo_powersave_mode {
    IEEE80211_MIMO_POWERSAVE_NONE,    /* no mimo power save */
    IEEE80211_MIMO_POWERSAVE_STATIC,  /* static mimo power save */
    IEEE80211_MIMO_POWERSAVE_DYNAMIC  /* dynamic mimo powersave */
} ieee80211_mimo_powersave_mode;

#ifdef ATH_COALESCING
typedef enum _ieee80211_coalescing_state {
    IEEE80211_COALESCING_DISABLED   = 0,        /* Coalescing is disabled*/
    IEEE80211_COALESCING_DYNAMIC    = 1,        /* Dynamically move to Enabled state based on Uruns*/
    IEEE80211_COALESCING_ENABLED    = 2,        /* Coalescing is enabled*/
} ieee80211_coalescing_state;

#define IEEE80211_TX_COALESCING_THRESHOLD     5 /* Number of underrun errors to trigger coalescing */
#endif

typedef enum _ieee80211_cap {
    IEEE80211_CAP_SHSLOT,                    /* CAPABILITY: short slot */
    IEEE80211_CAP_SHPREAMBLE,                /* CAPABILITY: short premable */
    IEEE80211_CAP_MULTI_DOMAIN,              /* CAPABILITY: multiple domain */
    IEEE80211_CAP_WMM,                       /* CAPABILITY: WMM */
    IEEE80211_CAP_HT,                        /* CAPABILITY: HT */
} ieee80211_cap;

typedef enum _ieee80211_device_param {
    IEEE80211_DEVICE_NUM_TX_CHAIN,
    IEEE80211_DEVICE_NUM_RX_CHAIN,
    IEEE80211_DEVICE_TX_CHAIN_MASK,
    IEEE80211_DEVICE_RX_CHAIN_MASK,
    IEEE80211_DEVICE_TX_CHAIN_MASK_LEGACY,
    IEEE80211_DEVICE_RX_CHAIN_MASK_LEGACY, 
    IEEE80211_DEVICE_BMISS_LIMIT,            /* # of beacon misses for HW to generate BMISS intr */
    IEEE80211_DEVICE_PROTECTION_MODE,        /* protection mode*/
    IEEE80211_DEVICE_BLKDFSCHAN,             /* block the use of DFS channels */
    IEEE80211_DEVICE_GREEN_AP_PS_ENABLE,
    IEEE80211_DEVICE_GREEN_AP_PS_TIMEOUT,
    IEEE80211_DEVICE_GREEN_AP_PS_ON_TIME,
    IEEE80211_DEVICE_CWM_EXTPROTMODE,
    IEEE80211_DEVICE_CWM_EXTPROTSPACING,
    IEEE80211_DEVICE_CWM_ENABLE,
    IEEE80211_DEVICE_CWM_EXTBUSYTHRESHOLD,
    IEEE80211_DEVICE_DOTH,
    IEEE80211_DEVICE_ADDBA_MODE,
    IEEE80211_DEVICE_COUNTRYCODE,
    IEEE80211_DEVICE_MULTI_CHANNEL,           /* turn on/off off channel support */    
    IEEE80211_DEVICE_MAX_AMSDU_SIZE,          /* Size of AMSDU to be sent on the air */
#if ATH_SUPPORT_IBSS_HT 
    IEEE80211_DEVICE_HT20ADHOC,
    IEEE80211_DEVICE_HT40ADHOC,
    IEEE80211_DEVICE_HTADHOCAGGR,
#endif
    IEEE80211_DEVICE_P2P,                     /* Enable or Disable P2P */
    IEEE80211_DEVICE_OVERRIDE_SCAN_PROBERESPONSE_IE, /* Override scan Probe response IE, 0: Don't over-ride */
} ieee80211_device_param;

typedef enum _ieee80211_param {
    IEEE80211_BEACON_INTVAL,                 /* in TUs */
#if ATH_SUPPORT_AP_WDS_COMBO
    IEEE80211_NO_BEACON,                     /* the vap does not tx beacon/probe resp. */
#endif
    IEEE80211_LISTEN_INTVAL,                 /* number of beacons */
    IEEE80211_DTIM_INTVAL,                   /* number of beacons */
    IEEE80211_BMISS_COUNT_RESET,             /* number of beacon miss intrs before reset */
    IEEE80211_BMISS_COUNT_MAX,               /* number of beacon miss intrs for bmiss notificationst */
    IEEE80211_ATIM_WINDOW,                   /* ATIM window */
    IEEE80211_SHORT_SLOT,                    /* short slot on/off */
    IEEE80211_SHORT_PREAMBLE,                /* short preamble on/off */
    IEEE80211_RTS_THRESHOLD,                 /* rts threshold, 0 means no rts threshold  */
    IEEE80211_FRAG_THRESHOLD,                /* fragmentation threshold, 0 means no rts threshold  */
    IEEE80211_FIXED_RATE,                    /* 
                                              * rate code series(0: auto rate, 32 bit value:  rate 
                                              * codes for 4 rate series. each byte for one rate series) 
                                              */
    IEEE80211_MCAST_RATE,                    /* rate in Kbps */
    IEEE80211_TXPOWER,                       /* in 0.5db units */
    IEEE80211_AMPDU_DENCITY,                 /* AMPDU dencity*/
    IEEE80211_AMPDU_LIMIT,                   /* AMPDU limit*/
    IEEE80211_WPS_MODE,                      /* WPS mode*/
    IEEE80211_TSN_MODE,                      /* TSN mode*/
    IEEE80211_MULTI_DOMAIN,                  /* Multiple domain */
    IEEE80211_SAFE_MODE,                     /* Safe mode */
    IEEE80211_NOBRIDGE_MODE,                 /* No bridging done, all frames sent up the stack */
    IEEE80211_PERSTA_KEYTABLE_SIZE,          /* IBSS-only, read-only: persta key table size */
    IEEE80211_RECEIVE_80211,                 /* deliver std 802.11 frames 802.11 instead of ethernet frames on the rx */
    IEEE80211_SEND_80211,                    /* OS sends std 802.11 frames 802.11 instead of ethernet frames on tx side */
    IEEE80211_MSG_FLAGS,                     /* flags to turn on different debug messages */
    IEEE80211_MIN_BEACON_COUNT,              /* minumum number beacons to tx/rx before vap can pause */
    IEEE80211_IDLE_TIME,                     /* minimun no activity time before vap can pause */
    IEEE80211_MIN_FRAMESIZE,                 /* smallest frame size we are allowed to receive */
                                             /* features. 0:feature is off. 1:feature is on. */
    IEEE80211_FEATURE_WMM,                   /* WMM */
    IEEE80211_FEATURE_WMM_PWRSAVE,           /* WMM Power Save */
    IEEE80211_FEATURE_UAPSD,                 /* UAPSD setting (BE/BK/VI/VO) */
    IEEE80211_FEATURE_WDS,                   /* dynamic WDS feature */
    IEEE80211_FEATURE_PRIVACY,               /* encryption */
    IEEE80211_FEATURE_DROP_UNENC,            /* drop un encrypted frames */
    IEEE80211_FEATURE_COUNTER_MEASURES ,     /* turn on couter measures */
    IEEE80211_FEATURE_HIDE_SSID,             /* turn on hide ssid feature */
    IEEE80211_FEATURE_APBRIDGE,              /* turn on internal mcast traffic bridging for AP */
    IEEE80211_FEATURE_PUREB,                 /* turn on pure B mode for AP */
    IEEE80211_FEATURE_PUREG,                 /* turn on pure G mode for AP */
    IEEE80211_FEATURE_REGCLASS,              /* add regulatory class IE in AP */
    IEEE80211_FEATURE_COUNTRY_IE,            /* add country IE for vap in AP */
    IEEE80211_FEATURE_IC_COUNTRY_IE,         /* add country IE for ic in AP */
    IEEE80211_FEATURE_DOTH,                  /* enable 802.11h */
    IEEE80211_FEATURE_PURE11N,               /* enable pure 11n  mode */
    IEEE80211_FEATURE_PRIVATE_RSNIE,         /* enable OS shim to setup RSN IE*/
    IEEE80211_FEATURE_COPY_BEACON,           /* keep a copy of beacon */
    IEEE80211_FEATURE_PSPOLL,                /* enable/disable pspoll mode in power save SM */
    IEEE80211_FEATURE_CONTINUE_PSPOLL_FOR_MOREDATA, /* enable/disable option to contunue sending ps polls when there is more data */
    IEEE80211_FEATURE_AMPDU,                 /* Enable or Disable Aggregation */
#ifdef ATH_COALESCING
    IEEE80211_FEATURE_TX_COALESCING,         /* enable tx coalescing */
#endif
    IEEE80211_FEATURE_VAP_IND,               /* Repeater independant VAP */   
    IEEE80211_FIXED_RETRIES,                 /* fixed retries  0-4 */
    IEEE80211_SHORT_GI,                      /* short gi on/off */
    IEEE80211_HT40_INTOLERANT,
    IEEE80211_CHWIDTH,
    IEEE80211_CHEXTOFFSET,
    IEEE80211_STA_QUICKKICKOUT,
    IEEE80211_CHSCANINIT,
    IEEE80211_FEATURE_STAFWD,                /* dynamic AP Client  feature */
    IEEE80211_DRIVER_CAPS,                   
#if ATH_SUPPORT_IQUE
    IEEE80211_IQUE_CONFIG,
    IEEE80211_ME,                            /* mcast enhancement mode */
	IEEE80211_MEDEBUG,
	IEEE80211_MEDUMP,
	IEEE80211_ME_SNOOPLENGTH,
	IEEE80211_ME_TIMER,
	IEEE80211_ME_TIMEOUT,
	IEEE80211_ME_DROPMCAST,
    IEEE80211_ME_SHOWDENY,
    IEEE80211_ME_CLEARDENY,
    IEEE80211_ME_ADDDENY,
    IEEE80211_HBR_TIMER,
#endif //ATH_SUPPORT_IQUE
    IEEE80211_UAPSD_MAXSP,                    /* UAPSD service period setting (0:unlimited, 2,4,6) */
    IEEE80211_WEP_MBSSID,
    IEEE80211_MGMT_RATE,                      /* ieee rate to be used for management*/ 
    IEEE80211_RESMGR_VAP_AIR_TIME_LIMIT,      /* When multi-channel enabled, restrict air-time allocated to a VAP */
#if UMAC_SUPPORT_RPTPLACEMENT                 /* Repeater placement assist feature */ 
    IEEE80211_RPT_CUSTPROTO_ENABLE,           /* Enable Custom Protocol bwn RootAP & RPT */
    IEEE80211_RPT_GPUTCALC_ENABLE,            /* Enable goodput calculator */
    IEEE80211_RPT_DEVUP,                      /* Indicate AP has sent out 1st beacon */ 
    IEEE80211_RPT_MACDEV,                     /* Indicate device whose MAC Address follows */
    IEEE80211_RPT_MACADDR1,                   /* Upper 4 bytes of device's MAC address */
    IEEE80211_RPT_MACADDR2,                   /* Lower 2 bytes of device's MAC address */ 
    IEEE80211_RPT_GPUTMODE,                   /* Indicates mode of operation of goodput calc */
    IEEE80211_RPT_TXPROTOMSG,                 /* Protocol message number to be transmitted */
    IEEE80211_RPT_RXPROTOMSG,                 /* Protocol message number received */
    IEEE80211_RPT_STATUS,                     /* Repeater placement training status */
    IEEE80211_RPT_ASSOC,                      /* Repeater placement association status */
    IEEE80211_RPT_NUMSTAS,                    /* Number of STAs used in training */
    IEEE80211_RPT_STA1ROUTE,                  /* Routing table for STA#1 */
    IEEE80211_RPT_STA2ROUTE,                  /* Routing table for STA#2 */
    IEEE80211_RPT_STA3ROUTE,                  /* Routing table for STA#3 */
    IEEE80211_RPT_STA4ROUTE,                  /* Routing table for STA#4 */
#endif //UMAC_SUPPORT_RPTPLACEMENT
    IEEE80211_TDLS_MACADDR1,                  /* Upper 4 bytes of device's MAC address */
    IEEE80211_TDLS_MACADDR2,                  /* Lower 2 bytes of device's MAC address */
    IEEE80211_TDLS_ACTION,                    /* TDLS action requested                 */ 

#ifdef  ATH_SUPPORT_AOW
    IEEE80211_AOW_SWRETRIES,
    IEEE80211_AOW_LATENCY,
    IEEE80211_AOW_PLAY_LOCAL,
    IEEE80211_AOW_CLEAR_AUDIO_CHANNELS,
    IEEE80211_AOW_STATS,
    IEEE80211_AOW_INTERLEAVE,
    IEEE80211_AOW_ER,
    IEEE80211_AOW_EC,
    IEEE80211_AOW_EC_FMAP,
    IEEE80211_AOW_ES,
    IEEE80211_AOW_ESS,
    IEEE80211_AOW_ESS_COUNT,
    IEEE80211_AOW_ESTATS,
    IEEE80211_AOW_ENABLE_CAPTURE,
    IEEE80211_AOW_FORCE_INPUT,
    IEEE80211_AOW_PRINT_CAPTURE,
    IEEE80211_AOW_LIST_AUDIO_CHANNELS,
#endif  /* ATH_SUPPORT_AOW */
    IEEE80211_AUTO_ASSOC,
    IEEE80211_MAX_CLIENT_NUMBERS,
} ieee80211_param;

#define  IEEE80211_PROTECTION_NONE         0
#define  IEEE80211_PROTECTION_CTSTOSELF    1
#define  IEEE80211_PROTECTION_RTS_CTS      2

typedef enum _ieee80211_privacy_filter {
    IEEE80211_PRIVACY_FILTER_ALLWAYS,
    IEEE80211_PRIVACY_FILTER_KEY_UNAVAILABLE,
} ieee80211_privacy_filter ;

typedef enum _ieee80211_privacy_filter_packet_type {
    IEEE80211_PRIVACY_FILTER_PACKET_UNICAST,
    IEEE80211_PRIVACY_FILTER_PACKET_MULTICAST,
    IEEE80211_PRIVACY_FILTER_PACKET_BOTH
} ieee80211_privacy_filter_packet_type ;

typedef struct _ieee80211_privacy_excemption_filter {
    u_int16_t                               ether_type; /* type of ethernet to apply this filter, in host byte order*/
    ieee80211_privacy_filter                filter_type; 
    ieee80211_privacy_filter_packet_type    packet_type;
} ieee80211_privacy_exemption;

/*
 * Authentication mode.
 * NB: the usage of auth modes NONE, AUTO are deprecated,
 * they are implemented through combinations of other auth modes
 * and cipher types. The deprecated values are preserved here to
 * maintain binary compatibility with applications like
 * wpa_supplicant and hostapd.
 */
typedef enum _ieee80211_auth_mode {
    IEEE80211_AUTH_NONE     = 0, /* deprecated */
    IEEE80211_AUTH_OPEN     = 1, /* open */
    IEEE80211_AUTH_SHARED   = 2, /* shared-key */
    IEEE80211_AUTH_8021X    = 3, /* 802.1x */
    IEEE80211_AUTH_AUTO     = 4, /* deprecated */
    IEEE80211_AUTH_WPA      = 5, /* WPA */
    IEEE80211_AUTH_RSNA     = 6, /* WPA2/RSNA */
    IEEE80211_AUTH_CCKM     = 7, /* CCK */
    IEEE80211_AUTH_WAPI     = 8, /* WAPI */
} ieee80211_auth_mode;

#define IEEE80211_AUTH_MAX      (IEEE80211_AUTH_WAPI+1)

/*
 * Cipher types.
 * NB: The values are preserved here to maintain binary compatibility
 * with applications like wpa_supplicant and hostapd.
 */
typedef enum _ieee80211_cipher_type {
    IEEE80211_CIPHER_WEP        = 0,
    IEEE80211_CIPHER_TKIP       = 1,
    IEEE80211_CIPHER_AES_OCB    = 2,
    IEEE80211_CIPHER_AES_CCM    = 3,
    IEEE80211_CIPHER_WAPI       = 4,
    IEEE80211_CIPHER_CKIP       = 5,
    IEEE80211_CIPHER_NONE       = 6,
} ieee80211_cipher_type;

#define IEEE80211_CIPHER_MAX    (IEEE80211_CIPHER_NONE+1)

/* key direction */
typedef enum _ieee80211_key_direction {
    IEEE80211_KEY_DIR_TX,
    IEEE80211_KEY_DIR_RX,
    IEEE80211_KEY_DIR_BOTH
} ieee80211_key_direction;

#define IEEE80211_KEYIX_NONE    ((u_int16_t) -1)

typedef struct _ieee80211_keyval {
    ieee80211_cipher_type   keytype;
    ieee80211_key_direction keydir;
    u_int                   persistent:1,   /* persistent key */
                            mfp:1;          /* management frame protection */
    u_int16_t               keylen;         /* length of the key data fields */
    u_int8_t                *macaddr;       /* mac address of length IEEE80211_ADDR_LEN . all bytes are 0xff for multicast key */
    u_int64_t               keyrsc;
    u_int64_t               keytsc;
    u_int16_t               txmic_offset;   /* TKIP/SMS4 only: offset to tx mic key */
    u_int16_t               rxmic_offset;   /* TKIP/SMS4 only: offset to rx mic key */
    u_int8_t                *keydata;
#if ATH_SUPPORT_WAPI
    u_int8_t                key_used;       /*index for WAPI rekey labeling*/
#endif    
} ieee80211_keyval;

typedef enum _ieee80211_rsn_param {
    IEEE80211_UCAST_CIPHER_LEN,
    IEEE80211_MCAST_CIPHER_LEN,
    IEEE80211_KEYMGT_ALGS,
    IEEE80211_RSN_CAPS
} ieee80211_rsn_param;

typedef enum _ieee80211_acl_cmd {
    IEEE80211_ACL_ADDMAC,
    IEEE80211_ACL_DELMAC,
    IEEE80211_ACL_POLICY_OPEN,
    IEEE80211_ACL_POLICY_ALLOW,
    IEEE80211_ACL_POLICY_DENY,
    IEEE80211_ACL_POLICY_RADIUS,
    IEEE80211_ACL_FLUSH,
    IEEE80211_ACL_DETACH
} ieee80211_acl_cmd;

#define IEEE80211_PMKID_LEN     16

typedef struct _ieee80211_pmkid_entry {
    u_int8_t    bssid[IEEE80211_ADDR_LEN];
    u_int8_t    pmkid[IEEE80211_PMKID_LEN];
} ieee80211_pmkid_entry;

typedef enum _wlan_wme_param {
    WLAN_WME_CWMIN,
    WLAN_WME_CWMAX,
    WLAN_WME_AIFS,
    WLAN_WME_TXOPLIMIT,
    WLAN_WME_ACM,      /*bss only*/
    WLAN_WME_ACKPOLICY /*bss only*/
} wlan_wme_param;

typedef enum _ieee80211_frame_type {
    IEEE80211_FRAME_TYPE_PROBEREQ,
    IEEE80211_FRAME_TYPE_BEACON,
    IEEE80211_FRAME_TYPE_PROBERESP,
    IEEE80211_FRAME_TYPE_ASSOCREQ,
    IEEE80211_FRAME_TYPE_ASSOCRESP
} ieee80211_frame_type; 

#define IEEE80211_FRAME_TYPE_MAX    (IEEE80211_FRAME_TYPE_ASSOCRESP+1)

typedef enum _ieee80211_ampdu_mode {
    IEEE80211_AMPDU_MODE_OFF,   /* disable AMPDU */
    IEEE80211_AMPDU_MODE_ON,    /* enable AMPDU */
    IEEE80211_AMPDU_MODE_WDSVAR /* enable AMPDU with 4addr WAR */
} ieee80211_ampdu_mode;

typedef enum _ieee80211_reset_type {
    IEEE80211_RESET_TYPE_DEVICE = 0,    /* device reset on error: tx timeout and etc. */
    IEEE80211_RESET_TYPE_DOT11_INTF,    /* dot11 reset: only reset one network interface (vap) */
    IEEE80211_RESET_TYPE_INTERNAL,      /* internal reset */
} ieee80211_reset_type;

typedef struct _ieee80211_reset_request {
    ieee80211_reset_type    type;

    u_int                   reset_hw:1,         /* reset the actual H/W */
                            /*
                             * The following fields are only valid for DOT11 reset, i.e.,
                             * IEEE80211_RESET_TYPE_DOT11_INTF
                             */
                            reset_phy:1,        /* reset PHY */
                            reset_mac:1,        /* reset MAC */
                            set_default_mib:1,  /* set default MIB variables */
                            no_flush:1;
    u_int8_t                macaddr[IEEE80211_ADDR_LEN];
    enum ieee80211_phymode  phy_mode;
} ieee80211_reset_request;

#define IEEE80211_MSG_MLME      0x80000000      /* MLME */
#define IEEE80211_MSG_DEBUG     0x40000000      /* IFF_DEBUG equivalent */
#define IEEE80211_MSG_DUMPPKTS  0x20000000      /* IFF_LINK2 equivalant */
#define IEEE80211_MSG_CRYPTO    0x10000000      /* crypto work */
#define IEEE80211_MSG_INPUT     0x08000000      /* input handling */
#define IEEE80211_MSG_XRATE     0x04000000      /* rate set handling */
#define IEEE80211_MSG_ELEMID    0x02000000      /* element id parsing */
#define IEEE80211_MSG_NODE      0x01000000      /* node handling */
#define IEEE80211_MSG_ASSOC     0x00800000      /* association handling */
#define IEEE80211_MSG_AUTH      0x00400000      /* authentication handling */
#define IEEE80211_MSG_SCAN      0x00200000      /* scanning */
#define IEEE80211_MSG_OUTPUT    0x00100000      /* output handling */
#define IEEE80211_MSG_STATE     0x00080000      /* state machine */
#define IEEE80211_MSG_POWER     0x00040000      /* power save handling */
#define IEEE80211_MSG_DOT1X     0x00020000      /* 802.1x authenticator */
#define IEEE80211_MSG_DOT1XSM   0x00010000      /* 802.1x state machine */
#define IEEE80211_MSG_RADIUS    0x00008000      /* 802.1x radius client */
#define IEEE80211_MSG_RADDUMP   0x00004000      /* dump 802.1x radius packets */
#define IEEE80211_MSG_RADKEYS   0x00002000      /* dump 802.1x keys */
#define IEEE80211_MSG_WPA       0x00001000      /* WPA/RSN protocol */
#define IEEE80211_MSG_ACL       0x00000800      /* ACL handling */
#define IEEE80211_MSG_WME       0x00000400      /* WME protocol */
#define IEEE80211_MSG_IQUE      0x00000200      /* IQUE features */
#define IEEE80211_MSG_DOTH      0x00000100      /* 11.h */
#define IEEE80211_MSG_INACT     0x00000080      /* inactivity handling */
#define IEEE80211_MSG_ROAM      0x00000040      /* sta-mode roaming */
#define IEEE80211_MSG_ACTION    0x00000020      /* action management frames */
#define IEEE80211_MSG_WDS       0x00000010      /* WDS handling */
#define IEEE80211_MSG_SCANENTRY 0x00000008      /* scan entry */
#define IEEE80211_MSG_SCAN_SM   0x00000004      /* scan state machine */
#define IEEE80211_MSG_ACS       0x00000002      /* auto channel selection */
#define IEEE80211_MSG_TDLS      0x00000001      /* TDLS */
#define IEEE80211_MSG_ANY       0xffffffff      /* anything */

/* verbosity levels */
#define     IEEE80211_VERBOSE_OFF                  100 
#define     IEEE80211_VERBOSE_FORCE               1
#define     IEEE80211_VERBOSE_SERIOUS             2
#define     IEEE80211_VERBOSE_NORMAL              3
#define     IEEE80211_VERBOSE_LOUD                4
#define     IEEE80211_VERBOSE_DETAILED            5
#define     IEEE80211_VERBOSE_COMPLEX             6
#define     IEEE80211_VERBOSE_FUNCTION            7 
#define     IEEE80211_VERBOSE_TRACE               8

#define IEEE80211_DEBUG_DEFAULT IEEE80211_MSG_DEBUG     

/*
 * the lower 4 bits of the msg flags are used for extending the
 * debug flags. 
 */ 

/*
 * flag defintions for wlan_mlme_stop_bss(vap) API.
 */
#define WLAN_MLME_STOP_BSS_F_SEND_DEAUTH                0x01
#define WLAN_MLME_STOP_BSS_F_CLEAR_ASSOC_STATE          0x02
#define WLAN_MLME_STOP_BSS_F_FORCE_STOP_RESET           0x04
#define WLAN_MLME_STOP_BSS_F_WAIT_RX_DONE               0x08
#define WLAN_MLME_STOP_BSS_F_NO_RESET                   0x10
#define WLAN_MLME_STOP_BSS_F_STANDBY                    0x20

/*
 * WAPI commands to authenticator
 */
#define WAPI_WAI_REQUEST            (u_int16_t)0x00F1
#define WAPI_UNICAST_REKEY          (u_int16_t)0x00F2
#define WAPI_STA_AGING              (u_int16_t)0x00F3
#define WAPI_MUTIL_REKEY            (u_int16_t)0x00F4
#define WAPI_STA_STATS              (u_int16_t)0x00F5

/*
 * IEEE80211 PHY Statistics.
 */
struct ieee80211_phy_stats {
    u_int64_t   ips_tx_packets;      /* frames successfully transmitted */
    u_int64_t   ips_tx_multicast;    /* multicast/broadcast frames successfully transmitted */
    u_int64_t   ips_tx_fragments;    /* fragments successfully transmitted */
    u_int64_t   ips_tx_xretries;     /* frames that are xretried. NB: not number of retries */
    u_int64_t   ips_tx_retries;      /* frames transmitted after retries. NB: not number of retries */
    u_int64_t   ips_tx_multiretries; /* frames transmitted after more than one retry. */
    u_int64_t   ips_tx_timeout;      /* frames that expire the dot11MaxTransmitMSDULifetime */
    u_int64_t   ips_rx_packets;      /* frames successfully received */
    u_int64_t   ips_rx_multicast;    /* multicast/broadcast frames successfully received */
    u_int64_t   ips_rx_fragments;    /* fragments successfully received */
    u_int64_t   ips_rx_timeout;      /* frmaes that expired the dot11MaxReceiveLifetime */
    u_int64_t   ips_rx_dup;          /* duplicated fragments */
    u_int64_t   ips_rx_mdup;         /* multiple duplicated fragments */
    u_int64_t   ips_rx_promiscuous;  /* frames that are received only because promiscuous filter is on */
    u_int64_t   ips_rx_promiscuous_fragments; /* fragments that are received only because promiscuous filter is on */
    u_int64_t   ips_tx_rts;          /* RTS success count */
    u_int64_t   ips_tx_shortretry;   /* tx on-chip retries (short). RTSFailCnt */
    u_int64_t   ips_tx_longretry;    /* tx on-chip retries (long). DataFailCnt */
    u_int64_t   ips_rx_crcerr;       /* rx failed 'cuz of bad CRC */
    u_int64_t   ips_rx_fifoerr;      /* rx failed 'cuz of FIFO overrun */
    u_int64_t   ips_rx_decrypterr;   /* rx decryption error */
};

struct ieee80211_mac_stats {
    u_int64_t   ims_tx_packets; /* frames successfully transmitted */
    u_int64_t   ims_rx_packets; /* frames successfully received */
    u_int64_t   ims_tx_bytes;	/* bytes successfully transmitted */
    u_int64_t	ims_rx_bytes;   /* bytes successfully received */

    /* Decryption errors */
    u_int64_t   ims_rx_unencrypted; /* rx w/o wep and privacy on */
    u_int64_t   ims_rx_badkeyid;    /* rx w/ incorrect keyid */
    u_int64_t   ims_rx_decryptok;   /* rx decrypt okay */
    u_int64_t   ims_rx_decryptcrc;  /* rx decrypt failed on crc */
    u_int64_t   ims_rx_wepfail;     /* rx wep processing failed */
    u_int64_t   ims_rx_tkipreplay;  /* rx seq# violation (TKIP) */
    u_int64_t   ims_rx_tkipformat;  /* rx format bad (TKIP) */
    u_int64_t   ims_rx_tkipmic;     /* rx MIC check failed (TKIP) */
    u_int64_t   ims_rx_tkipicv;     /* rx ICV check failed (TKIP) */
    u_int64_t   ims_rx_ccmpreplay;  /* rx seq# violation (CCMP) */
    u_int64_t   ims_rx_ccmpformat;  /* rx format bad (CCMP) */
    u_int64_t   ims_rx_ccmpmic;     /* rx MIC check failed (CCMP) */

    /* Other Tx/Rx errors */
    u_int64_t   ims_tx_discard;     /* tx dropped by NIC */
    u_int64_t   ims_rx_discard;     /* rx dropped by NIC */

    u_int64_t   ims_rx_countermeasure; /* rx TKIP countermeasure activation count */
};

/*
 * Summary statistics.
 */
struct ieee80211_stats {
    u_int32_t   is_rx_badversion;          /* rx frame with bad version */
    u_int32_t   is_rx_tooshort;            /* rx frame too short */
    u_int32_t   is_rx_wrongbss;            /* rx from wrong bssid */
    u_int32_t   is_rx_wrongdir;            /* rx w/ wrong direction */
    u_int32_t   is_rx_mcastecho;           /* rx discard 'cuz mcast echo */
    u_int32_t   is_rx_notassoc;            /* rx discard 'cuz sta !assoc */
    u_int32_t   is_rx_noprivacy;           /* rx w/ wep but privacy off */
    u_int32_t   is_rx_decap;               /* rx decapsulation failed */
    u_int32_t   is_rx_mgtdiscard;          /* rx discard mgt frames */
    u_int32_t   is_rx_ctl;                 /* rx discard ctrl frames */
    u_int32_t   is_rx_beacon;              /* rx beacon frames */
    u_int32_t   is_rx_rstoobig;            /* rx rate set truncated */
    u_int32_t   is_rx_elem_missing;        /* rx required element missing*/
    u_int32_t   is_rx_elem_toobig;         /* rx element too big */
    u_int32_t   is_rx_elem_toosmall;       /* rx element too small */
    u_int32_t   is_rx_elem_unknown;        /* rx element unknown */
    u_int32_t   is_rx_badchan;             /* rx frame w/ invalid chan */
    u_int32_t   is_rx_chanmismatch;        /* rx frame chan mismatch */
    u_int32_t   is_rx_nodealloc;           /* rx frame dropped */
    u_int32_t   is_rx_ssidmismatch;        /* rx frame ssid mismatch  */
    u_int32_t   is_rx_auth_unsupported;    /* rx w/ unsupported auth alg */
    u_int32_t   is_rx_auth_fail;           /* rx sta auth failure */
    u_int32_t   is_rx_auth_countermeasures;/* rx auth discard 'cuz CM */
    u_int32_t   is_rx_assoc_bss;           /* rx assoc from wrong bssid */
    u_int32_t   is_rx_assoc_notauth;       /* rx assoc w/o auth */
    u_int32_t   is_rx_assoc_capmismatch;   /* rx assoc w/ cap mismatch */
    u_int32_t   is_rx_assoc_norate;        /* rx assoc w/ no rate match */
    u_int32_t   is_rx_assoc_badwpaie;      /* rx assoc w/ bad WPA IE */
    u_int32_t   is_rx_deauth;              /* rx deauthentication */
    u_int32_t   is_rx_disassoc;            /* rx disassociation */
    u_int32_t   is_rx_action;              /* rx action mgt */
    u_int32_t   is_rx_badsubtype;          /* rx frame w/ unknown subtype*/
    u_int32_t   is_rx_nobuf;               /* rx failed for lack of buf */
    u_int32_t   is_rx_ahdemo_mgt;          /* rx discard ahdemo mgt frame*/
    u_int32_t   is_rx_bad_auth;            /* rx bad auth request */
    u_int32_t   is_rx_unauth;              /* rx on unauthorized port */
    u_int32_t   is_rx_badcipher;           /* rx failed 'cuz key type */
    u_int32_t   is_tx_nodefkey;            /* tx failed 'cuz no defkey */
    u_int32_t   is_tx_noheadroom;          /* tx failed 'cuz no space */
    u_int32_t   is_rx_nocipherctx;         /* rx failed 'cuz key !setup */
    u_int32_t   is_rx_acl;                 /* rx discard 'cuz acl policy */
    u_int32_t   is_rx_ffcnt;               /* rx fast frames */
    u_int32_t   is_rx_badathtnl;           /* driver key alloc failed */
    u_int32_t   is_rx_nowds;               /* 4-addr packets received with no wds enabled */
    u_int32_t   is_tx_nobuf;               /* tx failed for lack of buf */
    u_int32_t   is_tx_nonode;              /* tx failed for no node */
    u_int32_t   is_tx_unknownmgt;          /* tx of unknown mgt frame */
    u_int32_t   is_tx_badcipher;           /* tx failed 'cuz key type */
    u_int32_t   is_tx_ffokcnt;             /* tx fast frames sent success */
    u_int32_t   is_tx_fferrcnt;            /* tx fast frames sent success */
    u_int32_t   is_scan_active;            /* active scans started */
    u_int32_t   is_scan_passive;           /* passive scans started */
    u_int32_t   is_node_timeout;           /* nodes timed out inactivity */
    u_int32_t   is_crypto_nomem;           /* no memory for crypto ctx */
    u_int32_t   is_crypto_tkip;            /* tkip crypto done in s/w */
    u_int32_t   is_crypto_tkipenmic;       /* tkip en-MIC done in s/w */
    u_int32_t   is_crypto_tkipdemic;       /* tkip de-MIC done in s/w */
    u_int32_t   is_crypto_tkipcm;          /* tkip counter measures */
    u_int32_t   is_crypto_ccmp;            /* ccmp crypto done in s/w */
    u_int32_t   is_crypto_wep;             /* wep crypto done in s/w */
    u_int32_t   is_crypto_setkey_cipher;   /* cipher rejected key */
    u_int32_t   is_crypto_setkey_nokey;    /* no key index for setkey */
    u_int32_t   is_crypto_delkey;          /* driver key delete failed */
    u_int32_t   is_crypto_badcipher;       /* unknown cipher */
    u_int32_t   is_crypto_nocipher;        /* cipher not available */
    u_int32_t   is_crypto_attachfail;      /* cipher attach failed */
    u_int32_t   is_crypto_swfallback;      /* cipher fallback to s/w */
    u_int32_t   is_crypto_keyfail;         /* driver key alloc failed */
    u_int32_t   is_crypto_enmicfail;       /* en-MIC failed */
    u_int32_t   is_ibss_capmismatch;       /* merge failed-cap mismatch */
    u_int32_t   is_ibss_norate;            /* merge failed-rate mismatch */
    u_int32_t   is_ps_unassoc;             /* ps-poll for unassoc. sta */
    u_int32_t   is_ps_badaid;              /* ps-poll w/ incorrect aid */
    u_int32_t   is_ps_qempty;              /* ps-poll w/ nothing to send */
};

typedef enum _ieee80211_send_frame_type {
    IEEE80211_SEND_NULL,
    IEEE80211_SEND_QOSNULL,
} ieee80211_send_frame_type;

typedef struct _ieee80211_tspec_info {
    u_int8_t    traffic_type;
    u_int8_t    direction;
    u_int8_t    dot1Dtag;
    u_int8_t    acc_policy_edca;
    u_int8_t    acc_policy_hcca;
    u_int8_t    aggregation;
    u_int8_t    psb;
    u_int8_t    ack_policy;
    u_int16_t   norminal_msdu_size;
    u_int16_t   max_msdu_size;
    u_int32_t   min_srv_interval;
    u_int32_t   max_srv_interval;
    u_int32_t   inactivity_interval;
    u_int32_t   suspension_interval;
    u_int32_t   srv_start_time;
    u_int32_t   min_data_rate;
    u_int32_t   mean_data_rate;
    u_int32_t   peak_data_rate;
    u_int32_t   max_burst_size;
    u_int32_t   delay_bound;
    u_int32_t   min_phy_rate;
    u_int16_t   surplus_bw;
} ieee80211_tspec_info;

#ifndef EXTERNAL_USE_ONLY
/*
 * Manual ADDBA support
 */
enum {
    ADDBA_SEND     = 0,
    ADDBA_STATUS   = 1,
    DELBA_SEND     = 2,
    ADDBA_RESP     = 3,
    ADDBA_CLR_RESP = 4,
};

enum {
    ADDBA_MODE_AUTO   = 0,
    ADDBA_MODE_MANUAL = 1,
};

struct ieee80211_addba_delba_request {
    wlan_dev_t             ic;
    u_int8_t               action;
    u_int8_t               tid;
    u_int16_t              status;
    u_int16_t              aid;
    u_int32_t              arg1;
    u_int32_t              arg2;
};
#endif /* EXTERNAL_USE_ONLY */

#ifdef ATH_BT_COEX
typedef enum _ieee80211_bt_coex_info_type {
    IEEE80211_BT_COEX_INFO_SCHEME        = 0,
    IEEE80211_BT_COEX_INFO_BTBUSY        = 1,
} ieee80211_bt_coex_info_type;
#endif

struct tkip_countermeasure {
    u_int16_t   mic_count_in_60s;
    u_int32_t   timestamp;
} ;

enum _ieee80211_qos_frame_direction {
    IEEE80211_RX_QOS_FRAME = 0,
    IEEE80211_TX_QOS_FRAME = 1,
    IEEE80211_TX_COMPLETE_QOS_FRAME = 2
};

#endif
