/*
 * hostapd / UNIX domain socket -based control interface
 * Copyright (c) 2004-2010, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "utils/includes.h"

#ifndef CONFIG_NATIVE_WINDOWS

#include <sys/un.h>
#include <sys/stat.h>
#include <stddef.h>

#include "utils/common.h"
#include "utils/eloop.h"
#include "common/version.h"
#include "common/ieee802_11_defs.h"
#include "drivers/driver.h"
#include "radius/radius_client.h"
#include "ap/hostapd.h"
#include "ap/ap_config.h"
#include "ap/ieee802_1x.h"
#include "ap/wpa_auth.h"
#include "ap/ieee802_11.h"
#include "ap/sta_info.h"
#include "ap/wps_hostapd.h"
#include "ap/ctrl_iface_ap.h"
#include "ap/ap_drv_ops.h"
#include "ap/beacon.h"
#include "ap/gas_serv.h"
#include "wps/wps_defs.h"
#include "wps/wps.h"
#include "config_file.h"
#include "ctrl_iface.h"


struct wpa_ctrl_dst {
	struct wpa_ctrl_dst *next;
	struct sockaddr_un addr;
	socklen_t addrlen;
	int debug_level;
	int errors;
	int anqp;
};


static void hostapd_ctrl_iface_send(struct hostapd_data *hapd, int level,
				    const char *buf, size_t len);


static int hostapd_ctrl_iface_attach(struct hostapd_data *hapd,
				     struct sockaddr_un *from,
				     socklen_t fromlen, int anqp)
{
	struct wpa_ctrl_dst *dst;

	dst = os_zalloc(sizeof(*dst));
	if (dst == NULL)
		return -1;
	os_memcpy(&dst->addr, from, sizeof(struct sockaddr_un));
	dst->addrlen = fromlen;
	if (anqp) {
		dst->anqp = anqp;
		dst->debug_level = 0xffff;
	} else
		dst->debug_level = MSG_INFO;
	dst->next = hapd->ctrl_dst;
	hapd->ctrl_dst = dst;
	wpa_hexdump(MSG_DEBUG, "CTRL_IFACE monitor attached",
		    (u8 *) from->sun_path,
		    fromlen - offsetof(struct sockaddr_un, sun_path));
	return 0;
}


static int hostapd_ctrl_iface_detach(struct hostapd_data *hapd,
				     struct sockaddr_un *from,
				     socklen_t fromlen)
{
	struct wpa_ctrl_dst *dst, *prev = NULL;

	dst = hapd->ctrl_dst;
	while (dst) {
		if (fromlen == dst->addrlen &&
		    os_memcmp(from->sun_path, dst->addr.sun_path,
			      fromlen - offsetof(struct sockaddr_un, sun_path))
		    == 0) {
			if (prev == NULL)
				hapd->ctrl_dst = dst->next;
			else
				prev->next = dst->next;
			os_free(dst);
			wpa_hexdump(MSG_DEBUG, "CTRL_IFACE monitor detached",
				    (u8 *) from->sun_path,
				    fromlen -
				    offsetof(struct sockaddr_un, sun_path));
			return 0;
		}
		prev = dst;
		dst = dst->next;
	}
	return -1;
}


#ifdef CONFIG_HS20
static int hostapd_ctrl_iface_is_anqpsock(struct hostapd_data *hapd,
					  struct sockaddr_un *from,
					  socklen_t fromlen)
{
	struct wpa_ctrl_dst *dst, *prev = NULL;

	dst = hapd->ctrl_dst;
	while (dst) {
		if (fromlen == dst->addrlen &&
		    os_memcmp(from->sun_path, dst->addr.sun_path,
			      fromlen - offsetof(struct sockaddr_un, sun_path))
		    == 0 && dst->anqp)
			return 1;
		prev = dst;
		dst = dst->next;
	}
	return 0;
}
#endif /* CONFIG_HS20 */


static int hostapd_ctrl_iface_level(struct hostapd_data *hapd,
				    struct sockaddr_un *from,
				    socklen_t fromlen,
				    char *level)
{
	struct wpa_ctrl_dst *dst;

	wpa_printf(MSG_DEBUG, "CTRL_IFACE LEVEL %s", level);

	dst = hapd->ctrl_dst;
	while (dst) {
		if (fromlen == dst->addrlen &&
		    os_memcmp(from->sun_path, dst->addr.sun_path,
			      fromlen - offsetof(struct sockaddr_un, sun_path))
		    == 0) {
			wpa_hexdump(MSG_DEBUG, "CTRL_IFACE changed monitor "
				    "level", (u8 *) from->sun_path, fromlen -
				    offsetof(struct sockaddr_un, sun_path));
			dst->debug_level = atoi(level);
			return 0;
		}
		dst = dst->next;
	}

	return -1;
}


static int hostapd_ctrl_iface_new_sta(struct hostapd_data *hapd,
				      const char *txtaddr)
{
	u8 addr[ETH_ALEN];
	struct sta_info *sta;

	wpa_printf(MSG_DEBUG, "CTRL_IFACE NEW_STA %s", txtaddr);

	if (hwaddr_aton(txtaddr, addr))
		return -1;

	sta = ap_get_sta(hapd, addr);
	if (sta)
		return 0;

	wpa_printf(MSG_DEBUG, "Add new STA " MACSTR " based on ctrl_iface "
		   "notification", MAC2STR(addr));
	sta = ap_sta_add(hapd, addr);
	if (sta == NULL)
		return -1;

	hostapd_new_assoc_sta(hapd, sta, 0);
	return 0;
}


#ifdef CONFIG_IEEE80211W
#ifdef NEED_AP_MLME
static int hostapd_ctrl_iface_sa_query(struct hostapd_data *hapd,
				       const char *txtaddr)
{
	u8 addr[ETH_ALEN];
	u8 trans_id[WLAN_SA_QUERY_TR_ID_LEN];

	wpa_printf(MSG_DEBUG, "CTRL_IFACE SA_QUERY %s", txtaddr);

	if (hwaddr_aton(txtaddr, addr) ||
	    os_get_random(trans_id, WLAN_SA_QUERY_TR_ID_LEN) < 0)
		return -1;

	ieee802_11_send_sa_query_req(hapd, addr, trans_id);

	return 0;
}
#endif /* NEED_AP_MLME */
#endif /* CONFIG_IEEE80211W */


#ifdef CONFIG_WPS
static int hostapd_ctrl_iface_wps_pin(struct hostapd_data *hapd, char *txt)
{
	char *pin = os_strchr(txt, ' ');
	char *timeout_txt;
	int timeout;
	u8 addr_buf[ETH_ALEN], *addr = NULL;
	char *pos;

	if (pin == NULL)
		return -1;
	*pin++ = '\0';

	timeout_txt = os_strchr(pin, ' ');
	if (timeout_txt) {
		*timeout_txt++ = '\0';
		timeout = atoi(timeout_txt);
		pos = os_strchr(timeout_txt, ' ');
		if (pos) {
			*pos++ = '\0';
			if (hwaddr_aton(pos, addr_buf) == 0)
				addr = addr_buf;
		}
	} else
		timeout = 0;

	return hostapd_wps_add_pin(hapd, addr, txt, pin, timeout);
}


static int hostapd_ctrl_iface_wps_check_pin(
	struct hostapd_data *hapd, char *cmd, char *buf, size_t buflen)
{
	char pin[9];
	size_t len;
	char *pos;
	int ret;

	wpa_hexdump_ascii_key(MSG_DEBUG, "WPS_CHECK_PIN",
			      (u8 *) cmd, os_strlen(cmd));
	for (pos = cmd, len = 0; *pos != '\0'; pos++) {
		if (*pos < '0' || *pos > '9')
			continue;
		pin[len++] = *pos;
		if (len == 9) {
			wpa_printf(MSG_DEBUG, "WPS: Too long PIN");
			return -1;
		}
	}
	if (len != 4 && len != 8) {
		wpa_printf(MSG_DEBUG, "WPS: Invalid PIN length %d", (int) len);
		return -1;
	}
	pin[len] = '\0';

	if (len == 8) {
		unsigned int pin_val;
		pin_val = atoi(pin);
		if (!wps_pin_valid(pin_val)) {
			wpa_printf(MSG_DEBUG, "WPS: Invalid checksum digit");
			ret = os_snprintf(buf, buflen, "FAIL-CHECKSUM\n");
			if (ret < 0 || (size_t) ret >= buflen)
				return -1;
			return ret;
		}
	}

	ret = os_snprintf(buf, buflen, "%s", pin);
	if (ret < 0 || (size_t) ret >= buflen)
		return -1;

	return ret;
}


#ifdef CONFIG_WPS_OOB
static int hostapd_ctrl_iface_wps_oob(struct hostapd_data *hapd, char *txt)
{
	char *path, *method, *name;

	path = os_strchr(txt, ' ');
	if (path == NULL)
		return -1;
	*path++ = '\0';

	method = os_strchr(path, ' ');
	if (method == NULL)
		return -1;
	*method++ = '\0';

	name = os_strchr(method, ' ');
	if (name != NULL)
		*name++ = '\0';

	return hostapd_wps_start_oob(hapd, txt, path, method, name);
}
#endif /* CONFIG_WPS_OOB */


static int hostapd_ctrl_iface_wps_ap_pin(struct hostapd_data *hapd, char *txt,
					 char *buf, size_t buflen)
{
	int timeout = 300;
	char *pos;
	const char *pin_txt;

	pos = os_strchr(txt, ' ');
	if (pos)
		*pos++ = '\0';

	if (os_strcmp(txt, "disable") == 0) {
		hostapd_wps_ap_pin_disable(hapd);
		return os_snprintf(buf, buflen, "OK\n");
	}

	if (os_strcmp(txt, "random") == 0) {
		if (pos)
			timeout = atoi(pos);
		pin_txt = hostapd_wps_ap_pin_random(hapd, timeout);
		if (pin_txt == NULL)
			return -1;
		return os_snprintf(buf, buflen, "%s", pin_txt);
	}

	if (os_strcmp(txt, "get") == 0) {
		pin_txt = hostapd_wps_ap_pin_get(hapd);
		if (pin_txt == NULL)
			return -1;
		return os_snprintf(buf, buflen, "%s", pin_txt);
	}

	if (os_strcmp(txt, "set") == 0) {
		char *pin;
		if (pos == NULL)
			return -1;
		pin = pos;
		pos = os_strchr(pos, ' ');
		if (pos) {
			*pos++ = '\0';
			timeout = atoi(pos);
		}
		if (os_strlen(pin) > buflen)
			return -1;
		if (hostapd_wps_ap_pin_set(hapd, pin, timeout) < 0)
			return -1;
		return os_snprintf(buf, buflen, "%s", pin);
	}

	return -1;
}


static int hostapd_ctrl_iface_wps_config(struct hostapd_data *hapd, char *txt)
{
	char *pos;
	char *ssid, *auth, *encr = NULL, *key = NULL;

	ssid = txt;
	pos = os_strchr(txt, ' ');
	if (!pos)
		return -1;
	*pos++ = '\0';

	auth = pos;
	pos = os_strchr(pos, ' ');
	if (pos) {
		*pos++ = '\0';
		encr = pos;
		pos = os_strchr(pos, ' ');
		if (pos) {
			*pos++ = '\0';
			key = pos;
		}
	}

	return hostapd_wps_config_ap(hapd, ssid, auth, encr, key);
}


static int hostapd_ctrl_iface_wps_get_config(struct hostapd_data *hapd,
					     char *buf, size_t buflen)
{
	int ret;
	char *pos, *end, str[255], strval[255];
	int wpa = 0, auth_alg = 0;
	size_t i;
	char *prefix;
	char wpa_key_mgmt[32], wpa_pairwise[32], wpa_psk[255];

	pos = buf;
	end = buf + buflen;

	os_memset(buf, '\0', buflen);
	os_memset(str, '\0', sizeof(str));
	os_memset(strval, '\0', sizeof(strval));
	os_memset(wpa_pairwise, '\0', sizeof(wpa_pairwise));
	os_memset(wpa_key_mgmt, '\0', sizeof(wpa_key_mgmt));
	os_memset(wpa_psk, '\0', sizeof(wpa_psk));
	os_memset(buf, '\0', buflen);
	ret = sprintf(str, "wps_state=%d\n", hapd->wps->wps_state);
	if (ret >= 0 && buflen > (size_t) ret) {
		strcat(buf, str);
		buflen -= ret;
	} else
		return -1;
	ret = sprintf(str, "ssid=%s\n", hapd->wps->ssid);
	if (ret >= 0 && buflen > (size_t) ret) {
		strcat(buf, str);
		buflen -= ret;
	} else
		return -1;
	if (hapd->wps->auth_types & (WPS_AUTH_WPA2 | WPS_AUTH_WPA2PSK))
		wpa |= 2;
	if (hapd->wps->auth_types & (WPS_AUTH_WPA | WPS_AUTH_WPAPSK))
		wpa |= 1;
	ret = sprintf(str, "wpa=%d\n", wpa);
	if (ret >= 0 && buflen > (size_t) ret) {
		strcat(buf, str);
		buflen -= ret;
	} else
		return -1;
	prefix = "";
	if (hapd->wps->auth_types & (WPS_AUTH_WPA2 | WPS_AUTH_WPA)) {
		sprintf(strval, "WPA-EAP");
		strcat(wpa_key_mgmt, strval);
		prefix = " ";
	}
	if (hapd->wps->auth_types & (WPS_AUTH_WPA2PSK | WPS_AUTH_WPAPSK)) {
		sprintf(strval, "%sWPA-PSK", prefix);
		strcat(wpa_key_mgmt, strval);
	}
	ret = sprintf(str, "wpa_key_mgmt=%s\n", wpa_key_mgmt);
	if (ret >= 0 && buflen > (size_t) ret) {
		strcat(buf, str);
		buflen -= ret;
	} else
		return -1;
	if (hapd->wps->auth_types & WPS_AUTH_OPEN)
		auth_alg |= WPA_AUTH_ALG_OPEN;
	if (hapd->wps->auth_types & WPS_AUTH_SHARED)
		auth_alg |= WPA_AUTH_ALG_SHARED;
	ret = sprintf(str, "auth_alg=%d\n", auth_alg);
	if (ret >= 0 && buflen > (size_t) ret) {
		strcat(buf, str);
		buflen -= ret;
	} else
		return -1;

	prefix = "";
	if (hapd->wps->encr_types & WPS_ENCR_AES) {
		sprintf(strval, "CCMP");
		strcat(wpa_pairwise, strval);
		prefix = " ";
	}
	if (hapd->wps->encr_types & WPS_ENCR_TKIP) {
		sprintf(strval, "%sTKIP", prefix);
		strcat(wpa_pairwise, strval);
	}
	ret = sprintf(str, "wpa_pairwise=%s\n", wpa_pairwise);
	if (ret >= 0 && buflen > (size_t) ret) {
		strcat(buf, str);
		buflen -= ret;
	} else
		return -1;
	if (hapd->wps->auth_types & (WPS_AUTH_WPA2PSK | WPS_AUTH_WPAPSK)) {
		if (hapd->wps->network_key_len >= 8 &&
		    hapd->wps->network_key_len < 64) {
			os_snprintf(wpa_psk,hapd->wps->network_key_len + 1,
				    "%s", hapd->wps->network_key);
			ret = sprintf(str, "wpa_passphrase=%s", wpa_psk);
			if (ret >= 0 && buflen > (size_t) ret) {
				strcat(buf, str);
				buflen -= ret;
			} else
				return -1;
		} else if (hapd->wps->network_key_len == 64) {
			os_snprintf(wpa_psk,hapd->wps->network_key_len,
				    "%s", hapd->wps->network_key);
			ret = sprintf(str, "wpa_psk=%s", wpa_psk);
			if (ret >= 0 && buflen > (size_t) ret) {
				strcat(buf, str);
				buflen -= ret;
			} else
				return -1;
		}
	}
	return os_strlen(buf);
}
#endif /* CONFIG_WPS */

#ifdef CONFIG_HS20

static void hostapd_ctrl_iface_hs20_process_ie(struct hostapd_data *hapd,
					       const u8 *sta_addr,
					       struct gas_dialog_info *hs20)
{
	unsigned int check_bits =
		ANQP_REQ_VENUE_NAME |
		ANQP_REQ_NETWORK_AUTH_TYPE |
		ANQP_REQ_ROAMING_CONSORTIUM |
		ANQP_REQ_IP_ADDR_TYPE_AVAILABILITY |
		ANQP_REQ_NAI_REALM |
		ANQP_REQ_3GPP_CELLULAR_NETWORK |
		ANQP_REQ_DOMAIN_NAME |
		ANQP_REQ_OPERATOR_FRIENDLY_NAME |
		ANQP_REQ_WAN_METRICS |
		ANQP_REQ_CONNECTION_CAPABILITY |
		ANQP_REQ_NAI_HOME_REALM;

	if (hs20->comeback_delay)
		return;

	if ((hs20->requested & check_bits) ==
	    (hs20->requested & hs20->received & check_bits))
		gas_serv_tx_gas_response(hapd, sta_addr, hs20);
}


static int hostapd_ctrl_iface_anqp_detach(struct hostapd_data *hapd,
					  struct sockaddr_un *from,
					  socklen_t fromlen)
{
	int ret;
	struct wpa_ctrl_dst *dst;

	ret = hostapd_ctrl_iface_detach(hapd, from, fromlen);
	if (ret < 0)
		return ret;

	for (dst = hapd->ctrl_dst; dst; dst = dst->next) {
		if (dst->anqp)
			break;
	}

	if (dst)
		return 0; /* Another ANQP socket remains */

	wpa_printf(MSG_DEBUG, "ANQP: Unregister external ANQP processing");
	hapd->anqp_type_mask = 0;
	return 0;
}


static int hostapd_ctrl_iface_anqp_attach(struct hostapd_data *hapd,
					  const char *data, int len,
					  struct sockaddr_un *from,
					  socklen_t fromlen)
{
	int ret = 0;
	u8 type[4], delay[2];
	u32 anqp_type;
	u16 anqp_delay;

	if (len % 12)
		return -1;

	ret = hostapd_ctrl_iface_attach(hapd, from, fromlen, 1);
	if (ret)
		return ret;

	hapd->anqp_type_mask = 0;

	while (len) {
		if (hexstr2bin(data, type, 4) < 0 ||
		    hexstr2bin(data + 8, delay, 2) < 0) {
			wpa_printf(MSG_DEBUG, "Invalid hex substr %s", data);
			return 1;
		}
		anqp_type = WPA_GET_BE32(type);
		anqp_delay = WPA_GET_BE16(delay);
		wpa_printf(MSG_DEBUG, "Registering anqp_type 0x%x with "
			   "anqp_delay 0x%x", anqp_type, anqp_delay);
		switch (anqp_type) {
		case ANQP_VENUE_NAME:
			hapd->anqp_type_mask |= ANQP_REQ_VENUE_NAME;
			hapd->venue_name_delay = anqp_delay;
			break;
		case ANQP_NETWORK_AUTH_TYPE:
			hapd->anqp_type_mask |=
				ANQP_REQ_NETWORK_AUTH_TYPE;
			hapd->network_auth_type_delay = anqp_delay;
			break;
		case ANQP_ROAMING_CONSORTIUM:
			hapd->anqp_type_mask |=
				ANQP_REQ_ROAMING_CONSORTIUM;
			hapd->roaming_consortium_list_delay = anqp_delay;
			break;
		case ANQP_IP_ADDR_TYPE_AVAILABILITY:
			hapd->anqp_type_mask |=
				ANQP_REQ_IP_ADDR_TYPE_AVAILABILITY;
			hapd->ipaddr_type_delay = anqp_delay;
			break;
		case ANQP_NAI_REALM:
			hapd->anqp_type_mask |= ANQP_REQ_NAI_REALM;
			hapd->nai_realm_list_delay = anqp_delay;
			break;
		case ANQP_3GPP_CELLULAR_NETWORK:
			hapd->anqp_type_mask |=
				ANQP_REQ_3GPP_CELLULAR_NETWORK;
			hapd->cellular_network_delay = anqp_delay;
			break;
		case ANQP_DOMAIN_NAME:
			hapd->anqp_type_mask |= ANQP_REQ_DOMAIN_NAME;
			hapd->domain_name_list_delay = anqp_delay;
			break;
		case 0x10000 | HS20_STYPE_OPERATOR_FRIENDLY_NAME:
			hapd->anqp_type_mask |=
				ANQP_REQ_OPERATOR_FRIENDLY_NAME;
			hapd->operator_friendly_name_delay = anqp_delay;
			break;
		case 0x10000 | HS20_STYPE_WAN_METRICS:
			hapd->anqp_type_mask |= ANQP_REQ_WAN_METRICS;
			hapd->wan_metrics_delay = anqp_delay;
			break;
		case 0x10000 | HS20_STYPE_CONNECTION_CAPABILITY:
			hapd->anqp_type_mask |= ANQP_REQ_CONNECTION_CAPABILITY;
			hapd->connection_capability_delay = anqp_delay;
			break;
		case 0x10000 | HS20_STYPE_NAI_HOME_REALM_QUERY:
			hapd->anqp_type_mask |= ANQP_REQ_NAI_HOME_REALM;
			hapd->nai_home_realm_delay = anqp_delay;
			break;
		case 0x10000 | HS20_STYPE_OPERATING_CLASS:
			hapd->anqp_type_mask |= ANQP_REQ_OPERATING_CLASS;
			hapd->nai_home_realm_delay = anqp_delay;
			break;
		default:
			wpa_printf(MSG_ERROR, "Unrecognized ANQP-HS20 Type "
				   "0x%x", anqp_type);
			break;
		}
		len -= 12;
		data += 12;
	}
	return 0;
}


static int hostapd_ctrl_iface_send_anqp_req(struct hostapd_data *hapd,
					    const u8 *buf, size_t buf_len)
{
	struct wpa_ctrl_dst *dst, *next;
	int sent = 0;

	dst = hapd->ctrl_dst;
	if (hapd->ctrl_sock < 0 || dst == NULL)
		return -1;

	while (dst) {
		next = dst->next;
		if (dst->anqp) {
			wpa_hexdump(MSG_DEBUG, "CTRL_IFACE_ANQP send",
				    (u8 *) dst->addr.sun_path, dst->addrlen -
				    offsetof(struct sockaddr_un, sun_path));
			if (sendto(hapd->ctrl_sock, buf, buf_len, 0,
				   (struct sockaddr *) &dst->addr,
				   dst->addrlen) < 0) {
				int _errno = errno;
				wpa_printf(MSG_INFO, "CTRL_IFACE_ANQP: "
					   "%d - %s", errno, strerror(errno));
				dst->errors++;
				if (dst->errors > 10 || _errno == ENOENT) {
					hostapd_ctrl_iface_anqp_detach(
						hapd, &dst->addr,
						dst->addrlen);
				}
			} else {
				dst->errors = 0;
				sent++;
			}
		}
		dst = next;
	}

	return sent ? 0 : -1;
}


static int hostapd_ctrl_iface_rx_venue_name(struct gas_dialog_info *di,
					    u8 *data, int len)
{
	os_free(di->venue_name);
	di->venue_name = os_malloc(len);
	if (di->venue_name == NULL)
		return -1;
	os_memcpy(di->venue_name, data, len);
	di->venue_name_len = len;
	di->received |= ANQP_REQ_VENUE_NAME;
	return 0;
}


static int hostapd_ctrl_iface_rx_net_auth_type(struct gas_dialog_info *di,
					       u8 *data, int len)
{
	os_free(di->net_auth_type);
	di->net_auth_type = os_malloc(len);
	if (di->net_auth_type == NULL)
		return -1;
	os_memcpy(di->net_auth_type, data, len);
	di->net_auth_type_len = len;
	di->received |= ANQP_REQ_NETWORK_AUTH_TYPE;
	return 0;
}


static int hostapd_ctrl_iface_rx_roaming_list(struct gas_dialog_info *di,
					      u8 *data, int len)
{
	os_free(di->roaming_consortium);
	di->roaming_consortium = os_malloc(len);
	if (di->roaming_consortium == NULL)
		return -1;
	os_memcpy(di->roaming_consortium, data, len);
	di->roaming_consortium_len = len;
	di->received |= ANQP_REQ_ROAMING_CONSORTIUM;
	return 0;
}


static int hostapd_ctrl_iface_rx_ipaddr_type(struct gas_dialog_info *di,
					     u8 *data, int len)
{
	os_free(di->ipaddr_type);
	di->ipaddr_type = os_malloc(len);
	if (di->ipaddr_type == NULL)
		return -1;
	os_memcpy(di->ipaddr_type, data, len);
	di->ipaddr_type_len = len;
	di->received |= ANQP_REQ_IP_ADDR_TYPE_AVAILABILITY;
	return 0;
}


static int hostapd_ctrl_iface_rx_nai_realm_list(struct gas_dialog_info *di,
						u8 *data, int len)
{
	os_free(di->nai_realm);
	di->nai_realm = os_malloc(len);
	if (di->nai_realm == NULL)
		return -1;
	os_memcpy(di->nai_realm, data, len);
	di->nai_realm_len = len;
	di->received |= ANQP_REQ_NAI_REALM;
	return 0;
}


static int hostapd_ctrl_iface_rx_3gpp_cell_net(struct gas_dialog_info *di,
					       u8 *data, int len)
{
	os_free(di->cell_net);
	di->cell_net = os_malloc(len);
	if (di->cell_net == NULL)
		return -1;
	os_memcpy(di->cell_net, data, len);
	di->cell_net_len = len;
	di->received |= ANQP_REQ_3GPP_CELLULAR_NETWORK;
	return 0;
}


static int hostapd_ctrl_iface_rx_domain_name_list(struct gas_dialog_info *di,
						  u8 *data, int len)
{
	os_free(di->domain_name);
	di->domain_name = os_malloc(len);
	if (di->domain_name == NULL)
		return -1;
	os_memcpy(di->domain_name, data, len);
	di->domain_name_len = len;
	di->received |= ANQP_REQ_DOMAIN_NAME;
	return 0;
}


static int hostapd_ctrl_iface_rx_oper_friendly_name(
	struct gas_dialog_info *di, u8 *data, int len)
{
	os_free(di->oper_friendly_name);
	di->oper_friendly_name = os_malloc(len);
	if (di->oper_friendly_name == NULL)
		return -1;
	os_memcpy(di->oper_friendly_name, data, len);
	di->oper_friendly_name_len = len;
	di->received |= ANQP_REQ_OPERATOR_FRIENDLY_NAME;
	return 0;
}


static int hostapd_ctrl_iface_rx_wan_metrics(struct gas_dialog_info *di,
					     u8 *data, int len)
{
	os_free(di->wan_metrics);
	di->wan_metrics = os_malloc(len);
	if (di->wan_metrics == NULL)
		return -1;
	os_memcpy(di->wan_metrics, data, len);
	di->wan_metrics_len = len;
	di->received |= ANQP_REQ_WAN_METRICS;
	return 0;
}


static int hostapd_ctrl_iface_rx_conn_capability(struct gas_dialog_info *di,
						 u8 *data, int len)
{
	os_free(di->conn_capability);
	di->conn_capability = os_malloc(len);
	if (di->conn_capability == NULL)
		return -1;
	os_memcpy(di->conn_capability, data, len);
	di->conn_capability_len = len;
	di->received |= ANQP_REQ_CONNECTION_CAPABILITY;
	return 0;
}


static int hostapd_ctrl_iface_rx_nai_home_realm_list(
	struct gas_dialog_info *di, u8 *data, int len)
{
	os_free(di->nai_home_realm);
	di->nai_home_realm = os_malloc(len);
	if (di->nai_home_realm == NULL)
		return -1;
	os_memcpy(di->nai_home_realm, data, len);
	di->nai_home_realm_len = len;
	di->received |= ANQP_REQ_NAI_HOME_REALM;
	return 0;
}

static int hostapd_ctrl_iface_rx_operating_class(struct gas_dialog_info *di,
						 u8 *data, int len)
{
	os_free(di->operating_class);
	di->operating_class = os_malloc(len);
	if (di->operating_class == NULL)
		return -1;
	os_memcpy(di->operating_class, data, len);
	di->operating_class_len = len;
	di->received |= ANQP_REQ_OPERATING_CLASS;
	return 0;
}


static int hostapd_ctrl_iface_anqp_resp(struct hostapd_data *hapd, char *data,
					int len)
{
	struct anqp_hdr *hdr;
	struct gas_dialog_info *dialog;
	u32 anqp_type, anqp_len;

	if (len < 8)
		return -1;

	hdr = (struct anqp_hdr *) data;
	anqp_type = be_to_host32(hdr->anqp_type);
	anqp_len = be_to_host32(hdr->anqp_len);

	dialog = gas_serv_dialog_find(hapd, hdr->sta_addr, hdr->dialog_token);
	if (!dialog)
		return -1;

	switch (anqp_type) {
	case ANQP_VENUE_NAME:
		hostapd_ctrl_iface_rx_venue_name(dialog, hdr->data, anqp_len);
		break;
	case ANQP_NETWORK_AUTH_TYPE:
		hostapd_ctrl_iface_rx_net_auth_type(dialog, hdr->data,
						     anqp_len);
		break;
	case ANQP_ROAMING_CONSORTIUM:
		hostapd_ctrl_iface_rx_roaming_list(dialog, hdr->data,
						    anqp_len);
		break;
	case ANQP_IP_ADDR_TYPE_AVAILABILITY:
		hostapd_ctrl_iface_rx_ipaddr_type(dialog, hdr->data,
						  anqp_len);
		break;
	case ANQP_NAI_REALM:
		hostapd_ctrl_iface_rx_nai_realm_list(dialog, hdr->data,
						     anqp_len);
		break;
	case ANQP_3GPP_CELLULAR_NETWORK:
		hostapd_ctrl_iface_rx_3gpp_cell_net(dialog, hdr->data,
						    anqp_len);
		break;
	case ANQP_DOMAIN_NAME:
		hostapd_ctrl_iface_rx_domain_name_list(dialog, hdr->data,
						       anqp_len);
		break;
	case 0x10000 | HS20_STYPE_OPERATOR_FRIENDLY_NAME:
		hostapd_ctrl_iface_rx_oper_friendly_name(dialog, hdr->data,
							 anqp_len);
		break;
	case 0x10000 | HS20_STYPE_WAN_METRICS:
		hostapd_ctrl_iface_rx_wan_metrics(dialog, hdr->data, anqp_len);
		break;
	case 0x10000 | HS20_STYPE_CONNECTION_CAPABILITY:
		hostapd_ctrl_iface_rx_conn_capability(dialog, hdr->data,
						      anqp_len);
		break;
	case 0x10000 | HS20_STYPE_NAI_HOME_REALM_QUERY:
		hostapd_ctrl_iface_rx_nai_home_realm_list(dialog, hdr->data,
							  anqp_len);
		break;
	case 0x10000 | HS20_STYPE_OPERATING_CLASS:
		hostapd_ctrl_iface_rx_operating_class(dialog, hdr->data,
		                                      anqp_len);
		break;
	default:
		wpa_printf(MSG_ERROR, "Unrecognized ANQP-HS20 Type 0x%x",
			   anqp_type);
		break;
	}
	hostapd_ctrl_iface_hs20_process_ie(hapd, hdr->sta_addr, dialog);
	return 0;
}


static int hostapd_ctrl_iface_set_venue_name(struct hostapd_data *hapd,
					     u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 4)
		return -1;
	data += 4;
	len -= 4;

	os_free(bss->hs20_venue_name);
	bss->hs20_venue_name = os_malloc(len);
	if (bss->hs20_venue_name == NULL)
		return -1;
	os_memcpy(bss->hs20_venue_name, data, len);
	bss->hs20_venue_name_len = len;
	return 0;
}


static int hostapd_ctrl_iface_set_net_auth_type(struct hostapd_data *hapd,
						u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 4)
		return -1;
	data += 4;
	len -= 4;

	os_free(bss->hs20_network_auth_type);
	bss->hs20_network_auth_type = os_malloc(len);
	if (bss->hs20_network_auth_type == NULL)
		return -1;
	os_memcpy(bss->hs20_network_auth_type, data, len);
	bss->hs20_network_auth_type_len = len;
	return 0;
}


static int hostapd_ctrl_iface_set_roaming_list(struct hostapd_data *hapd,
					       u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;
	unsigned int count = 0;
	struct hostapd_roaming_consortium *rc = NULL, *nrc;
	u8 *pos;
	u8 oi_len;

	pos = data;
	if (pos + 4 <= data + len &&
	    WPA_GET_LE16(pos) == ANQP_ROAMING_CONSORTIUM &&
	    WPA_GET_LE16(pos + 2) == len - 4)
		pos += 4; /* Skip ANQP element header */

	while (pos < data + len) {
		oi_len = *pos++;
		if (pos + oi_len > data + len) {
			wpa_printf(MSG_DEBUG, "Invalid Roaming Consortium "
				   "list element");
			os_free(rc);
			return -1;
		}
		wpa_hexdump(MSG_DEBUG, "Add roaming consortium", pos, oi_len);
		nrc = os_realloc(rc,
				 sizeof(struct hostapd_roaming_consortium) *
				 (count + 1));
		if (nrc == NULL) {
			os_free(rc);
			return -1;
		}

		os_memcpy(nrc[count].oi, pos, oi_len);
		pos += oi_len;
		nrc[count].len = oi_len;

		rc = nrc;
		count++;
	}

	os_free(bss->roaming_consortium);
	bss->roaming_consortium = rc;
	bss->roaming_consortium_count = count;

	/*
	 * Update Roaming Consortium element in Beacon and Probe Response
	 * frames.
	 */
	ieee802_11_set_beacon(hapd);
	hostapd_set_ap_wps_ie(hapd);

	return 0;
}


static int hostapd_ctrl_iface_set_ipaddr_type(struct hostapd_data *hapd,
					      u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 4)
		return -1;
	data += 4;
	len -= 4;

	bss->hs20_ipaddr_type_availability = *data;
	bss->hs20_ipaddr_type_configured = 1;
	return 0;
}


static int hostapd_ctrl_iface_set_nai_realm_list(struct hostapd_data *hapd,
						 u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 4)
		return -1;
	data += 4;
	len -= 4;

	os_free(bss->hs20_nai_realm_list);
	bss->hs20_nai_realm_list = os_malloc(len);
	if (bss->hs20_nai_realm_list == NULL)
		return -1;
	os_memcpy(bss->hs20_nai_realm_list, data, len);
	bss->hs20_nai_realm_list_len = len;
	return 0;
}


static int hostapd_ctrl_iface_set_3gpp_cell_net(struct hostapd_data *hapd,
						u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 4)
		return -1;
	data += 4;
	len -= 4;

	os_free(bss->hs20_3gpp_cellular_network);
	bss->hs20_3gpp_cellular_network = os_malloc(len);
	if (bss->hs20_3gpp_cellular_network == NULL)
		return -1;
	os_memcpy(bss->hs20_3gpp_cellular_network, data, len);
	bss->hs20_3gpp_cellular_network_len = len;
	return 0;
}


static int hostapd_ctrl_iface_set_domain_name_list(struct hostapd_data *hapd,
						   u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 4)
		return -1;
	data += 4;
	len -= 4;

	os_free(bss->hs20_domain_name_list_value);
	bss->hs20_domain_name_list_value = os_malloc(len);
	if (bss->hs20_domain_name_list_value == NULL)
		return -1;
	os_memcpy(bss->hs20_domain_name_list_value, data, len);
	bss->hs20_domain_name_list_len = len;
	return 0;
}


static int hostapd_ctrl_iface_set_oper_friendly_name(struct hostapd_data *hapd,
						     u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 10)
		return -1;
	data += 10;
	len -= 10;

	os_free(bss->hs20_operator_friendly_name);
	bss->hs20_operator_friendly_name = os_malloc(len);
	if (bss->hs20_operator_friendly_name == NULL)
		return -1;
	os_memcpy(bss->hs20_operator_friendly_name, data, len);
	bss->hs20_operator_friendly_name_len = len;
	return 0;
}


static int hostapd_ctrl_iface_set_wan_metrics(struct hostapd_data *hapd,
					      u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 10)
		return -1;
	data += 10;
	len -= 10;

	os_free(bss->hs20_wan_metrics);
	bss->hs20_wan_metrics = os_malloc(len);
	if (bss->hs20_wan_metrics == NULL)
		return -1;
	os_memcpy(bss->hs20_wan_metrics, data, len);
	return 0;
}


static int hostapd_ctrl_iface_set_conn_capability(struct hostapd_data *hapd,
						  u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 10)
		return -1;
	data += 10;
	len -= 10;

	os_free(bss->hs20_connection_capability);
	bss->hs20_connection_capability = os_malloc(len);
	if (bss->hs20_connection_capability == NULL)
		return -1;
	os_memcpy(bss->hs20_connection_capability, data, len);
	bss->hs20_connection_capability_len = len;
	return 0;
}


static int hostapd_ctrl_iface_set_operating_class(struct hostapd_data *hapd,
						  u8 *data, int len)
{
	struct hostapd_bss_config *bss = hapd->conf;

	/* entire element is sent, strip hdr */
	if (len < 10)
		return -1;
	data += 10;
	len -= 10;

	os_free(bss->hs20_operating_class);
	bss->hs20_operating_class = os_malloc(len);
	if (bss->hs20_operating_class == NULL)
		return -1;
	os_memcpy(bss->hs20_operating_class, data, len);
	bss->hs20_operating_class_len = len;
	return 0;
}


/* format of ANQP_SET message is the ASCII string:
 * "ANQP_SET <8 byte anqp type in hex> <entire element including
 * hdr in hex>"
 */
static int hostapd_ctrl_iface_anqp_set(struct hostapd_data *hapd,
				       const char *data, int len)
{
	int ret = 0;
	u8 type[4];
	u32 anqp_type;
	u8 *info_elem;

	if (len < 9)
		return -1;

	if (hexstr2bin(data, type, 4) < 0) {
		wpa_printf(MSG_ERROR, "Invalid hex substr %s", data);
		return 1;
	}

	anqp_type = WPA_GET_BE32(type);
	data += 9;
	len -= 9;
	info_elem = os_malloc(len / 2);
	if (!info_elem) {
		wpa_printf(MSG_ERROR, "memory allocation failed\n");
		return 2;
	}
	if (hexstr2bin(data, info_elem, len / 2) < 0) {
		wpa_printf(MSG_ERROR, "Invalid hex substr %s", data);
		os_free(info_elem);
		return 3;
	}

	switch (anqp_type) {
	case ANQP_VENUE_NAME:
		ret = hostapd_ctrl_iface_set_venue_name(hapd, info_elem,
							len / 2);
		break;
	case ANQP_NETWORK_AUTH_TYPE:
		ret = hostapd_ctrl_iface_set_net_auth_type(hapd, info_elem,
							   len / 2);
		break;
	case ANQP_ROAMING_CONSORTIUM:
		ret = hostapd_ctrl_iface_set_roaming_list(hapd, info_elem,
							  len / 2);
		break;
	case ANQP_IP_ADDR_TYPE_AVAILABILITY:
		ret = hostapd_ctrl_iface_set_ipaddr_type(hapd, info_elem,
							 len / 2);
		break;
	case ANQP_NAI_REALM:
		ret = hostapd_ctrl_iface_set_nai_realm_list(hapd, info_elem,
							    len / 2);
		break;
	case ANQP_3GPP_CELLULAR_NETWORK:
		ret = hostapd_ctrl_iface_set_3gpp_cell_net(hapd, info_elem,
							   len / 2);
		break;
	case ANQP_DOMAIN_NAME:
		ret = hostapd_ctrl_iface_set_domain_name_list(hapd, info_elem,
							      len / 2);
		break;
	case 0x10000 | HS20_STYPE_OPERATOR_FRIENDLY_NAME:
		ret = hostapd_ctrl_iface_set_oper_friendly_name(
			hapd, info_elem, len / 2);
		break;
	case 0x10000 | HS20_STYPE_WAN_METRICS:
		ret = hostapd_ctrl_iface_set_wan_metrics(hapd, info_elem,
							 len / 2);
		break;
	case 0x10000 | HS20_STYPE_CONNECTION_CAPABILITY:
		ret = hostapd_ctrl_iface_set_conn_capability(hapd, info_elem,
							     len / 2);
		break;
	case 0x10000 | HS20_STYPE_OPERATING_CLASS:
		ret = hostapd_ctrl_iface_set_operating_class(hapd, info_elem,
							     len / 2);
		break;
	default:
		wpa_printf(MSG_ERROR, "Unrecognized ANQP Type 0x%x",
			   anqp_type);
		break;
	}
	os_free(info_elem);
	return ret;
}

#endif /* CONFIG_HS20 */


static int hostapd_ctrl_iface_ess_disassoc(struct hostapd_data *hapd,
					   const char *cmd)
{
	u8 addr[ETH_ALEN];
	const char *url;
	u8 buf[1000], *pos;
	struct ieee80211_mgmt *mgmt;
	size_t url_len;

	if (hwaddr_aton(cmd, addr))
		return -1;
	url = cmd + 17;
	if (*url != ' ')
		return -1;
	url++;
	url_len = os_strlen(url);
	if (url_len > 255)
		return -1;

	os_memset(buf, 0, sizeof(buf));
	mgmt = (struct ieee80211_mgmt *) buf;
	mgmt->frame_control = IEEE80211_FC(WLAN_FC_TYPE_MGMT,
					   WLAN_FC_STYPE_ACTION);
	os_memcpy(mgmt->da, addr, ETH_ALEN);
	os_memcpy(mgmt->sa, hapd->own_addr, ETH_ALEN);
	os_memcpy(mgmt->bssid, hapd->own_addr, ETH_ALEN);
	mgmt->u.action.category = WLAN_ACTION_WNM;
	mgmt->u.action.u.bss_tm_req.action = WNM_BSS_TRANS_MGMT_REQ;
	mgmt->u.action.u.bss_tm_req.dialog_token = 1;
	mgmt->u.action.u.bss_tm_req.req_mode =
		WNM_BSS_TM_REQ_ESS_DISASSOC_IMMINENT;
	mgmt->u.action.u.bss_tm_req.disassoc_timer = host_to_le16(0);
	mgmt->u.action.u.bss_tm_req.validity_interval = 0;

	pos = mgmt->u.action.u.bss_tm_req.variable;

	/* Session Information URL */
	*pos++ = url_len;
	os_memcpy(pos, url, url_len);
	pos += url_len;

	if (hostapd_drv_send_mlme(hapd, buf, pos - buf, 0) < 0) {
		wpa_printf(MSG_DEBUG, "Failed to send BSS Transition "
			   "Management Request frame");
		return -1;
	}

	return 0;
}


static int hostapd_ctrl_iface_get_config(struct hostapd_data *hapd,
					 char *buf, size_t buflen)
{
	int ret;
	char *pos, *end;

	pos = buf;
	end = buf + buflen;

	ret = os_snprintf(pos, end - pos, "bssid=" MACSTR "\n"
			  "ssid=%s\n",
			  MAC2STR(hapd->own_addr),
			  hapd->conf->ssid.ssid);
	if (ret < 0 || ret >= end - pos)
		return pos - buf;
	pos += ret;

#ifdef CONFIG_WPS
	ret = os_snprintf(pos, end - pos, "wps_state=%s\n",
			  hapd->conf->wps_state == 0 ? "disabled" :
			  (hapd->conf->wps_state == 1 ? "not configured" :
			   "configured"));
	if (ret < 0 || ret >= end - pos)
		return pos - buf;
	pos += ret;

	if (hapd->conf->wps_state && hapd->conf->wpa &&
	    hapd->conf->ssid.wpa_passphrase) {
		ret = os_snprintf(pos, end - pos, "passphrase=%s\n",
				  hapd->conf->ssid.wpa_passphrase);
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	}

	if (hapd->conf->wps_state && hapd->conf->wpa &&
	    hapd->conf->ssid.wpa_psk &&
	    hapd->conf->ssid.wpa_psk->group) {
		char hex[PMK_LEN * 2 + 1];
		wpa_snprintf_hex(hex, sizeof(hex),
				 hapd->conf->ssid.wpa_psk->psk, PMK_LEN);
		ret = os_snprintf(pos, end - pos, "psk=%s\n", hex);
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	}
#endif /* CONFIG_WPS */

	if (hapd->conf->wpa && hapd->conf->wpa_key_mgmt) {
		ret = os_snprintf(pos, end - pos, "key_mgmt=");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;

		if (hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_PSK) {
			ret = os_snprintf(pos, end - pos, "WPA-PSK ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
		if (hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_IEEE8021X) {
			ret = os_snprintf(pos, end - pos, "WPA-EAP ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
#ifdef CONFIG_IEEE80211R
		if (hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_FT_PSK) {
			ret = os_snprintf(pos, end - pos, "FT-PSK ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
		if (hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_FT_IEEE8021X) {
			ret = os_snprintf(pos, end - pos, "FT-EAP ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
#endif /* CONFIG_IEEE80211R */
#ifdef CONFIG_IEEE80211W
		if (hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_PSK_SHA256) {
			ret = os_snprintf(pos, end - pos, "WPA-PSK-SHA256 ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
		if (hapd->conf->wpa_key_mgmt & WPA_KEY_MGMT_IEEE8021X_SHA256) {
			ret = os_snprintf(pos, end - pos, "WPA-EAP-SHA256 ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
#endif /* CONFIG_IEEE80211W */

		ret = os_snprintf(pos, end - pos, "\n");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	}

	if (hapd->conf->wpa && hapd->conf->wpa_group == WPA_CIPHER_CCMP) {
		ret = os_snprintf(pos, end - pos, "group_cipher=CCMP\n");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	} else if (hapd->conf->wpa &&
		   hapd->conf->wpa_group == WPA_CIPHER_TKIP) {
		ret = os_snprintf(pos, end - pos, "group_cipher=TKIP\n");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	}

	if ((hapd->conf->wpa & WPA_PROTO_RSN) && hapd->conf->rsn_pairwise) {
		ret = os_snprintf(pos, end - pos, "rsn_pairwise_cipher=");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;

		if (hapd->conf->rsn_pairwise & WPA_CIPHER_CCMP) {
			ret = os_snprintf(pos, end - pos, "CCMP ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
		if (hapd->conf->rsn_pairwise & WPA_CIPHER_TKIP) {
			ret = os_snprintf(pos, end - pos, "TKIP ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}

		ret = os_snprintf(pos, end - pos, "\n");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	}

	if ((hapd->conf->wpa & WPA_PROTO_WPA) && hapd->conf->wpa_pairwise) {
		ret = os_snprintf(pos, end - pos, "wpa_pairwise_cipher=");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;

		if (hapd->conf->wpa_pairwise & WPA_CIPHER_CCMP) {
			ret = os_snprintf(pos, end - pos, "CCMP ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}
		if (hapd->conf->wpa_pairwise & WPA_CIPHER_TKIP) {
			ret = os_snprintf(pos, end - pos, "TKIP ");
			if (ret < 0 || ret >= end - pos)
				return pos - buf;
			pos += ret;
		}

		ret = os_snprintf(pos, end - pos, "\n");
		if (ret < 0 || ret >= end - pos)
			return pos - buf;
		pos += ret;
	}

	return pos - buf;
}


static int hostapd_ctrl_iface_set(struct hostapd_data *hapd, char *cmd)
{
	char *value;
	int ret = 0;

	value = os_strchr(cmd, ' ');
	if (value == NULL)
		return -1;
	*value++ = '\0';

	wpa_printf(MSG_DEBUG, "CTRL_IFACE SET '%s'='%s'", cmd, value);
	if (0) {
#ifdef CONFIG_WPS_TESTING
	} else if (os_strcasecmp(cmd, "wps_version_number") == 0) {
		long int val;
		val = strtol(value, NULL, 0);
		if (val < 0 || val > 0xff) {
			ret = -1;
			wpa_printf(MSG_DEBUG, "WPS: Invalid "
				   "wps_version_number %ld", val);
		} else {
			wps_version_number = val;
			wpa_printf(MSG_DEBUG, "WPS: Testing - force WPS "
				   "version %u.%u",
				   (wps_version_number & 0xf0) >> 4,
				   wps_version_number & 0x0f);
			hostapd_wps_update_ie(hapd);
		}
	} else if (os_strcasecmp(cmd, "wps_testing_dummy_cred") == 0) {
		wps_testing_dummy_cred = atoi(value);
		wpa_printf(MSG_DEBUG, "WPS: Testing - dummy_cred=%d",
			   wps_testing_dummy_cred);
#endif /* CONFIG_WPS_TESTING */
#ifdef CONFIG_INTERWORKING
	} else if (os_strcasecmp(cmd, "gas_frag_limit") == 0) {
		int val = atoi(value);
		if (val <= 0)
			ret = -1;
		else
			hapd->gas_frag_limit = val;
#endif /* CONFIG_INTERWORKING */
	} else {
		ret = hostapd_set_iface(hapd->iconf, hapd->conf, cmd, value);
	}

	return ret;
}


static int hostapd_ctrl_iface_get(struct hostapd_data *hapd, char *cmd,
				  char *buf, size_t buflen)
{
	int res;

	wpa_printf(MSG_DEBUG, "CTRL_IFACE GET '%s'", cmd);

	if (os_strcmp(cmd, "version") == 0) {
		res = os_snprintf(buf, buflen, "%s", VERSION_STR);
		if (res < 0 || (unsigned int) res >= buflen)
			return -1;
		return res;
	}

	return -1;
}


static int hostapd_ctrl_iface_enable(struct hostapd_iface *iface)
{
	if (hostapd_enable_iface(iface) < 0) {
		wpa_printf(MSG_ERROR, "Enabling of interface failed");
		return -1;
	}
	return 0;
}


static int hostapd_ctrl_iface_reload(struct hostapd_iface *iface)
{
	if (hostapd_reload_iface(iface) < 0) {
		wpa_printf(MSG_ERROR, "Reloading of interface failed");
		return -1;
	}
	return 0;
}


static int hostapd_ctrl_iface_disable(struct hostapd_iface *iface)
{
	if (hostapd_disable_iface(iface) < 0) {
		wpa_printf(MSG_ERROR, "Disabling of interface failed");
		return -1;
	}
	return 0;
}


static void hostapd_ctrl_iface_receive(int sock, void *eloop_ctx,
				       void *sock_ctx)
{
	struct hostapd_data *hapd = eloop_ctx;
	char buf[512];
	int res;
	struct sockaddr_un from;
	socklen_t fromlen = sizeof(from);
	char *reply;
	const int reply_size = 4096;
	int reply_len;
	int level = MSG_DEBUG;

	res = recvfrom(sock, buf, sizeof(buf) - 1, 0,
		       (struct sockaddr *) &from, &fromlen);
	if (res < 0) {
		perror("recvfrom(ctrl_iface)");
		return;
	}
	buf[res] = '\0';
	if (os_strcmp(buf, "PING") == 0)
		level = MSG_EXCESSIVE;
	wpa_hexdump_ascii(level, "RX ctrl_iface", (u8 *) buf, res);

	reply = os_malloc(reply_size);
	if (reply == NULL) {
		sendto(sock, "FAIL\n", 5, 0, (struct sockaddr *) &from,
		       fromlen);
		return;
	}

	os_memcpy(reply, "OK\n", 3);
	reply_len = 3;

	if (os_strcmp(buf, "PING") == 0) {
		os_memcpy(reply, "PONG\n", 5);
		reply_len = 5;
	} else if (os_strncmp(buf, "RELOG", 5) == 0) {
		if (wpa_debug_reopen_file() < 0)
			reply_len = -1;
	} else if (os_strcmp(buf, "MIB") == 0) {
		reply_len = ieee802_11_get_mib(hapd, reply, reply_size);
		if (reply_len >= 0) {
			res = wpa_get_mib(hapd->wpa_auth, reply + reply_len,
					  reply_size - reply_len);
			if (res < 0)
				reply_len = -1;
			else
				reply_len += res;
		}
		if (reply_len >= 0) {
			res = ieee802_1x_get_mib(hapd, reply + reply_len,
						 reply_size - reply_len);
			if (res < 0)
				reply_len = -1;
			else
				reply_len += res;
		}
#ifndef CONFIG_NO_RADIUS
		if (reply_len >= 0) {
			res = radius_client_get_mib(hapd->radius,
						    reply + reply_len,
						    reply_size - reply_len);
			if (res < 0)
				reply_len = -1;
			else
				reply_len += res;
		}
#endif /* CONFIG_NO_RADIUS */
	} else if (os_strcmp(buf, "STA-FIRST") == 0) {
		reply_len = hostapd_ctrl_iface_sta_first(hapd, reply,
							 reply_size);
	} else if (os_strncmp(buf, "STA ", 4) == 0) {
		reply_len = hostapd_ctrl_iface_sta(hapd, buf + 4, reply,
						   reply_size);
	} else if (os_strncmp(buf, "STA-NEXT ", 9) == 0) {
		reply_len = hostapd_ctrl_iface_sta_next(hapd, buf + 9, reply,
							reply_size);
	} else if (os_strcmp(buf, "ATTACH") == 0) {
		if (hostapd_ctrl_iface_attach(hapd, &from, fromlen, 0))
			reply_len = -1;
	} else if (os_strcmp(buf, "DETACH") == 0) {
		if (hostapd_ctrl_iface_detach(hapd, &from, fromlen))
			reply_len = -1;
	} else if (os_strncmp(buf, "LEVEL ", 6) == 0) {
		if (hostapd_ctrl_iface_level(hapd, &from, fromlen,
						    buf + 6))
			reply_len = -1;
	} else if (os_strncmp(buf, "NEW_STA ", 8) == 0) {
		if (hostapd_ctrl_iface_new_sta(hapd, buf + 8))
			reply_len = -1;
	} else if (os_strncmp(buf, "DEAUTHENTICATE ", 15) == 0) {
		if (hostapd_ctrl_iface_deauthenticate(hapd, buf + 15))
			reply_len = -1;
	} else if (os_strncmp(buf, "DISASSOCIATE ", 13) == 0) {
		if (hostapd_ctrl_iface_disassociate(hapd, buf + 13))
			reply_len = -1;
#ifdef CONFIG_IEEE80211W
#ifdef NEED_AP_MLME
	} else if (os_strncmp(buf, "SA_QUERY ", 9) == 0) {
		if (hostapd_ctrl_iface_sa_query(hapd, buf + 9))
			reply_len = -1;
#endif /* NEED_AP_MLME */
#endif /* CONFIG_IEEE80211W */
#ifdef CONFIG_WPS
	} else if (os_strncmp(buf, "WPS_PIN ", 8) == 0) {
		if (hostapd_ctrl_iface_wps_pin(hapd, buf + 8))
			reply_len = -1;
	} else if (os_strncmp(buf, "WPS_CHECK_PIN ", 14) == 0) {
		reply_len = hostapd_ctrl_iface_wps_check_pin(
			hapd, buf + 14, reply, reply_size);
	} else if (os_strcmp(buf, "WPS_PBC") == 0) {
		if (hostapd_wps_button_pushed(hapd, NULL))
			reply_len = -1;
	} else if (os_strcmp(buf, "WPS_CANCEL") == 0) {
		if (hostapd_wps_cancel(hapd))
			reply_len = -1;
#ifdef CONFIG_WPS_OOB
	} else if (os_strncmp(buf, "WPS_OOB ", 8) == 0) {
		if (hostapd_ctrl_iface_wps_oob(hapd, buf + 8))
			reply_len = -1;
#endif /* CONFIG_WPS_OOB */
	} else if (os_strncmp(buf, "WPS_AP_PIN ", 11) == 0) {
		reply_len = hostapd_ctrl_iface_wps_ap_pin(hapd, buf + 11,
							  reply, reply_size);
	} else if (os_strncmp(buf, "WPS_CONFIG ", 11) == 0) {
		if (hostapd_ctrl_iface_wps_config(hapd, buf + 11) < 0)
			reply_len = -1;
	} else if (os_strncmp(buf, "WPS_GET_CONFIG", 14) == 0) {
		reply_len = hostapd_ctrl_iface_wps_get_config(hapd, reply,
							      reply_size);
#endif /* CONFIG_WPS */
#ifdef CONFIG_HS20
	} else if (os_strncmp(buf, "ANQP_ATTACH ", 12) == 0) {
		hostapd_ctrl_iface_anqp_attach(hapd, buf + 12, res - 12, &from,
					       fromlen);
		goto no_reply;
	} else if (os_strncmp(buf, "ANQP_DETACH", 11) == 0) {
		hostapd_ctrl_iface_anqp_detach(hapd, &from, fromlen);
		goto no_reply;
	} else if (os_strncmp(buf, "ANQP_SET ", 9) == 0) {
		hostapd_ctrl_iface_anqp_set(hapd, buf + 9, res - 9);
		/* ANQP_SET is called by hostapd_cli or anqpserver. The latter
		 * doesn't expect a response
		 */
		if (hostapd_ctrl_iface_is_anqpsock(hapd, &from, fromlen))
		goto no_reply;
#endif /* CONFIG_HS20 */
	} else if (os_strncmp(buf, "ESS_DISASSOC ", 13) == 0) {
		if (hostapd_ctrl_iface_ess_disassoc(hapd, buf + 13))
			reply_len = -1;
	} else if (os_strcmp(buf, "GET_CONFIG") == 0) {
		reply_len = hostapd_ctrl_iface_get_config(hapd, reply,
							  reply_size);
	} else if (os_strncmp(buf, "SET ", 4) == 0) {
		if (hostapd_ctrl_iface_set(hapd, buf + 4))
			reply_len = -1;
	} else if (os_strncmp(buf, "GET ", 4) == 0) {
		reply_len = hostapd_ctrl_iface_get(hapd, buf + 4, reply,
						   reply_size);
	} else if (os_strncmp(buf, "ENABLE", 6) == 0) {
		if (hostapd_ctrl_iface_enable(hapd->iface))
			reply_len = -1;
	} else if (os_strncmp(buf, "RELOAD", 6) == 0) {
		if (hostapd_ctrl_iface_reload(hapd->iface))
			reply_len = -1;
	} else if (os_strncmp(buf, "DISABLE", 7) == 0) {
		if (hostapd_ctrl_iface_disable(hapd->iface))
			reply_len = -1;
	} else {
#ifdef CONFIG_HS20
		if (hostapd_ctrl_iface_is_anqpsock(hapd, &from, fromlen)) {
			hostapd_ctrl_iface_anqp_resp(hapd, buf, res);
			goto no_reply;
		}
#endif /* CONFIG_HS20 */
		os_memcpy(reply, "UNKNOWN COMMAND\n", 16);
		reply_len = 16;
	}

	if (reply_len < 0) {
		os_memcpy(reply, "FAIL\n", 5);
		reply_len = 5;
	}
	sendto(sock, reply, reply_len, 0, (struct sockaddr *) &from, fromlen);
#ifdef CONFIG_HS20
no_reply:
#endif /* CONFIG_HS20 */
	os_free(reply);
}


static char * hostapd_ctrl_iface_path(struct hostapd_data *hapd)
{
	char *buf;
	size_t len;

	if (hapd->conf->ctrl_interface == NULL)
		return NULL;

	len = os_strlen(hapd->conf->ctrl_interface) +
		os_strlen(hapd->conf->iface) + 2;
	buf = os_malloc(len);
	if (buf == NULL)
		return NULL;

	os_snprintf(buf, len, "%s/%s",
		    hapd->conf->ctrl_interface, hapd->conf->iface);
	buf[len - 1] = '\0';
	return buf;
}


static void hostapd_ctrl_iface_msg_cb(void *ctx, int level,
				      const char *txt, size_t len)
{
	struct hostapd_data *hapd = ctx;
	if (hapd == NULL)
		return;
	hostapd_ctrl_iface_send(hapd, level, txt, len);
}


int hostapd_ctrl_iface_init(struct hostapd_data *hapd)
{
	struct sockaddr_un addr;
	int s = -1;
	char *fname = NULL;

	if (hapd->ctrl_sock > -1) {
		wpa_printf(MSG_DEBUG, "ctrl_iface already exists!");
		return 0;
	}

	if (hapd->conf->ctrl_interface == NULL)
		return 0;

	if (mkdir(hapd->conf->ctrl_interface, S_IRWXU | S_IRWXG) < 0) {
		if (errno == EEXIST) {
			wpa_printf(MSG_DEBUG, "Using existing control "
				   "interface directory.");
		} else {
			perror("mkdir[ctrl_interface]");
			goto fail;
		}
	}

	if (hapd->conf->ctrl_interface_gid_set &&
	    chown(hapd->conf->ctrl_interface, 0,
		  hapd->conf->ctrl_interface_gid) < 0) {
		perror("chown[ctrl_interface]");
		return -1;
	}

	if (os_strlen(hapd->conf->ctrl_interface) + 1 +
	    os_strlen(hapd->conf->iface) >= sizeof(addr.sun_path))
		goto fail;

	s = socket(PF_UNIX, SOCK_DGRAM, 0);
	if (s < 0) {
		perror("socket(PF_UNIX)");
		goto fail;
	}

	os_memset(&addr, 0, sizeof(addr));
#ifdef __FreeBSD__
	addr.sun_len = sizeof(addr);
#endif /* __FreeBSD__ */
	addr.sun_family = AF_UNIX;
	fname = hostapd_ctrl_iface_path(hapd);
	if (fname == NULL)
		goto fail;
	os_strlcpy(addr.sun_path, fname, sizeof(addr.sun_path));
	if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		wpa_printf(MSG_DEBUG, "ctrl_iface bind(PF_UNIX) failed: %s",
			   strerror(errno));
		if (connect(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
			wpa_printf(MSG_DEBUG, "ctrl_iface exists, but does not"
				   " allow connections - assuming it was left"
				   "over from forced program termination");
			if (unlink(fname) < 0) {
				perror("unlink[ctrl_iface]");
				wpa_printf(MSG_ERROR, "Could not unlink "
					   "existing ctrl_iface socket '%s'",
					   fname);
				goto fail;
			}
			if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) <
			    0) {
				perror("hostapd-ctrl-iface: bind(PF_UNIX)");
				goto fail;
			}
			wpa_printf(MSG_DEBUG, "Successfully replaced leftover "
				   "ctrl_iface socket '%s'", fname);
		} else {
			wpa_printf(MSG_INFO, "ctrl_iface exists and seems to "
				   "be in use - cannot override it");
			wpa_printf(MSG_INFO, "Delete '%s' manually if it is "
				   "not used anymore", fname);
			os_free(fname);
			fname = NULL;
			goto fail;
		}
	}

	if (hapd->conf->ctrl_interface_gid_set &&
	    chown(fname, 0, hapd->conf->ctrl_interface_gid) < 0) {
		perror("chown[ctrl_interface/ifname]");
		goto fail;
	}

	if (chmod(fname, S_IRWXU | S_IRWXG) < 0) {
		perror("chmod[ctrl_interface/ifname]");
		goto fail;
	}
	os_free(fname);

	hapd->ctrl_sock = s;
	eloop_register_read_sock(s, hostapd_ctrl_iface_receive, hapd,
				 NULL);
	hapd->msg_ctx = hapd;
	wpa_msg_register_cb(hostapd_ctrl_iface_msg_cb);
#ifdef CONFIG_HS20
	hapd->anqp_remote_req = hostapd_ctrl_iface_send_anqp_req;
#endif /* CONFIG_HS20 */

	return 0;

fail:
	if (s >= 0)
		close(s);
	if (fname) {
		unlink(fname);
		os_free(fname);
	}
	return -1;
}


void hostapd_ctrl_iface_deinit(struct hostapd_data *hapd)
{
	struct wpa_ctrl_dst *dst, *prev;

	if (hapd->ctrl_sock > -1) {
		char *fname;
		eloop_unregister_read_sock(hapd->ctrl_sock);
		close(hapd->ctrl_sock);
		hapd->ctrl_sock = -1;
		fname = hostapd_ctrl_iface_path(hapd);
		if (fname)
			unlink(fname);
		os_free(fname);

		if (hapd->conf->ctrl_interface &&
		    rmdir(hapd->conf->ctrl_interface) < 0) {
			if (errno == ENOTEMPTY) {
				wpa_printf(MSG_DEBUG, "Control interface "
					   "directory not empty - leaving it "
					   "behind");
			} else {
				perror("rmdir[ctrl_interface]");
			}
		}
	}

	dst = hapd->ctrl_dst;
	while (dst) {
		prev = dst;
		dst = dst->next;
		os_free(prev);
	}
}


static int hostapd_ctrl_iface_add(struct hapd_interfaces *interfaces,
				  char *buf)
{
	if (hostapd_add_iface(interfaces, buf) < 0) {
		wpa_printf(MSG_ERROR, "Adding interface %s failed", buf);
		return -1;
	}
	return 0;
}


static int hostapd_ctrl_iface_remove(struct hapd_interfaces *interfaces,
				     char *buf)
{
	if (hostapd_remove_iface(interfaces, buf) < 0) {
		wpa_printf(MSG_ERROR, "Removing interface %s failed", buf);
		return -1;
	}
	return 0;
}


static void hostapd_global_ctrl_iface_receive(int sock, void *eloop_ctx,
					      void *sock_ctx)
{
	void *interfaces = eloop_ctx;
	char buf[256];
	int res;
	struct sockaddr_un from;
	socklen_t fromlen = sizeof(from);
	char reply_buf[24];
	int reply_len;
	int ret = 0;

	res = recvfrom(sock, buf, sizeof(buf) - 1, 0,
		       (struct sockaddr *) &from, &fromlen);
	if (res < 0) {
		perror("recvfrom(ctrl_iface)");
		return;
	}
	buf[res] = '\0';

	if (os_strncmp (buf, "ADD ", 4) == 0) {
		ret = hostapd_ctrl_iface_add(interfaces, buf + 4);
	} else if (os_strncmp (buf, "REMOVE ", 7) == 0) {
		ret = hostapd_ctrl_iface_remove(interfaces, buf + 7);
	}

	if (ret == -1)
		reply_len = os_snprintf(reply_buf, sizeof(reply_buf), "FAIL");
	else
		reply_len = os_snprintf(reply_buf, sizeof(reply_buf), "OK");

	sendto(sock, reply_buf, reply_len, 0, (struct sockaddr *) &from,
	       fromlen);
}


static char * hostapd_global_ctrl_iface_path(struct hapd_interfaces *interface)
{
	char *buf;
	size_t len;

	if (interface->global_iface_path == NULL)
		return NULL;

	len = os_strlen(interface->global_iface_path) +
		os_strlen(interface->global_iface_name) + 2;
	buf = os_malloc(len);
	if (buf == NULL)
		return NULL;

	os_snprintf(buf, len, "%s/%s", interface->global_iface_path,
		    interface->global_iface_name);
	buf[len - 1] = '\0';
	return buf;
}


int hostapd_global_ctrl_iface_init(struct hapd_interfaces *interface)
{
	struct sockaddr_un addr;
	int s = -1;
	char *fname = NULL;

	if (interface->global_iface_path == NULL) {
		wpa_printf(MSG_DEBUG, "ctrl_iface not configured!");
		return 0;
	}

	if (mkdir(interface->global_iface_path, S_IRWXU | S_IRWXG) < 0) {
		if (errno == EEXIST) {
			wpa_printf(MSG_DEBUG, "Using existing control "
				   "interface directory.");
		} else {
			perror("mkdir[ctrl_interface]");
			goto fail;
		}
	}

	if (os_strlen(interface->global_iface_path) + 1 +
	    os_strlen(interface->global_iface_name) >= sizeof(addr.sun_path))
		goto fail;

	s = socket(PF_UNIX, SOCK_DGRAM, 0);
	if (s < 0) {
		perror("socket(PF_UNIX)");
		goto fail;
	}

	os_memset(&addr, 0, sizeof(addr));
#ifdef __FreeBSD__
	addr.sun_len = sizeof(addr);
#endif /* __FreeBSD__ */
	addr.sun_family = AF_UNIX;
	fname = hostapd_global_ctrl_iface_path(interface);
	if (fname == NULL)
		goto fail;
	os_strlcpy(addr.sun_path, fname, sizeof(addr.sun_path));
	if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		wpa_printf(MSG_DEBUG, "ctrl_iface bind(PF_UNIX) failed: %s",
			   strerror(errno));
		if (connect(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
			wpa_printf(MSG_DEBUG, "ctrl_iface exists, but does not"
				   " allow connections - assuming it was left"
				   "over from forced program termination");
			if (unlink(fname) < 0) {
				perror("unlink[ctrl_iface]");
				wpa_printf(MSG_ERROR, "Could not unlink "
					   "existing ctrl_iface socket '%s'",
					   fname);
				goto fail;
			}
			if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) <
			    0) {
				perror("bind(PF_UNIX)");
				goto fail;
			}
			wpa_printf(MSG_DEBUG, "Successfully replaced leftover "
				   "ctrl_iface socket '%s'", fname);
		} else {
			wpa_printf(MSG_INFO, "ctrl_iface exists and seems to "
				   "be in use - cannot override it");
			wpa_printf(MSG_INFO, "Delete '%s' manually if it is "
				   "not used anymore", fname);
			os_free(fname);
			fname = NULL;
			goto fail;
		}
	}

	if (chmod(fname, S_IRWXU | S_IRWXG) < 0) {
		perror("chmod[ctrl_interface/ifname]");
		goto fail;
	}
	os_free(fname);

	interface->global_ctrl_sock = s;
	eloop_register_read_sock(s, hostapd_global_ctrl_iface_receive,
				 interface, NULL);

	return 0;

fail:
	if (s >= 0)
		close(s);
	if (fname) {
		unlink(fname);
		os_free(fname);
	}
	return -1;
}


void hostapd_global_ctrl_iface_deinit(struct hapd_interfaces *interfaces)
{
	char *fname = NULL;

	if (interfaces->global_ctrl_sock > -1) {
		eloop_unregister_read_sock(interfaces->global_ctrl_sock);
		close(interfaces->global_ctrl_sock);
		interfaces->global_ctrl_sock = -1;
		fname = hostapd_global_ctrl_iface_path(interfaces);
		if (fname) {
			unlink(fname);
			os_free(fname);
		}

		if (interfaces->global_iface_path &&
		    rmdir(interfaces->global_iface_path) < 0) {
			if (errno == ENOTEMPTY) {
				wpa_printf(MSG_DEBUG, "Control interface "
					   "directory not empty - leaving it "
					   "behind");
			} else {
				perror("rmdir[ctrl_interface]");
			}
		}
		os_free(interfaces->global_iface_path);
		interfaces->global_iface_path = NULL;
	}
}


static void hostapd_ctrl_iface_send(struct hostapd_data *hapd, int level,
				    const char *buf, size_t len)
{
	struct wpa_ctrl_dst *dst, *next;
	struct msghdr msg;
	int idx;
	struct iovec io[2];
	char levelstr[10];

	dst = hapd->ctrl_dst;
	if (hapd->ctrl_sock < 0 || dst == NULL)
		return;

	os_snprintf(levelstr, sizeof(levelstr), "<%d>", level);
	io[0].iov_base = levelstr;
	io[0].iov_len = os_strlen(levelstr);
	io[1].iov_base = (char *) buf;
	io[1].iov_len = len;
	os_memset(&msg, 0, sizeof(msg));
	msg.msg_iov = io;
	msg.msg_iovlen = 2;

	idx = 0;
	while (dst) {
		next = dst->next;
		if (level >= dst->debug_level) {
			wpa_hexdump(MSG_DEBUG, "CTRL_IFACE monitor send",
				    (u8 *) dst->addr.sun_path, dst->addrlen -
				    offsetof(struct sockaddr_un, sun_path));
			msg.msg_name = &dst->addr;
			msg.msg_namelen = dst->addrlen;
			if (sendmsg(hapd->ctrl_sock, &msg, 0) < 0) {
				int _errno = errno;
				wpa_printf(MSG_INFO, "CTRL_IFACE monitor[%d]: "
					   "%d - %s",
					   idx, errno, strerror(errno));
				dst->errors++;
				if (dst->errors > 10 || _errno == ENOENT) {
					hostapd_ctrl_iface_detach(
						hapd, &dst->addr,
						dst->addrlen);
				}
			} else
				dst->errors = 0;
		}
		idx++;
		dst = next;
	}
}

#endif /* CONFIG_NATIVE_WINDOWS */
