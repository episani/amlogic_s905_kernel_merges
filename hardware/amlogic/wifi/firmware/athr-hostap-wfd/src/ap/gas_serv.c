/*
 * Generic advertisement service (GAS) server
 * Copyright (c) 2011-2012, Qualcomm Atheros, Inc.
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "includes.h"

#include "common.h"
#include "common/ieee802_11_defs.h"
#include "common/gas.h"
#include "utils/eloop.h"
#include "hostapd.h"
#include "ap_config.h"
#include "ap_drv_ops.h"
#include "sta_info.h"
#include "gas_serv.h"


struct anqp_query_info {
	unsigned int request;
	unsigned int remote_request;
	const void *param;
	u32 param_arg;
	u16 remote_delay;
};


static struct gas_dialog_info *
gas_dialog_create(struct hostapd_data *hapd, const u8 *addr, u8 dialog_token)
{
	struct sta_info *sta;
	struct gas_dialog_info *dia = NULL;
	int i, j;

	sta = ap_get_sta(hapd, addr);
	if (!sta) {
		/*
		 * We need a STA entry to be able to maintain state for
		 * the GAS query.
		 */
		wpa_printf(MSG_DEBUG, "ANQP: Add a temporary STA entry for "
			   "GAS query");
		sta = ap_sta_add(hapd, addr);
		if (!sta) {
			wpa_printf(MSG_DEBUG, "Failed to add STA " MACSTR
				   " for GAS query", MAC2STR(addr));
			return NULL;
		}
		sta->flags |= WLAN_STA_GAS;
		/*
		 * The default inactivity is 300 seconds. We don't need
		 * it to be that long.
		 */
		ap_sta_session_timeout(hapd, sta, 5);
	}

	if (sta->gas_dialog == NULL) {
		sta->gas_dialog = os_zalloc(GAS_DIALOG_MAX *
					    sizeof(struct gas_dialog_info));
		if (sta->gas_dialog == NULL)
			return NULL;
	}

	for (i = sta->gas_dialog_next, j = 0; j < GAS_DIALOG_MAX; i++, j++) {
		if (i == GAS_DIALOG_MAX)
			i = 0;
		if (sta->gas_dialog[i].valid)
			continue;
		dia = &sta->gas_dialog[i];
		dia->valid = 1;
		dia->index = i;
		dia->dialog_token = dialog_token;
		sta->gas_dialog_next = (++i == GAS_DIALOG_MAX) ? 0 : i;
		return dia;
	}

	wpa_msg(hapd->msg_ctx, MSG_ERROR, "ANQP: Could not create dialog for "
		MACSTR " dialog_token %u. Consider increasing "
		"GAS_DIALOG_MAX.", MAC2STR(addr), dialog_token);

	return NULL;
}


struct gas_dialog_info *
gas_serv_dialog_find(struct hostapd_data *hapd, const u8 *addr,
		     u8 dialog_token)
{
	struct sta_info *sta;
	int i;

	sta = ap_get_sta(hapd, addr);
	if (!sta) {
		wpa_printf(MSG_DEBUG, "ANQP: could not find STA " MACSTR,
			   MAC2STR(addr));
		return NULL;
	}
	for (i = 0; sta->gas_dialog && i < GAS_DIALOG_MAX; i++) {
		if (sta->gas_dialog[i].dialog_token != dialog_token ||
		    !sta->gas_dialog[i].valid)
			continue;
		return &sta->gas_dialog[i];
	}
	wpa_printf(MSG_DEBUG, "ANQP: Could not find dialog for "
		   MACSTR " dialog_token %u", MAC2STR(addr), dialog_token);
	return NULL;
}


void gas_serv_dialog_clear(struct gas_dialog_info *dia)
{
	wpabuf_free(dia->sd_resp);
	os_free(dia->venue_name);
	os_free(dia->net_auth_type);
	os_free(dia->roaming_consortium);
	os_free(dia->ipaddr_type);
	os_free(dia->nai_realm);
	os_free(dia->cell_net);
	os_free(dia->domain_name);
	os_free(dia->oper_friendly_name);
	os_free(dia->wan_metrics);
	os_free(dia->conn_capability);
	os_free(dia->nai_home_realm);
	os_memset(dia, 0, sizeof(*dia));
}


static void gas_serv_free_dialogs(struct hostapd_data *hapd,
				  const u8 *sta_addr)
{
	struct sta_info *sta;
	int i;

	sta = ap_get_sta(hapd, sta_addr);
	if (sta == NULL || sta->gas_dialog == NULL)
		return;

	for (i = 0; i < GAS_DIALOG_MAX; i++) {
		if (sta->gas_dialog[i].valid)
			return;
	}

	os_free(sta->gas_dialog);
	sta->gas_dialog = NULL;
}


static void anqp_add_hs_capab_list(struct hostapd_data *hapd,
				   struct wpabuf *buf)
{
	u8 *len;

	len = gas_anqp_add_element(buf, ANQP_VENDOR_SPECIFIC);
	wpabuf_put_be24(buf, OUI_WFA);
	wpabuf_put_u8(buf, HS20_ANQP_OUI_TYPE);
	wpabuf_put_u8(buf, HS20_STYPE_CAPABILITY_LIST);
	wpabuf_put_u8(buf, 0); /* Reserved */
	wpabuf_put_u8(buf, HS20_STYPE_CAPABILITY_LIST);
	if (hapd->conf->hs20_operator_friendly_name ||
	    (hapd->anqp_type_mask & ANQP_REQ_OPERATOR_FRIENDLY_NAME))
		wpabuf_put_u8(buf, HS20_STYPE_OPERATOR_FRIENDLY_NAME);
	if (hapd->conf->hs20_wan_metrics ||
	    (hapd->anqp_type_mask & ANQP_REQ_WAN_METRICS))
		wpabuf_put_u8(buf, HS20_STYPE_WAN_METRICS);
	if (hapd->conf->hs20_connection_capability ||
	    (hapd->anqp_type_mask & ANQP_REQ_CONNECTION_CAPABILITY))
		wpabuf_put_u8(buf, HS20_STYPE_CONNECTION_CAPABILITY);
	if (hapd->anqp_type_mask & ANQP_REQ_NAI_HOME_REALM)
		wpabuf_put_u8(buf, HS20_STYPE_NAI_HOME_REALM_QUERY);
	if (hapd->conf->hs20_operating_class ||
	    (hapd->anqp_type_mask & ANQP_REQ_OPERATING_CLASS))
		wpabuf_put_u8(buf, HS20_STYPE_OPERATING_CLASS);
	gas_anqp_set_element_len(buf, len);
}


static void anqp_add_capab_list(struct hostapd_data *hapd,
				struct wpabuf *buf)
{
	u8 *len;

	len = gas_anqp_add_element(buf, ANQP_CAPABILITY_LIST);
	wpabuf_put_le16(buf, ANQP_CAPABILITY_LIST);
	if (hapd->conf->venue_name || hapd->conf->hs20_venue_name ||
	    (hapd->anqp_type_mask & ANQP_REQ_VENUE_NAME))
		wpabuf_put_le16(buf, ANQP_VENUE_NAME);
	if (hapd->conf->hs20_network_auth_type ||
	    (hapd->anqp_type_mask & ANQP_REQ_NETWORK_AUTH_TYPE))
		wpabuf_put_le16(buf, ANQP_NETWORK_AUTH_TYPE);
	if (hapd->conf->roaming_consortium ||
	    (hapd->anqp_type_mask & ANQP_REQ_ROAMING_CONSORTIUM))
		wpabuf_put_le16(buf, ANQP_ROAMING_CONSORTIUM);
	if (hapd->conf->hs20_ipaddr_type_configured ||
	    (hapd->anqp_type_mask & ANQP_REQ_IP_ADDR_TYPE_AVAILABILITY))
		wpabuf_put_le16(buf, ANQP_IP_ADDR_TYPE_AVAILABILITY);
	if (hapd->conf->hs20_nai_realm_list ||
	    hapd->conf->nai_realm_data ||
	    (hapd->anqp_type_mask & ANQP_REQ_NAI_REALM))
		wpabuf_put_le16(buf, ANQP_NAI_REALM);
	if (hapd->conf->hs20_3gpp_cellular_network ||
	    (hapd->anqp_type_mask & ANQP_REQ_3GPP_CELLULAR_NETWORK))
		wpabuf_put_le16(buf, ANQP_3GPP_CELLULAR_NETWORK);
	if (hapd->conf->hs20_domain_name_list_value ||
	    (hapd->anqp_type_mask & ANQP_REQ_DOMAIN_NAME))
		wpabuf_put_le16(buf, ANQP_DOMAIN_NAME);
	anqp_add_hs_capab_list(hapd, buf);
	gas_anqp_set_element_len(buf, len);
}


static void anqp_add_venue_name(struct hostapd_data *hapd, struct wpabuf *buf,
				struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_venue_name) {
		wpabuf_put_le16(buf, ANQP_VENUE_NAME);
		wpabuf_put_le16(buf, hapd->conf->hs20_venue_name_len);
		wpabuf_put_data(buf, hapd->conf->hs20_venue_name,
				hapd->conf->hs20_venue_name_len);
	} else if (hapd->conf->venue_name) {
		u8 *len;
		unsigned int i;
		len = gas_anqp_add_element(buf, ANQP_VENUE_NAME);
		wpabuf_put_u8(buf, hapd->conf->venue_group);
		wpabuf_put_u8(buf, hapd->conf->venue_type);
		for (i = 0; i < hapd->conf->venue_name_count; i++) {
			struct hostapd_venue_name *vn;
			vn = &hapd->conf->venue_name[i];
			wpabuf_put_u8(buf, 3 + vn->name_len);
			wpabuf_put_data(buf, vn->lang, 3);
			wpabuf_put_data(buf, vn->name, vn->name_len);
		}
		gas_anqp_set_element_len(buf, len);
	} else if (di) {
		wpabuf_put_data(buf, di->venue_name, di->venue_name_len);
	}
}


static void anqp_add_network_auth_type(struct hostapd_data *hapd,
				       struct wpabuf *buf,
				       struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_network_auth_type) {
		wpabuf_put_le16(buf, ANQP_NETWORK_AUTH_TYPE);
		wpabuf_put_le16(buf, hapd->conf->hs20_network_auth_type_len);
		wpabuf_put_data(buf, hapd->conf->hs20_network_auth_type,
				hapd->conf->hs20_network_auth_type_len);
	} else if (di) {
		wpabuf_put_data(buf, di->net_auth_type, di->net_auth_type_len);
	}
}


static void anqp_add_roaming_consortium(struct hostapd_data *hapd,
					struct wpabuf *buf,
					struct gas_dialog_info *di)
{
	unsigned int i;
	u8 *len;

	if (di && di->roaming_consortium) {
		wpabuf_put_data(buf, di->roaming_consortium,
				di->roaming_consortium_len);
		return;
	}

	len = gas_anqp_add_element(buf, ANQP_ROAMING_CONSORTIUM);
	for (i = 0; i < hapd->conf->roaming_consortium_count; i++) {
		struct hostapd_roaming_consortium *rc;
		rc = &hapd->conf->roaming_consortium[i];
		wpabuf_put_u8(buf, rc->len);
		wpabuf_put_data(buf, rc->oi, rc->len);
	}
	gas_anqp_set_element_len(buf, len);
}


static void anqp_add_ip_addr_type_availability(struct hostapd_data *hapd,
					       struct wpabuf *buf,
					       struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_ipaddr_type_configured) {
		wpabuf_put_le16(buf, ANQP_IP_ADDR_TYPE_AVAILABILITY);
		wpabuf_put_le16(buf, 1);
		wpabuf_put_u8(buf, hapd->conf->hs20_ipaddr_type_availability);
	} else if (di) {
		wpabuf_put_data(buf, di->ipaddr_type, di->ipaddr_type_len);
	}
}


static void anqp_add_nai_realm_data(struct wpabuf *buf,
				    struct hostapd_nai_realm_data *realm,
				    unsigned int realm_idx)
{
	u8 *realm_data_len;
	unsigned int i;

	wpa_printf(MSG_DEBUG, "realm=%s, len=%d", realm->realm[realm_idx],
		   (int) os_strlen(realm->realm[realm_idx]));
	realm_data_len = wpabuf_put(buf, 2);
	wpabuf_put_u8(buf, realm->encoding);
	wpabuf_put_u8(buf, os_strlen(realm->realm[realm_idx]));
	wpabuf_put_str(buf, realm->realm[realm_idx]);
	if (realm->num_auths) {
		wpabuf_put_u8(buf, 1); /* EAP method count */
		wpabuf_put_u8(buf, 2 + (3 * realm->num_auths));
		wpabuf_put_u8(buf, realm->eap_method);
		wpabuf_put_u8(buf, realm->num_auths);
		for (i = 0; i < realm->num_auths; i++) {
			wpabuf_put_u8(buf, realm->auth_id[i]);
			wpabuf_put_u8(buf, 1);
			wpabuf_put_u8(buf, realm->auth_val[i]);
		}
	} else {
		wpabuf_put_u8(buf, 0); /* EAP method count */
	}
	gas_anqp_set_element_len(buf, realm_data_len);
}


static int process_hs_nai_home_realm(struct hostapd_data *hapd,
				     struct wpabuf *buf,
				     struct anqp_query_info *qi)
{
	unsigned int i, j, k;
	u8 num_realms, num_matching = 0, encoding, realm_len, *realm_list_len;
	struct hostapd_nai_realm_data *realm;
	const u8 *pos, *realm_name, *end;
	struct {
		unsigned int realm_data_idx;
		unsigned int realm_idx;
	} matches[10];

	pos = qi->param;
	end = pos + qi->param_arg;
	num_realms = *pos++;

	for (i = 0; i < num_realms && num_matching < 10; i++) {
		encoding = *pos++;
		realm_len = *pos++;
		realm_name = pos;
		for (j = 0; j < hapd->conf->nai_realm_count &&
			     num_matching < 10; j++) {
			realm = &hapd->conf->nai_realm_data[j];
			if (encoding != realm->encoding)
				continue;
			for (k = 0; realm->realm[k] && num_matching < 10; k++)
			{
				if (os_strncmp((char *) realm_name,
					       realm->realm[k], realm_len) !=
				    0)
					continue;
				matches[num_matching].realm_data_idx = j;
				matches[num_matching].realm_idx = k;
				num_matching++;
			}
		}
		pos += realm_len;
		if (pos > end) {
			wpa_hexdump(MSG_ERROR, "NAI Home Realm processing "
				    "error", qi->param, qi->param_arg);
			return -1;
		}
	}

	realm_list_len = gas_anqp_add_element(buf, ANQP_NAI_REALM);
	wpabuf_put_le16(buf, num_matching);

	/*
	 * There are two ways to format. 1. each realm in a NAI Realm Data unit
	 * 2. all realms that share the same EAP methods in a NAI Realm Data
	 * unit. The first format is likely to be bigger in size than the
	 * second, but may be easier to parse and process by the receiver.
	 */
	for (i = 0; i < num_matching; i++) {
		wpa_printf(MSG_DEBUG, "realm_idx %d, realm_data_idx %d",
			   matches[i].realm_data_idx, matches[i].realm_idx);
		realm = &hapd->conf->nai_realm_data[matches[i].realm_data_idx];
		anqp_add_nai_realm_data(buf, realm, matches[i].realm_idx);
	}
	gas_anqp_set_element_len(buf, realm_list_len);
	return 0;
}


static void anqp_add_nai_realm(struct hostapd_data *hapd, struct wpabuf *buf,
			       struct gas_dialog_info *di,
			       struct anqp_query_info *qi, int nai_realm,
			       int nai_home_realm)
{
	/* dynamically set nai_realm_list overrides static configuration */
	if (nai_realm && hapd->conf->hs20_nai_realm_list) {
		wpabuf_put_le16(buf, ANQP_NAI_REALM);
		wpabuf_put_le16(buf, hapd->conf->hs20_nai_realm_list_len);
		wpabuf_put_data(buf, hapd->conf->hs20_nai_realm_list,
				hapd->conf->hs20_nai_realm_list_len);
	} else if (nai_realm && hapd->conf->nai_realm_data) {
		u8 *len;
		unsigned int i, j;
		len = gas_anqp_add_element(buf, ANQP_NAI_REALM);
		wpabuf_put_le16(buf, hapd->conf->nai_realm_count);
		for (i = 0; i < hapd->conf->nai_realm_count; i++) {
			u8 *realm_data_len, *realm_len;
			struct hostapd_nai_realm_data *realm;

			realm = &hapd->conf->nai_realm_data[i];
			realm_data_len = wpabuf_put(buf, 2);
			wpabuf_put_u8(buf, realm->encoding);
			realm_len = wpabuf_put(buf, 1);
			for (j = 0; realm->realm[j]; j++) {
				if (j > 0)
					wpabuf_put_u8(buf, ';');
				wpabuf_put_str(buf, realm->realm[j]);
			}
			*realm_len = (u8 *) wpabuf_put(buf, 0) - realm_len - 1;
			if (realm->num_auths) {
				wpabuf_put_u8(buf, 1); /* EAP method count */
				wpabuf_put_u8(buf, 2 + (3 * realm->num_auths));
				wpabuf_put_u8(buf, realm->eap_method);
				wpabuf_put_u8(buf, realm->num_auths);
				for (j = 0; j < realm->num_auths; j++) {
					wpabuf_put_u8(buf, realm->auth_id[j]);
					wpabuf_put_u8(buf, 1);
					wpabuf_put_u8(buf, realm->auth_val[j]);
				}
			} else {
				wpabuf_put_u8(buf, 0); /* EAP method count */
			}
			gas_anqp_set_element_len(buf, realm_data_len);
		}
		gas_anqp_set_element_len(buf, len);
	} else if (nai_home_realm && hapd->conf->nai_realm_data) {
		process_hs_nai_home_realm(hapd, buf, qi);
	} else if (di) {
		if (nai_realm)
			wpabuf_put_data(buf, di->nai_realm,
					di->nai_realm_len);
		if (nai_home_realm)
			wpabuf_put_data(buf, di->nai_home_realm,
					di->nai_home_realm_len);
	}
}


static void anqp_add_3gpp_cellular_network(struct hostapd_data *hapd,
					   struct wpabuf *buf,
					   struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_3gpp_cellular_network) {
		wpabuf_put_le16(buf, ANQP_3GPP_CELLULAR_NETWORK);
		wpabuf_put_le16(buf,
				hapd->conf->hs20_3gpp_cellular_network_len);
		wpabuf_put_data(buf, hapd->conf->hs20_3gpp_cellular_network,
				hapd->conf->hs20_3gpp_cellular_network_len);
	} else if (di) {
		wpabuf_put_data(buf, di->cell_net, di->cell_net_len);
	}
}


static void anqp_add_domain_name(struct hostapd_data *hapd, struct wpabuf *buf,
				 struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_domain_name_list_value) {
		wpabuf_put_le16(buf, ANQP_DOMAIN_NAME);
		wpabuf_put_le16(buf,
				hapd->conf->hs20_domain_name_list_len);
		wpabuf_put_data(buf, hapd->conf->hs20_domain_name_list_value,
				hapd->conf->hs20_domain_name_list_len);
	} else if (di) {
		wpabuf_put_data(buf, di->domain_name, di->domain_name_len);
	}
}


static void anqp_add_operator_friendly_name(struct hostapd_data *hapd,
					    struct wpabuf *buf,
					    struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_operator_friendly_name) {
		u8 *len = gas_anqp_add_element(buf, ANQP_VENDOR_SPECIFIC);
		wpabuf_put_be24(buf, OUI_WFA);
		wpabuf_put_u8(buf, HS20_ANQP_OUI_TYPE);
		wpabuf_put_u8(buf, HS20_STYPE_OPERATOR_FRIENDLY_NAME);
		wpabuf_put_u8(buf, 0); /* Reserved */
		wpabuf_put_data(buf, hapd->conf->hs20_operator_friendly_name,
				hapd->conf->hs20_operator_friendly_name_len);
		gas_anqp_set_element_len(buf, len);
	} else if (di) {
		wpabuf_put_data(buf, di->oper_friendly_name,
				di->oper_friendly_name_len);
	}
}


static void anqp_add_wan_metrics(struct hostapd_data *hapd,
				 struct wpabuf *buf,
				 struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_wan_metrics) {
		u8 *len = gas_anqp_add_element(buf, ANQP_VENDOR_SPECIFIC);
		wpabuf_put_be24(buf, OUI_WFA);
		wpabuf_put_u8(buf, HS20_ANQP_OUI_TYPE);
		wpabuf_put_u8(buf, HS20_STYPE_WAN_METRICS);
		wpabuf_put_u8(buf, 0); /* Reserved */
		wpabuf_put_data(buf, hapd->conf->hs20_wan_metrics, 13);
		gas_anqp_set_element_len(buf, len);
	} else if (di) {
		wpabuf_put_data(buf, di->wan_metrics, di->wan_metrics_len);
	}
}


static void anqp_add_connection_capability(struct hostapd_data *hapd,
					   struct wpabuf *buf,
					   struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_connection_capability) {
		u8 *len = gas_anqp_add_element(buf, ANQP_VENDOR_SPECIFIC);
		wpabuf_put_be24(buf, OUI_WFA);
		wpabuf_put_u8(buf, HS20_ANQP_OUI_TYPE);
		wpabuf_put_u8(buf, HS20_STYPE_CONNECTION_CAPABILITY);
		wpabuf_put_u8(buf, 0); /* Reserved */
		wpabuf_put_data(buf, hapd->conf->hs20_connection_capability,
				hapd->conf->hs20_connection_capability_len);
		gas_anqp_set_element_len(buf, len);
	} else if (di) {
		wpabuf_put_data(buf, di->conn_capability,
				di->conn_capability_len);
	}
}


static void anqp_add_operating_class(struct hostapd_data *hapd,
				     struct wpabuf *buf,
				     struct gas_dialog_info *di)
{
	if (hapd->conf->hs20_operating_class) {
		u8 *len = gas_anqp_add_element(buf, ANQP_VENDOR_SPECIFIC);
		wpabuf_put_be24(buf, OUI_WFA);
		wpabuf_put_u8(buf, HS20_ANQP_OUI_TYPE);
		wpabuf_put_u8(buf, HS20_STYPE_OPERATING_CLASS);
		wpabuf_put_u8(buf, 0); /* Reserved */
		wpabuf_put_data(buf, hapd->conf->hs20_operating_class,
				hapd->conf->hs20_operating_class_len);
		gas_anqp_set_element_len(buf, len);
	} else if (di) {
		wpabuf_put_data(buf, di->operating_class,
				di->operating_class_len);
	}
}


static struct wpabuf *
gas_serv_build_gas_resp_payload(struct hostapd_data *hapd,
				unsigned int request,
				struct gas_dialog_info *di,
				struct anqp_query_info *qi)
{
	struct wpabuf *buf;

	buf = wpabuf_alloc(1400);
	if (buf == NULL)
		return NULL;

	if (request & ANQP_REQ_CAPABILITY_LIST)
		anqp_add_capab_list(hapd, buf);
	if (request & ANQP_REQ_VENUE_NAME)
		anqp_add_venue_name(hapd, buf, di);
	if (request & ANQP_REQ_NETWORK_AUTH_TYPE)
		anqp_add_network_auth_type(hapd, buf, di);
	if (request & ANQP_REQ_ROAMING_CONSORTIUM)
		anqp_add_roaming_consortium(hapd, buf, di);
	if (request & ANQP_REQ_IP_ADDR_TYPE_AVAILABILITY)
		anqp_add_ip_addr_type_availability(hapd, buf, di);
	if (request & (ANQP_REQ_NAI_REALM | ANQP_REQ_NAI_HOME_REALM))
		anqp_add_nai_realm(hapd, buf, di, qi,
				   request & ANQP_REQ_NAI_REALM,
				   request & ANQP_REQ_NAI_HOME_REALM);
	if (request & ANQP_REQ_3GPP_CELLULAR_NETWORK)
		anqp_add_3gpp_cellular_network(hapd, buf, di);
	if (request & ANQP_REQ_DOMAIN_NAME)
		anqp_add_domain_name(hapd, buf, di);

	if (request & ANQP_REQ_HS_CAPABILITY_LIST)
		anqp_add_hs_capab_list(hapd, buf);
	if (request & ANQP_REQ_OPERATOR_FRIENDLY_NAME)
		anqp_add_operator_friendly_name(hapd, buf, di);
	if (request & ANQP_REQ_WAN_METRICS)
		anqp_add_wan_metrics(hapd, buf, di);
	if (request & ANQP_REQ_CONNECTION_CAPABILITY)
		anqp_add_connection_capability(hapd, buf, di);
	if (request & ANQP_REQ_OPERATING_CLASS)
		anqp_add_operating_class(hapd, buf, di);

	return buf;
}


static int gas_serv_send_remote_anqp_req(struct hostapd_data *hapd,
					 const u8 *buf, size_t buf_len)
{
	if (hapd->anqp_remote_req == NULL)
		return -1;
	return hapd->anqp_remote_req(hapd, buf, buf_len);
}


static int gas_serv_remote_anqp_query(struct hostapd_data *hapd,
				      const u8 *sta_addr, u8 dialog_token,
				      unsigned int req)
{
	struct anqp_hdr hdr;

	os_memset(&hdr, 0, sizeof(hdr));
	hdr.anqp_type = host_to_be32(req);
	os_memcpy(hdr.sta_addr, sta_addr, ETH_ALEN);
	hdr.dialog_token = dialog_token;
	return gas_serv_send_remote_anqp_req(hapd, (u8 *) &hdr,
					     sizeof(struct anqp_hdr));
}


static int gas_serv_remote_anqp_query_data(struct hostapd_data *hapd,
					   const u8 *sta_addr, u8 dialog_token,
					   unsigned int req,
					   const u8 *data, size_t len)
{
	u8 *buf;
	struct anqp_hdr *hdr;
	int ret;

	if (data == NULL)
		return -1;

	buf = os_malloc(sizeof(*hdr) + len);
	if (buf == NULL)
		return -1;

	hdr = (struct anqp_hdr *) buf;
	os_memset(hdr, 0, sizeof(*hdr));
	hdr->anqp_type = host_to_be32(req);
	os_memcpy(hdr->sta_addr, sta_addr, ETH_ALEN);
	hdr->dialog_token = dialog_token;
	hdr->anqp_len = host_to_be32(len);
	os_memcpy(hdr->data, data, len);

	ret = gas_serv_send_remote_anqp_req(hapd, buf,
					    sizeof(struct anqp_hdr) + len);

	os_free(buf);

	return ret;
}


static int gas_serv_get_info(struct hostapd_data *hapd, unsigned int request,
			     const void *param, u32 param_arg,
			     const u8 *sta_addr, u8 dialog_token)
{
	int err = 0;

	if ((request & ANQP_REQ_VENUE_NAME) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       ANQP_VENUE_NAME) < 0)
		err++;

	if ((request & ANQP_REQ_NETWORK_AUTH_TYPE) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       ANQP_NETWORK_AUTH_TYPE) < 0)
		err++;

	if ((request & ANQP_REQ_ROAMING_CONSORTIUM) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       ANQP_ROAMING_CONSORTIUM) < 0)
		err++;

	if ((request & ANQP_REQ_IP_ADDR_TYPE_AVAILABILITY) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       ANQP_IP_ADDR_TYPE_AVAILABILITY) < 0)
		err++;

	if ((request & ANQP_REQ_NAI_REALM) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       ANQP_NAI_REALM) < 0)
		err++;

	if ((request & ANQP_REQ_3GPP_CELLULAR_NETWORK) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       ANQP_3GPP_CELLULAR_NETWORK) < 0)
		err++;

	if ((request & ANQP_REQ_DOMAIN_NAME) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       ANQP_DOMAIN_NAME) < 0)
		err++;

	if ((request & ANQP_REQ_OPERATOR_FRIENDLY_NAME) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       0x10000 |
				       HS20_STYPE_OPERATOR_FRIENDLY_NAME) < 0)
		err++;

	if ((request & ANQP_REQ_WAN_METRICS) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       0x10000 | HS20_STYPE_WAN_METRICS) < 0)
		err++;

	if ((request & ANQP_REQ_CONNECTION_CAPABILITY) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       0x10000 |
				       HS20_STYPE_CONNECTION_CAPABILITY)
	    < 0)
		err++;

	if ((request & ANQP_REQ_NAI_HOME_REALM) &&
	    gas_serv_remote_anqp_query_data(hapd, sta_addr, dialog_token,
					    0x10000 |
					    HS20_STYPE_NAI_HOME_REALM_QUERY,
					    param, param_arg) < 0)
		err++;

	if ((request & ANQP_REQ_OPERATING_CLASS) &&
	    gas_serv_remote_anqp_query(hapd, sta_addr, dialog_token,
				       0x10000 | HS20_STYPE_OPERATING_CLASS) <
	    0)
		err++;

	return err ? -1 : 0;
}


static void gas_serv_clear_cached_ies(void *eloop_data, void *user_ctx)
{
	struct gas_dialog_info *dia = eloop_data;

	wpa_printf(MSG_DEBUG, "GAS: Timeout triggered, clearing dialog for "
		   "dialog token %d", dia->dialog_token);

	gas_serv_dialog_clear(dia);
}


static void set_anqp_req(unsigned int bit, const char *name, int local,
			 unsigned int remote, u16 remote_delay,
			 struct anqp_query_info *qi)
{
	qi->request |= bit;
	if (local) {
		wpa_printf(MSG_DEBUG, "ANQP: %s (local)", name);
	} else if (bit & remote) {
		wpa_printf(MSG_DEBUG, "ANQP: %s (remote)", name);
		qi->remote_request |= bit;
		if (remote_delay > qi->remote_delay)
			qi->remote_delay = remote_delay;
	} else {
		wpa_printf(MSG_DEBUG, "ANQP: %s not available", name);
	}
}


static void rx_anqp_query_list_id(struct hostapd_data *hapd, u16 info_id,
				  struct anqp_query_info *qi)
{
	switch (info_id) {
	case ANQP_CAPABILITY_LIST:
		set_anqp_req(ANQP_REQ_CAPABILITY_LIST, "Capability List", 1, 0,
			     0, qi);
		break;
	case ANQP_VENUE_NAME:
		set_anqp_req(ANQP_REQ_VENUE_NAME, "Venue Name",
			     hapd->conf->hs20_venue_name != NULL ||
			     hapd->conf->venue_name != NULL,
			     hapd->anqp_type_mask,
			     hapd->venue_name_delay, qi);
		break;
	case ANQP_NETWORK_AUTH_TYPE:
		set_anqp_req(ANQP_REQ_NETWORK_AUTH_TYPE, "Network Auth Type",
			     hapd->conf->hs20_network_auth_type != NULL,
			     hapd->anqp_type_mask,
			     hapd->network_auth_type_delay, qi);
		break;
	case ANQP_ROAMING_CONSORTIUM:
		set_anqp_req(ANQP_REQ_ROAMING_CONSORTIUM, "Roaming Consortium",
			     hapd->conf->roaming_consortium != NULL &&
			     !(hapd->anqp_type_mask &
			       ANQP_REQ_ROAMING_CONSORTIUM),
			     hapd->anqp_type_mask,
			     hapd->roaming_consortium_list_delay, qi);
		break;
	case ANQP_IP_ADDR_TYPE_AVAILABILITY:
		set_anqp_req(ANQP_REQ_IP_ADDR_TYPE_AVAILABILITY,
			     "IP Addr Type Availability",
			     hapd->conf->hs20_ipaddr_type_configured,
			     hapd->anqp_type_mask,
			     hapd->ipaddr_type_delay, qi);
		break;
	case ANQP_NAI_REALM:
		set_anqp_req(ANQP_REQ_NAI_REALM, "NAI Realm",
			     hapd->conf->hs20_nai_realm_list != NULL ||
			     hapd->conf->nai_realm_data != NULL,
			     hapd->anqp_type_mask,
			     hapd->nai_realm_list_delay, qi);
		break;
	case ANQP_3GPP_CELLULAR_NETWORK:
		set_anqp_req(ANQP_REQ_3GPP_CELLULAR_NETWORK,
			     "3GPP Cellular Network",
			     hapd->conf->hs20_3gpp_cellular_network != NULL,
			     hapd->anqp_type_mask,
			     hapd->cellular_network_delay, qi);
		break;
	case ANQP_DOMAIN_NAME:
		set_anqp_req(ANQP_REQ_DOMAIN_NAME, "Domain Name",
			     hapd->conf->hs20_domain_name_list_value != NULL,
			     hapd->anqp_type_mask,
			     hapd->domain_name_list_delay, qi);
		break;
	default:
		wpa_printf(MSG_DEBUG, "ANQP: Unsupported Info Id %u",
			   info_id);
		break;
	}
}


static void rx_anqp_query_list(struct hostapd_data *hapd,
			       const u8 *pos, const u8 *end,
			       struct anqp_query_info *qi)
{
	wpa_printf(MSG_DEBUG, "ANQP: %u Info IDs requested in Query list",
		   (unsigned int) (end - pos) / 2);

	while (pos + 2 <= end) {
		rx_anqp_query_list_id(hapd, WPA_GET_LE16(pos), qi);
		pos += 2;
	}
}


static void rx_anqp_hs_query_list(struct hostapd_data *hapd, u8 subtype,
				  struct anqp_query_info *qi)
{
	switch (subtype) {
	case HS20_STYPE_CAPABILITY_LIST:
		set_anqp_req(ANQP_REQ_HS_CAPABILITY_LIST, "HS Capability List",
			     1, 0, 0, qi);
		break;
	case HS20_STYPE_OPERATOR_FRIENDLY_NAME:
		set_anqp_req(ANQP_REQ_OPERATOR_FRIENDLY_NAME,
			     "Operator Friendly Name",
			     hapd->conf->hs20_operator_friendly_name != NULL,
			     hapd->anqp_type_mask,
			     hapd->operator_friendly_name_delay, qi);
		break;
	case HS20_STYPE_WAN_METRICS:
		set_anqp_req(ANQP_REQ_WAN_METRICS, "WAN Metrics",
			     hapd->conf->hs20_wan_metrics != NULL,
			     hapd->anqp_type_mask,
			     hapd->wan_metrics_delay, qi);
		break;
	case HS20_STYPE_CONNECTION_CAPABILITY:
		set_anqp_req(ANQP_REQ_CONNECTION_CAPABILITY,
			     "Connection Capability",
			     hapd->conf->hs20_connection_capability != NULL,
			     hapd->anqp_type_mask,
			     hapd->connection_capability_delay, qi);
		break;
	case HS20_STYPE_OPERATING_CLASS:
		set_anqp_req(ANQP_REQ_OPERATING_CLASS, "Operating Class",
			     hapd->conf->hs20_operating_class != NULL,
			     hapd->anqp_type_mask,
			     hapd->operating_class_delay, qi);
		break;
	default:
		wpa_printf(MSG_DEBUG, "ANQP: Unsupported HS 2.0 subtype %u",
			   subtype);
		break;
	}
}


static void rx_anqp_hs_nai_home_realm(struct hostapd_data *hapd,
				      const u8 *pos, const u8 *end,
				      struct anqp_query_info *qi)
{
	qi->request |= ANQP_REQ_NAI_HOME_REALM;
	qi->param = pos;
	qi->param_arg = end - pos;
	if (hapd->conf->hs20_nai_realm_list != NULL ||
	    hapd->conf->nai_realm_data != NULL) {
		wpa_printf(MSG_DEBUG, "ANQP: HS 2.0 NAI Home Realm Query "
			   "(local)");
	} else if (hapd->anqp_type_mask & ANQP_REQ_NAI_HOME_REALM) {
		wpa_printf(MSG_DEBUG, "ANQP: HS 2.0 NAI Home Realm Query "
			   "(remote)");
		qi->remote_request |= ANQP_REQ_NAI_HOME_REALM;
		if (hapd->nai_home_realm_delay > qi->remote_delay)
			qi->remote_delay = hapd->nai_home_realm_delay;
	} else {
		wpa_printf(MSG_DEBUG, "ANQP: HS 2.0 NAI Home Realm Query not "
			   "available");
	}
}


static void rx_anqp_vendor_specific(struct hostapd_data *hapd,
				    const u8 *pos, const u8 *end,
				    struct anqp_query_info *qi)
{
	u32 oui;
	u8 subtype;

	if (pos + 4 > end) {
		wpa_printf(MSG_DEBUG, "ANQP: Too short vendor specific ANQP "
			   "Query element");
		return;
	}

	oui = WPA_GET_BE24(pos);
	pos += 3;
	if (oui != OUI_WFA) {
		wpa_printf(MSG_DEBUG, "ANQP: Unsupported vendor OUI %06x",
			   oui);
		return;
	}

	if (*pos != HS20_ANQP_OUI_TYPE) {
		wpa_printf(MSG_DEBUG, "ANQP: Unsupported WFA vendor type %u",
			   *pos);
		return;
	}
	pos++;

	if (pos + 1 >= end)
		return;

	subtype = *pos++;
	pos++; /* Reserved */
	switch (subtype) {
	case HS20_STYPE_QUERY_LIST:
		wpa_printf(MSG_DEBUG, "ANQP: HS 2.0 Query List");
		while (pos < end) {
			rx_anqp_hs_query_list(hapd, *pos, qi);
			pos++;
		}
		break;
	case HS20_STYPE_NAI_HOME_REALM_QUERY:
		rx_anqp_hs_nai_home_realm(hapd, pos, end, qi);
		break;
	default:
		wpa_printf(MSG_DEBUG, "ANQP: Unsupported HS 2.0 query subtype "
			   "%u", subtype);
		break;
	}
}


static void gas_serv_req_remote_processing(struct hostapd_data *hapd,
					   const u8 *sa, u8 dialog_token,
					   struct anqp_query_info *qi)
{
	struct gas_dialog_info *di;
	struct wpabuf *tx_buf = NULL;

	di = gas_dialog_create(hapd, sa, dialog_token);
	if (!di) {
		wpa_msg(hapd->msg_ctx, MSG_ERROR,
			"GAS: Could not create dialog for " MACSTR
			" (dialog token %u) ", MAC2STR(sa), dialog_token);
		return;
	}

	di->requested = qi->remote_request;
	di->received = 0;
	di->all_requested = qi->request;

	/* send the request to external agent */
	if (gas_serv_get_info(hapd, qi->remote_request, qi->param,
			      qi->param_arg, sa, dialog_token) < 0) {
		gas_serv_dialog_clear(di);
		tx_buf = gas_anqp_build_initial_resp_buf(
			dialog_token, WLAN_STATUS_ADV_SRV_UNREACHABLE, 0,
			NULL);
	} else if (qi->remote_delay >= GAS_SERV_MIN_COMEBACK_DELAY) {

		di->comeback_delay = qi->remote_delay;

		wpa_printf(MSG_DEBUG, "GAS: Tx GAS Initial Resp (comeback = "
			   "%d TU)",
			   qi->remote_delay + GAS_SERV_COMEBACK_DELAY_FUDGE);
		tx_buf = gas_anqp_build_initial_resp_buf(
			dialog_token, WLAN_STATUS_SUCCESS,
			qi->remote_delay + GAS_SERV_COMEBACK_DELAY_FUDGE,
			NULL);
	}

	if (!tx_buf)
		return;

	hostapd_drv_send_action(hapd, hapd->iface->freq, 0, sa,
				wpabuf_head(tx_buf), wpabuf_len(tx_buf));
	wpabuf_free(tx_buf);
}


static void gas_serv_req_local_processing(struct hostapd_data *hapd,
					  const u8 *sa, u8 dialog_token,
					  struct anqp_query_info *qi)
{
	struct wpabuf *buf, *tx_buf;

	buf = gas_serv_build_gas_resp_payload(hapd, qi->request, NULL, qi);
	wpa_hexdump_buf(MSG_MSGDUMP, "ANQP: Locally generated ANQP responses",
			buf);
	if (!buf)
		return;

	if (wpabuf_len(buf) > hapd->gas_frag_limit ||
	    hapd->conf->gas_comeback_delay) {
		struct gas_dialog_info *di;
		u16 comeback_delay = 1;

		if (hapd->conf->gas_comeback_delay) {
			/* Testing - allow overriding of the delay value */
			comeback_delay = hapd->conf->gas_comeback_delay;
		}

		wpa_printf(MSG_DEBUG, "ANQP: Too long response to fit in "
			   "initial response - use GAS comeback");
		di = gas_dialog_create(hapd, sa, dialog_token);
		if (!di) {
			wpa_printf(MSG_INFO, "ANQP: Could not create dialog "
				   "for " MACSTR " (dialog token %u)",
				   MAC2STR(sa), dialog_token);
			wpabuf_free(buf);
			return;
		}
		di->sd_resp = buf;
		di->sd_resp_pos = 0;
		tx_buf = gas_anqp_build_initial_resp_buf(
			dialog_token, WLAN_STATUS_SUCCESS, comeback_delay,
			NULL);
	} else {
		wpa_printf(MSG_DEBUG, "ANQP: Initial response (no comeback)");
		tx_buf = gas_anqp_build_initial_resp_buf(
			dialog_token, WLAN_STATUS_SUCCESS, 0, buf);
		wpabuf_free(buf);
	}
	if (!tx_buf)
		return;

	hostapd_drv_send_action(hapd, hapd->iface->freq, 0, sa,
				wpabuf_head(tx_buf), wpabuf_len(tx_buf));
	wpabuf_free(tx_buf);
}


static void gas_serv_rx_gas_initial_req(struct hostapd_data *hapd,
					const u8 *sa,
					const u8 *data, size_t len)
{
	const u8 *pos = data;
	const u8 *end = data + len;
	const u8 *next;
	u8 dialog_token;
	u16 slen;
	struct anqp_query_info qi;
	const u8 *adv_proto;

	if (len < 1 + 2)
		return;

	os_memset(&qi, 0, sizeof(qi));

	dialog_token = *pos++;
	wpa_msg(hapd->msg_ctx, MSG_DEBUG,
		"GAS: GAS Initial Request from " MACSTR " (dialog token %u) ",
		MAC2STR(sa), dialog_token);

	if (*pos != WLAN_EID_ADV_PROTO) {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG,
			"GAS: Unexpected IE in GAS Initial Request: %u", *pos);
		return;
	}
	adv_proto = pos++;

	slen = *pos++;
	next = pos + slen;
	if (next > end || slen < 2) {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG,
			"GAS: Invalid IE in GAS Initial Request");
		return;
	}
	pos++; /* skip QueryRespLenLimit and PAME-BI */

	if (*pos != ACCESS_NETWORK_QUERY_PROTOCOL) {
		struct wpabuf *buf;
		wpa_msg(hapd->msg_ctx, MSG_DEBUG,
			"GAS: Unsupported GAS advertisement protocol id %u",
			*pos);
		if (sa[0] & 0x01)
			return; /* Invalid source address - drop silently */
		buf = gas_build_initial_resp(
			dialog_token, WLAN_STATUS_GAS_ADV_PROTO_NOT_SUPPORTED,
			0, 2 + slen + 2);
		if (buf == NULL)
			return;
		wpabuf_put_data(buf, adv_proto, 2 + slen);
		wpabuf_put_le16(buf, 0); /* Query Response Length */
		hostapd_drv_send_action(hapd, hapd->iface->freq, 0, sa,
					wpabuf_head(buf), wpabuf_len(buf));
		wpabuf_free(buf);
		return;
	}

	pos = next;
	/* Query Request */
	if (pos + 2 > end)
		return;
	slen = WPA_GET_LE16(pos);
	pos += 2;
	if (pos + slen > end)
		return;
	end = pos + slen;

	/* ANQP Query Request */
	while (pos < end) {
		u16 info_id, elen;

		if (pos + 4 > end)
			return;

		info_id = WPA_GET_LE16(pos);
		pos += 2;
		elen = WPA_GET_LE16(pos);
		pos += 2;

		if (pos + elen > end) {
			wpa_printf(MSG_DEBUG, "ANQP: Invalid Query Request");
			return;
		}

		switch (info_id) {
		case ANQP_QUERY_LIST:
			rx_anqp_query_list(hapd, pos, pos + elen, &qi);
			break;
		case ANQP_VENDOR_SPECIFIC:
			rx_anqp_vendor_specific(hapd, pos, pos + elen, &qi);
			break;
		default:
			wpa_printf(MSG_DEBUG, "ANQP: Unsupported Query "
				   "Request element %u", info_id);
			break;
		}

		pos += elen;
	}

	if (qi.remote_request) {
		/*
		 * There is at least one element that needs to be fetched
		 * from the external anqpserver.
		 */
		gas_serv_req_remote_processing(hapd, sa, dialog_token, &qi);
	} else {
		gas_serv_req_local_processing(hapd, sa, dialog_token, &qi);
	}
}


void gas_serv_tx_gas_response(struct hostapd_data *hapd, const u8 *dst,
			      struct gas_dialog_info *dialog)
{
	struct wpabuf *buf, *tx_buf;
	u8 dialog_token = dialog->dialog_token;
	size_t frag_len;

	if (dialog->sd_resp == NULL) {
		buf = gas_serv_build_gas_resp_payload(hapd,
						      dialog->all_requested,
						      dialog, NULL);
		wpa_hexdump_buf(MSG_MSGDUMP, "ANQP: Generated ANQP responses",
			buf);
		if (!buf)
			goto tx_gas_response_done;
		dialog->sd_resp = buf;
		dialog->sd_resp_pos = 0;
	}
	frag_len = wpabuf_len(dialog->sd_resp) - dialog->sd_resp_pos;
	if (frag_len > hapd->gas_frag_limit || dialog->comeback_delay ||
	    hapd->conf->gas_comeback_delay) {
		u16 comeback_delay_tus = dialog->comeback_delay +
			GAS_SERV_COMEBACK_DELAY_FUDGE;
		u32 comeback_delay_secs, comeback_delay_usecs;

		if (hapd->conf->gas_comeback_delay) {
			/* Testing - allow overriding of the delay value */
			comeback_delay_tus = hapd->conf->gas_comeback_delay;
		}

		wpa_printf(MSG_DEBUG, "GAS: Response frag_len %u (frag limit "
			   "%u) and comeback delay %u, "
			   "requesting comebacks", (unsigned int) frag_len,
			   (unsigned int) hapd->gas_frag_limit,
			   dialog->comeback_delay);
		tx_buf = gas_anqp_build_initial_resp_buf(dialog_token,
							 WLAN_STATUS_SUCCESS,
							 comeback_delay_tus,
							 NULL);
		if (tx_buf) {
			wpa_msg(hapd->msg_ctx, MSG_DEBUG,
				"GAS: Tx GAS Initial Resp (comeback = 10TU)");
			hostapd_drv_send_action(hapd, hapd->iface->freq, 0,
						dst,
						wpabuf_head(tx_buf),
						wpabuf_len(tx_buf));
		}
		wpabuf_free(tx_buf);

		/* start a timer of 1.5 * comeback-delay */
		comeback_delay_tus = comeback_delay_tus +
			(comeback_delay_tus / 2);
		comeback_delay_secs = (comeback_delay_tus * 1024) / 1000000;
		comeback_delay_usecs = (comeback_delay_tus * 1024) -
			(comeback_delay_secs * 1000000);
		eloop_register_timeout(comeback_delay_secs,
				       comeback_delay_usecs,
				       gas_serv_clear_cached_ies, dialog,
				       NULL);
		goto tx_gas_response_done;
	}

	buf = wpabuf_alloc_copy(wpabuf_head_u8(dialog->sd_resp) +
				dialog->sd_resp_pos, frag_len);
	if (buf == NULL) {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: Buffer allocation "
			"failed");
		goto tx_gas_response_done;
	}
	tx_buf = gas_anqp_build_initial_resp_buf(dialog_token,
						 WLAN_STATUS_SUCCESS, 0, buf);
	wpabuf_free(buf);
	if (tx_buf == NULL)
		goto tx_gas_response_done;
	wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: Tx GAS Initial "
		"Response (frag_id %d frag_len %d)",
		dialog->sd_frag_id, (int) frag_len);
	dialog->sd_frag_id++;

	hostapd_drv_send_action(hapd, hapd->iface->freq, 0, dst,
				wpabuf_head(tx_buf), wpabuf_len(tx_buf));
	wpabuf_free(tx_buf);
tx_gas_response_done:
	gas_serv_clear_cached_ies(dialog, NULL);
}


static void gas_serv_rx_gas_comeback_req(struct hostapd_data *hapd,
					 const u8 *sa,
					 const u8 *data, size_t len)
{
	struct gas_dialog_info *dialog;
	struct wpabuf *buf, *tx_buf;
	u8 dialog_token;
	size_t frag_len;
	int more = 0;

	wpa_hexdump(MSG_DEBUG, "GAS: RX GAS Comeback Request", data, len);
	if (len < 1)
		return;
	dialog_token = *data;
	wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: Dialog Token: %u",
		dialog_token);

	dialog = gas_serv_dialog_find(hapd, sa, dialog_token);
	if (!dialog) {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: No pending SD "
			"response fragment for " MACSTR " dialog token %u",
			MAC2STR(sa), dialog_token);

		if (sa[0] & 0x01)
			return; /* Invalid source address - drop silently */
		tx_buf = gas_anqp_build_comeback_resp_buf(
			dialog_token, WLAN_STATUS_NO_OUTSTANDING_GAS_REQ, 0, 0,
			0, NULL);
		if (tx_buf == NULL)
			return;
		goto send_resp;
	}

	if (dialog->sd_resp == NULL) {
		wpa_printf(MSG_DEBUG, "GAS: Remote request 0x%x received 0x%x",
			   dialog->requested, dialog->received);
		if ((dialog->requested & dialog->received) !=
		    dialog->requested) {
			wpa_printf(MSG_DEBUG, "GAS: Did not receive response "
				   "from remote processing");
			gas_serv_dialog_clear(dialog);
			tx_buf = gas_anqp_build_comeback_resp_buf(
				dialog_token,
				WLAN_STATUS_GAS_RESP_NOT_RECEIVED, 0, 0, 0,
				NULL);
			if (tx_buf == NULL)
				return;
			goto send_resp;
		}

		buf = gas_serv_build_gas_resp_payload(hapd,
						      dialog->all_requested,
						      dialog, NULL);
		wpa_hexdump_buf(MSG_MSGDUMP, "ANQP: Generated ANQP responses",
			buf);
		if (!buf)
			goto rx_gas_comeback_req_done;
		dialog->sd_resp = buf;
		dialog->sd_resp_pos = 0;
	}
	frag_len = wpabuf_len(dialog->sd_resp) - dialog->sd_resp_pos;
	if (frag_len > hapd->gas_frag_limit) {
		frag_len = hapd->gas_frag_limit;
		more = 1;
	}
	wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: resp frag_len %u",
		(unsigned int) frag_len);
	buf = wpabuf_alloc_copy(wpabuf_head_u8(dialog->sd_resp) +
				dialog->sd_resp_pos, frag_len);
	if (buf == NULL) {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: Failed to allocate "
			"buffer");
		goto rx_gas_comeback_req_done;
	}
	tx_buf = gas_anqp_build_comeback_resp_buf(dialog_token,
						  WLAN_STATUS_SUCCESS,
						  dialog->sd_frag_id,
						  more, 0, buf);
	wpabuf_free(buf);
	if (tx_buf == NULL)
		goto rx_gas_comeback_req_done;
	wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: Tx GAS Comeback Response "
		"(frag_id %d more=%d frag_len=%d)",
		dialog->sd_frag_id, more, (int) frag_len);
	dialog->sd_frag_id++;
	dialog->sd_resp_pos += frag_len;

	if (more) {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: %d more bytes remain "
			"to be sent",
			(int) (wpabuf_len(dialog->sd_resp) -
			       dialog->sd_resp_pos));
	} else {
		wpa_msg(hapd->msg_ctx, MSG_DEBUG, "GAS: All fragments of "
			"SD response sent");
		gas_serv_dialog_clear(dialog);
		gas_serv_free_dialogs(hapd, sa);
	}

send_resp:
	hostapd_drv_send_action(hapd, hapd->iface->freq, 0, sa,
				wpabuf_head(tx_buf), wpabuf_len(tx_buf));
	wpabuf_free(tx_buf);
	return;

rx_gas_comeback_req_done:
	gas_serv_clear_cached_ies(dialog, NULL);
}


static void gas_serv_rx_public_action(void *ctx, const u8 *buf, size_t len,
				      int freq)
{
	struct hostapd_data *hapd = ctx;
	const struct ieee80211_mgmt *mgmt;
	size_t hdr_len;
	const u8 *sa, *data;

	mgmt = (const struct ieee80211_mgmt *) buf;
	hdr_len = (const u8 *) &mgmt->u.action.u.vs_public_action.action - buf;
	if (hdr_len > len)
		return;
	if (mgmt->u.action.category != WLAN_ACTION_PUBLIC)
		return;
	sa = mgmt->sa;
	len -= hdr_len;
	data = &mgmt->u.action.u.public_action.action;
	switch (data[0]) {
	case WLAN_PA_GAS_INITIAL_REQ:
		gas_serv_rx_gas_initial_req(hapd, sa, data + 1, len - 1);
		break;
	case WLAN_PA_GAS_COMEBACK_REQ:
		gas_serv_rx_gas_comeback_req(hapd, sa, data + 1, len - 1);
		break;
	}
}


int gas_serv_init(struct hostapd_data *hapd)
{
	hapd->public_action_cb = gas_serv_rx_public_action;
	hapd->public_action_cb_ctx = hapd;
	hapd->gas_frag_limit = 1400;
	if (hapd->conf->gas_frag_limit > 0)
		hapd->gas_frag_limit = hapd->conf->gas_frag_limit;
	return 0;
}


void gas_serv_deinit(struct hostapd_data *hapd)
{
}
