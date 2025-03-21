/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * Copyright (C) 2024 Intel Corporation
 */
#ifndef __iwl_mld_h__
#define __iwl_mld_h__

#include "iwl-trans.h"
#include "iwl-op-mode.h"
#include "fw/runtime.h"
#include "fw/notif-wait.h"
#include "fw/api/mac-cfg.h"
#include "fw/api/mac.h"
#include "fw/api/phy-ctxt.h"

#define IWL_MLD_MAX_ADDRESSES		5

/**
 * struct iwl_mld - MLD op mode
 *
 * @fw_id_to_bss_conf: maps a fw id of a link to the corresponding
 *	ieee80211_bss_conf.
 * @fw_id_to_vif: maps a fw id of a MAC context to the corresponding
 *	ieee80211_vif. Mapping is valid only when the MAC exists in the fw.
 * @dev: pointer to device struct. For printing purposes
 * @trans: pointer to the transport layer
 * @cfg: pointer to the device configuration
 * @fw: a pointer to the fw object
 * @hw: pointer to the hw object.
 * @wiphy: a pointer to the wiphy struct, for easier access to it.
 * @nvm_data: pointer to the nvm_data that includes all our capabilities
 * @fwrt: fw runtime data
 * @debugfs_dir: debugfs directory
 * @notif_wait: notification wait related data.
 * @async_handlers_list: a list of all async RX handlers. When a notifciation
 *	with an async handler is received, it is added to this list.
 *	When &async_handlers_wk runs - it runs these handlers one by one.
 * @async_handlers_lock: a lock for &async_handlers_list. Sync
 *	&async_handlers_wk and RX notifcation path.
 * @async_handlers_wk: A work to run all async RX handlers from
 *	&async_handlers_list.
 * @fw_status: bitmap of fw status bits
 * @fw_status.in_hw_restart: indicates that we are currently in restart flow.
 * @addresses: device MAC addresses.
 * @scan_cmd_size: size of %scan_cmd.
 * @scan_cmd: pointer to scan_cmd buffer (allocated once in op mode start).
 * @wowlan: WoWLAN support data.
 * @mcc_src: the source id of the MCC, comes from the firmware
 */
struct iwl_mld {
	/* Add here fields that need clean up on restart */
	struct_group(zeroed_on_hw_restart,
		struct ieee80211_bss_conf __rcu *fw_id_to_bss_conf[IWL_FW_MAX_LINK_ID + 1];
		struct ieee80211_vif __rcu *fw_id_to_vif[NUM_MAC_INDEX_DRIVER];
	);
	/* And here fields that survive a fw restart */
	struct device *dev;
	struct iwl_trans *trans;
	const struct iwl_cfg *cfg;
	const struct iwl_fw *fw;
	struct ieee80211_hw *hw;
	struct wiphy *wiphy;
	struct iwl_nvm_data *nvm_data;
	struct iwl_fw_runtime fwrt;
	struct dentry *debugfs_dir;
	struct iwl_notif_wait_data notif_wait;
	struct list_head async_handlers_list;
	spinlock_t async_handlers_lock;
	struct wiphy_work async_handlers_wk;

	struct {
		u32 running:1,
		    do_not_dump_once:1,
		    in_hw_restart:1;
	} fw_status;
	struct mac_address addresses[IWL_MLD_MAX_ADDRESSES];
	size_t scan_cmd_size;
	void *scan_cmd;
#ifdef CONFIG_PM
	struct wiphy_wowlan_support wowlan;
#endif /* CONFIG_PM */
	enum iwl_mcc_source mcc_src;
};

/* memset the part of the struct that requires cleanup on restart */
#define CLEANUP_STRUCT(_ptr)				\
	memset((void *)&_ptr->zeroed_on_hw_restart, 0,	\
	       sizeof(_ptr->zeroed_on_hw_restart))

/* Cleanup function for struct iwl_mld_vif, will be called in restart */
static inline void
iwl_cleanup_mld(struct iwl_mld *mld)
{
	CLEANUP_STRUCT(mld);
}

enum iwl_power_scheme {
	IWL_POWER_SCHEME_CAM = 1,
	IWL_POWER_SCHEME_BPS,
};

/**
 * struct iwl_mld_mod_params - module parameters for iwlmld
 * @power_scheme: one of enum iwl_power_scheme
 */
struct iwl_mld_mod_params {
	int power_scheme;
};

extern struct iwl_mld_mod_params iwlmld_mod_params;

/* Extract MLD priv from op_mode */
#define IWL_OP_MODE_GET_MLD(_iwl_op_mode)		\
	((struct iwl_mld *)(_iwl_op_mode)->op_mode_specific)

#define IWL_MAC80211_GET_MLD(_hw)			\
	IWL_OP_MODE_GET_MLD((struct iwl_op_mode *)((_hw)->priv))

void
iwl_mld_add_debugfs_files(struct iwl_mld *mld, struct dentry *debugfs_dir);
int iwl_mld_run_fw_init_sequence(struct iwl_mld *mld);
int iwl_mld_load_fw(struct iwl_mld *mld);
void iwl_mld_stop_fw(struct iwl_mld *mld);
int iwl_mld_start_fw(struct iwl_mld *mld);

/* Rx */
void iwl_mld_rx_mpdu(struct iwl_mld *mld, struct napi_struct *napi,
		     struct iwl_rx_cmd_buffer *rxb, int queue);

static inline u8 iwl_mld_get_valid_tx_ant(const struct iwl_mld *mld)
{
	u8 tx_ant = mld->fw->valid_tx_ant;

	if (mld->nvm_data && mld->nvm_data->valid_tx_ant)
		tx_ant &= mld->nvm_data->valid_tx_ant;

	return tx_ant;
}

static inline u8 iwl_mld_get_valid_rx_ant(const struct iwl_mld *mld)
{
	u8 rx_ant = mld->fw->valid_rx_ant;

	if (mld->nvm_data && mld->nvm_data->valid_rx_ant)
		rx_ant &= mld->nvm_data->valid_rx_ant;

	return rx_ant;
}

static inline u8 iwl_mld_phy_band_to_nl80211(u8 phy_band)
{
	switch (phy_band) {
	case PHY_BAND_24:
		return NL80211_BAND_2GHZ;
	case PHY_BAND_5:
		return NL80211_BAND_5GHZ;
	case PHY_BAND_6:
		return NL80211_BAND_6GHZ;
	default:
		WARN_ONCE(1, "Unsupported phy band (%u)\n", phy_band);
		return NL80211_BAND_5GHZ;
	}
}

extern const struct ieee80211_ops iwl_mld_hw_ops;

#define IWL_MLD_INVALID_FW_ID 0xff

#define IWL_MLD_ALLOC_FN(_type, _mac80211_type)						\
static int										\
iwl_mld_allocate_##_type##_fw_id(struct iwl_mld *mld,					\
				 struct iwl_mld_##_type *ptr,				\
				 struct ieee80211_##_mac80211_type *mac80211_ptr)	\
{											\
	for (int i = 0; i < ARRAY_SIZE(mld->fw_id_to_##_mac80211_type); i++) {		\
		if (rcu_access_pointer(mld->fw_id_to_##_mac80211_type[i]))		\
			continue;							\
		ptr->fw_id = i;								\
		rcu_assign_pointer(mld->fw_id_to_##_mac80211_type[i], mac80211_ptr);	\
		return 0;								\
	}										\
	return -ENOSPC;									\
}

#endif /* __iwl_mld_h__ */
