// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright (C) 2024 Intel Corporation
 */

#include <net/mac80211.h>

#include "fw/api/rx.h"
#include "fw/api/scan.h"
#include "fw/api/datapath.h"
#include "fw/dbg.h"

#include "mld.h"
#include "notif.h"
#include "mac80211.h"

#define DRV_DESCRIPTION "Intel(R) MLD wireless driver for Linux"
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IWLWIFI);

static const struct iwl_op_mode_ops iwl_mld_ops;

static int __init iwl_mld_init(void)
{
	int ret = iwl_opmode_register("iwlmld", &iwl_mld_ops);

	if (ret)
		pr_err("Unable to register MLD op_mode: %d\n", ret);

	return ret;
}
module_init(iwl_mld_init);

static void __exit iwl_mld_exit(void)
{
	iwl_opmode_deregister("iwlmld");
}
module_exit(iwl_mld_exit);

static bool
iwl_is_mld_op_mode_supported(struct iwl_trans *trans)
{
	/* TODO: Verify also by FW version */
	return trans->trans_cfg->device_family >= IWL_DEVICE_FAMILY_BZ;
}

static void
iwl_construct_mld(struct iwl_mld *mld, struct iwl_trans *trans,
		  const struct iwl_cfg *cfg, const struct iwl_fw *fw,
		  struct ieee80211_hw *hw, struct dentry *debugfs_dir)
{
	mld->dev = trans->dev;
	mld->trans = trans;
	mld->cfg = cfg;
	mld->fw = fw;
	mld->hw = hw;
	mld->wiphy = hw->wiphy;

#ifdef CPTCFG_IWLWIFI_DEBUGFS
	iwl_mld_add_debugfs_files(mld, debugfs_dir);
#endif

	iwl_notification_wait_init(&mld->notif_wait);

	/* Setup async RX handling */
	spin_lock_init(&mld->async_handlers_lock);
	INIT_LIST_HEAD(&mld->async_handlers_list);
	wiphy_work_init(&mld->async_handlers_wk,
			iwl_mld_async_handlers_wk);
}

static void __acquires(&mld->wiphy->mtx)
iwl_mld_fwrt_dump_start(void *ctx)
{
	struct iwl_mld *mld = ctx;

	wiphy_lock(mld->wiphy);
}

static void __releases(&mld->wiphy->mtx)
iwl_mld_fwrt_dump_end(void *ctx)
{
	struct iwl_mld *mld = ctx;

	wiphy_unlock(mld->wiphy);
}

static const struct iwl_fw_runtime_ops iwl_mld_fwrt_ops = {
	.dump_start = iwl_mld_fwrt_dump_start,
	.dump_end = iwl_mld_fwrt_dump_end,
};

static void
iwl_mld_construct_fw_runtime(struct iwl_mld *mld, struct iwl_trans *trans,
			     const struct iwl_fw *fw,
			     struct dentry *debugfs_dir)
{
	iwl_fw_runtime_init(&mld->fwrt, trans, fw, &iwl_mld_fwrt_ops, mld,
			    NULL, NULL, debugfs_dir);

	iwl_fw_set_current_image(&mld->fwrt, IWL_UCODE_REGULAR);
}

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search
 */
static const struct iwl_hcmd_names iwl_mld_legacy_names[] = {
	HCMD_NAME(UCODE_ALIVE_NTFY),
	HCMD_NAME(INIT_COMPLETE_NOTIF),
	HCMD_NAME(MFUART_LOAD_NOTIFICATION),
	HCMD_NAME(MCC_UPDATE_CMD),
};

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search
 */
static const struct iwl_hcmd_names iwl_mld_long_names[] = {
	HCMD_NAME(SCAN_CFG_CMD),
	HCMD_NAME(POWER_TABLE_CMD),
	HCMD_NAME(TX_ANT_CONFIGURATION_CMD),
	HCMD_NAME(RSS_CONFIG_CMD),
	HCMD_NAME(REPLY_BEACON_FILTERING_CMD),
	HCMD_NAME(LDBG_CONFIG_CMD),
};

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search
 */
static const struct iwl_hcmd_names iwl_mld_system_names[] = {
	HCMD_NAME(SHARED_MEM_CFG_CMD),
	HCMD_NAME(SOC_CONFIGURATION_CMD),
	HCMD_NAME(INIT_EXTENDED_CFG_CMD),
};

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search
 */
static const struct iwl_hcmd_names iwl_mld_reg_and_nvm_names[] = {
	HCMD_NAME(NVM_GET_INFO),
};

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search
 */
static const struct iwl_hcmd_names iwl_mld_debug_names[] = {
	HCMD_NAME(HOST_EVENT_CFG),
	HCMD_NAME(DBGC_SUSPEND_RESUME),
};

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search
 */
static const struct iwl_hcmd_names iwl_mld_mac_conf_names[] = {
	HCMD_NAME(MAC_CONFIG_CMD),
	HCMD_NAME(LINK_CONFIG_CMD),
};

/* Please keep this array *SORTED* by hex value.
 * Access is done through binary search
 */
static const struct iwl_hcmd_names iwl_mld_data_path_names[] = {
	HCMD_NAME(RFH_QUEUE_CONFIG_CMD),
};

VISIBLE_IF_IWLWIFI_KUNIT
const struct iwl_hcmd_arr iwl_mld_groups[] = {
	[LEGACY_GROUP] = HCMD_ARR(iwl_mld_legacy_names),
	[LONG_GROUP] = HCMD_ARR(iwl_mld_long_names),
	[SYSTEM_GROUP] = HCMD_ARR(iwl_mld_system_names),
	[MAC_CONF_GROUP] = HCMD_ARR(iwl_mld_mac_conf_names),
	[DATA_PATH_GROUP] = HCMD_ARR(iwl_mld_data_path_names),
	[REGULATORY_AND_NVM_GROUP] = HCMD_ARR(iwl_mld_reg_and_nvm_names),
	[DEBUG_GROUP] = HCMD_ARR(iwl_mld_debug_names),
};
EXPORT_SYMBOL_IF_IWLWIFI_KUNIT(iwl_mld_groups);

static void
iwl_mld_configure_trans(struct iwl_op_mode *op_mode)
{
	struct iwl_trans_config trans_cfg = {
		.op_mode = op_mode,
		/* Rx is not supported yet, but add it to avoid warnings */
		.rx_buf_size = iwl_amsdu_size_to_rxb_size(),
		.command_groups = iwl_mld_groups,
		.command_groups_size = ARRAY_SIZE(iwl_mld_groups),
		.fw_reset_handshake = true,
	};
	const struct iwl_mld *mld = IWL_OP_MODE_GET_MLD(op_mode);
	struct iwl_trans *trans = mld->trans;

	trans->rx_mpdu_cmd = REPLY_RX_MPDU_CMD;
	trans->iml = mld->fw->iml;
	trans->iml_len = mld->fw->iml_len;
	trans->wide_cmd_header = true;

	/*TODO: add more configurations here */

	iwl_trans_configure(trans, &trans_cfg);
}

static int
iwl_mld_alloc_scan_cmd(struct iwl_mld *mld)
{
	u8 scan_cmd_ver = iwl_fw_lookup_cmd_ver(mld->fw, SCAN_REQ_UMAC,
						IWL_FW_CMD_VER_UNKNOWN);
	size_t scan_cmd_size;

	if (scan_cmd_ver == 17) {
		scan_cmd_size = sizeof(struct iwl_scan_req_umac_v17);
	} else {
		IWL_ERR(mld, "Unexpected scan cmd version %d\n", scan_cmd_ver);
		return -EINVAL;
	}

	mld->scan_cmd = kmalloc(scan_cmd_size, GFP_KERNEL);
	if (!mld->scan_cmd)
		return -ENOMEM;

	mld->scan_cmd_size = scan_cmd_size;

	return 0;
}

/*
 *****************************************************
 * op mode ops functions
 *****************************************************
 */
static struct iwl_op_mode *
iwl_op_mode_mld_start(struct iwl_trans *trans, const struct iwl_cfg *cfg,
		      const struct iwl_fw *fw, struct dentry *dbgfs_dir)
{
	struct ieee80211_hw *hw;
	struct iwl_op_mode *op_mode;
	struct iwl_mld *mld;
	int ret;

	if (WARN_ON(!iwl_is_mld_op_mode_supported(trans)))
		return NULL;

	/* Allocate and initialize a new hardware device */
	hw = ieee80211_alloc_hw(sizeof(struct iwl_op_mode) +
				sizeof(struct iwl_mld),
				&iwl_mld_hw_ops);
	if (!hw)
		return NULL;

	op_mode = hw->priv;

	op_mode->ops = &iwl_mld_ops;

	mld = IWL_OP_MODE_GET_MLD(op_mode);

	iwl_construct_mld(mld, trans, cfg, fw, hw, dbgfs_dir);

	iwl_mld_construct_fw_runtime(mld, trans, fw, dbgfs_dir);

	/* Configure transport layer with the opmode specific params */
	iwl_mld_configure_trans(op_mode);

	/* Needed for sending commands */
	wiphy_lock(mld->wiphy);

	ret = iwl_mld_load_fw(mld);

	wiphy_unlock(mld->wiphy);

	if (ret)
		goto free_hw;

	iwl_mld_stop_fw(mld);

	if (iwl_mld_alloc_scan_cmd(mld))
		goto free_hw;

	if (iwl_mld_register_hw(mld))
		goto free_hw;

	return op_mode;

free_hw:
	ieee80211_free_hw(mld->hw);
	return NULL;
}

static void
iwl_op_mode_mld_stop(struct iwl_op_mode *op_mode)
{
	struct iwl_mld *mld = IWL_OP_MODE_GET_MLD(op_mode);

	ieee80211_unregister_hw(mld->hw);

	iwl_fw_runtime_free(&mld->fwrt);

	iwl_trans_op_mode_leave(mld->trans);

	kfree(mld->nvm_data);
	kfree(mld->scan_cmd);

	ieee80211_free_hw(mld->hw);
}

static void
iwl_mld_rx_rss(struct iwl_op_mode *op_mode, struct napi_struct *napi,
	       struct iwl_rx_cmd_buffer *rxb, unsigned int queue)
{
	/* TODO: add RX path :-) */
	WARN_ONCE(1, "RX is not supported yet\n");
}

static void
iwl_mld_queue_full(struct iwl_op_mode *op_mode, int hw_queue)
{
	/* TODO */
	WARN_ONCE(1, "Not supported yet\n");
}

static void
iwl_mld_queue_not_full(struct iwl_op_mode *op_mode, int hw_queue)
{
	/* TODO */
	WARN_ONCE(1, "Not supported yet\n");
}

static bool
iwl_mld_set_hw_rfkill_state(struct iwl_op_mode *op_mode, bool state)
{
	/* TODO */
	WARN_ONCE(1, "Not supported yet\n");
	return false;
}

static void
iwl_mld_free_skb(struct iwl_op_mode *op_mode, struct sk_buff *skb)
{
	/* TODO */
	WARN_ONCE(1, "Not supported yet\n");
}

static void iwl_mld_restart_nic(struct iwl_mld *mld)
{
	if (mld->fw_status.in_hw_restart) {
		/* TODO nested restarts */
		IWL_ERR(mld, "Nested restart. Not implemented\n");
		return;
	}

	/* TODO: get error recovery buffer (task=DP) */

	mld->fw_status.in_hw_restart = true;
	mld->fwrt.trans->dbg.restart_required = false;

	ieee80211_restart_hw(mld->hw);
}

static void
iwl_mld_nic_error(struct iwl_op_mode *op_mode, bool sync)
{
	struct iwl_mld *mld = IWL_OP_MODE_GET_MLD(op_mode);
	bool trans_dead = test_bit(STATUS_TRANS_DEAD, &mld->trans->status);

	if (!trans_dead && !mld->fw_status.do_not_dump_once)
		iwl_fwrt_dump_error_logs(&mld->fwrt);

	mld->fw_status.do_not_dump_once = false;

	/* WRT */
	iwl_fw_error_collect(&mld->fwrt, sync);

	/* Do restart only in the following conditions are met:
	 * 1. sync=false
	 *    (true means that the device is going to be shut down now)
	 * 2. trans is not dead
	 * 3. we consider the FW as running
	 *    (if 2 or 3 is not true -  there is nothing we can do anyway)
	 * 4. fw restart is allowed by module parameter
	 * 5. The trigger that brough us here is defined as one that requires
	 *    a restart (in the debug TLVs)
	 */
	if (sync || trans_dead || !mld->fw_status.running ||
	    !iwlwifi_mod_params.fw_restart ||
	    !mld->fwrt.trans->dbg.restart_required)
		return;

	iwl_mld_restart_nic(mld);
}

static void
iwl_mld_time_point(struct iwl_op_mode *op_mode,
		   enum iwl_fw_ini_time_point tp_id,
		   union iwl_dbg_tlv_tp_data *tp_data)
{
	struct iwl_mld *mld = IWL_OP_MODE_GET_MLD(op_mode);

	iwl_dbg_tlv_time_point(&mld->fwrt, tp_id, tp_data);
}

static const struct iwl_op_mode_ops iwl_mld_ops = {
	.start = iwl_op_mode_mld_start,
	.stop = iwl_op_mode_mld_stop,
	.rx = iwl_mld_rx,
	.rx_rss = iwl_mld_rx_rss,
	.queue_full = iwl_mld_queue_full,
	.queue_not_full = iwl_mld_queue_not_full,
	.hw_rf_kill = iwl_mld_set_hw_rfkill_state,
	.free_skb = iwl_mld_free_skb,
	.nic_error = iwl_mld_nic_error,
	.time_point = iwl_mld_time_point,
};

struct iwl_mld_mod_params iwlmld_mod_params = {
	.power_scheme = IWL_POWER_SCHEME_BPS,
};

module_param_named(power_scheme, iwlmld_mod_params.power_scheme, int, 0444);
MODULE_PARM_DESC(power_scheme,
		 "power management scheme: 1-active, 2-balanced, default: 2");
