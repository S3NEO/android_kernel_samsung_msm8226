/* Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef MDSS_MAGNA_720P_PANEL_H
#define MDSS_MAGNA_720P_PANEL_H

#include "mdss_panel.h"
#include "smart_dimming.h"
#include "smart_mtp_ea8061.h"

#define MAX_BRIGHTNESS_LEVEL 255
#define MID_BRIGHTNESS_LEVEL 143
#define LOW_BRIGHTNESS_LEVEL 15
#define DIM_BRIGHTNESS_LEVEL 25

#define BL_MIN_BRIGHTNESS		6
#define BL_MAX_BRIGHTNESS_LEVEL		192
#define BL_MID_BRIGHTNESS_LEVEL		94
#define BL_LOW_BRIGHTNESS_LEVEL		7
#define BL_DIM_BRIGHTNESS_LEVEL		13
#define BL_DEFAULT_BRIGHTNESS		BL_MID_BRIGHTNESS_LEVEL

#define MAX_PANEL_NAME_SIZE 100
#define LCD_DEBUG(X, ...) pr_info("[LCD]%s:"X, __func__, ## __VA_ARGS__);

enum mipi_samsung_cmd_list {
	PANEL_READY_TO_ON,
	PANEL_DISP_OFF,
	PANEL_DISPLAY_ON,
	PANEL_DISPLAY_OFF,
	PANEL_DISPLAY_UNBLANK,
	PANEL_DISPLAY_BLANK,
	PANEL_ALL_PIXEL_OFF,
	PANEL_BRIGHT_CTRL,
	PANEL_MTP_ENABLE,
	PANEL_MTP_DISABLE,
	PANEL_NEED_FLIP,
	PANEL_ACL_OFF,
	PANEL_ACL_ON,
	PANEL_LATE_ON,
	PANEL_EARLY_OFF,
	PANEL_TOUCHSENSING_ON,
	PANEL_TOUCHSENSING_OFF,
	PANEL_TEAR_ON,
	PANEL_TEAR_OFF,
	PANEL_LDI_FPS_CHANGE,
	PANEL_LDI_SET_VDDM_OFFSET, /*LDI_ADJ_VDDM_OFFSET*/
	PANEL_PARTIAL_ON,
	PANEL_PARTIAL_OFF,
	PANEl_FORCE_500CD,
	PANEL_MTP_READ,
	MANUFACTURE_READ,
#if defined(HBM_RE)
	PANEL_HBM_READ
#endif
};

enum {
	MIPI_RESUME_STATE,
	MIPI_SUSPEND_STATE,
};

struct cmd_map {
	int *bl_level;
	int *cmd_idx;
	int size;
};
struct candella_lux_map {
	int *lux_tab;
	int *cmd_idx;
	int lux_tab_size;
	int bkl[256];
};
 struct display_status{
	unsigned char acl_on;
	unsigned char curr_acl_cond;
	unsigned char is_smart_dim_loaded;
	unsigned char is_mdnie_loaded;
	unsigned char auto_brightness;
	unsigned char recovery_boot_mode;
	unsigned char on;
	unsigned char wait_disp_on;
	int curr_elvss_idx;
	int curr_acl_idx;
	int curr_aid_idx;
	int curr_gamma_idx;
	int bright_level;
	int recent_bright_level;
	int temperature;
	char temperature_value;
	int temper_need_update;
	int siop_status;
	int hbm_mode;
 };

 struct mdss_samsung_driver_data{
	struct display_status dstat;
	struct msm_fb_data_type *mfd;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mutex lock;
#if defined(CONFIG_LCD_CLASS_DEVICE)
	struct platform_device *msm_pdev;
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	char panel_name[MAX_PANEL_NAME_SIZE];
	int panel;
	unsigned int manufacture_id;
	unsigned int id3;
	unsigned int panel_350cd;
	struct smartdim_conf *sdimconf;
 };
struct panel_hrev {
	char *name;
	int panel_code;
};
enum {
	PANEL_720P_OCTA_EA8061,
};

#endif
void mdss_dsi_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_cmd_desc *cmds, int cnt, int flag);
