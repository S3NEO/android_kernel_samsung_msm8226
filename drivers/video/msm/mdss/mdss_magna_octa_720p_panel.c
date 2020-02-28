/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/err.h>
#if defined (CONFIG_LCD_CLASS_DEVICE)
#include <linux/lcd.h>
#endif
#include <asm/system_info.h>
#include "mdss_fb.h"
#include "mdnie_lite_tuning.h"
#include "mdss_dsi.h"
#include "mdss_magna_octa_720p_panel.h"
#include "dlog.h"

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
struct work_struct  err_fg_work;
static int err_fg_gpio;
static int esd_count;
static int err_fg_working;
#define ESD_DEBUG 1
#endif

//#define DDI_VIDEO_ENHANCE_TUNING
#if defined(DDI_VIDEO_ENHANCE_TUNING)
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#endif
#if defined(CONFIG_LCD_CONNECTION_CHECK)
static int lcd_connected_status;
#endif
#define DT_CMD_HDR 6
#define NOT_USING
//#define CMD_DEBUG
//#define TEMPERATURE_ELVSS_S6E3FA0
//#define HBM_RE
DEFINE_LED_TRIGGER(bl_led_trigger);

static struct dsi_buf dsi_panel_tx_buf;
static struct dsi_buf dsi_panel_rx_buf;

static struct dsi_cmd display_qcom_on_cmds;
static struct dsi_cmd display_qcom_off_cmds;
static struct dsi_cmd display_unblank_cmd;
static struct dsi_cmd display_blank_cmd;
static struct dsi_cmd acl_cmds_list;
static struct dsi_cmd elvss_cmds_list;
static struct dsi_cmd aid_cmds_list_350;
static struct dsi_cmd gamma_cmds_list;
static struct dsi_cmd gamma_update_cmds;
static struct dsi_cmd aclcont_cmds_list;

static struct dsi_cmd nv_mtp_read_cmds;

static struct dsi_cmd nv_enable_cmds;
static struct dsi_cmd nv_disable_cmds;
static struct dsi_cmd mtp_read_cmd;
static struct dsi_cmd manufature_cmd;
static struct dsi_cmd manufature_cmd2;

static struct dsi_cmd manufacture_id_cmds;
static struct dsi_cmd acl_off_cmd;
static struct dsi_cmd elvss_lowtemp_cmds_list;
static struct dsi_cmd mps_control_cmds;

/*contains mapping between bl_level and
index number of corresponding acl command
in acl command list*/
static struct cmd_map acl_map_table;
static struct cmd_map elvss_map_table;
static struct cmd_map aid_map_table;
static struct cmd_map smart_acl_elvss_map_table;
static struct candella_lux_map candela_map_table_350;
static struct dsi_cmd smart_acl_elvss_cmds_list;

#if defined(HMB_RE)
static struct dsi_cmd nv_hbm_read_cmds;
static struct dsi_cmd nv_hbm_read_cmds2;
static struct dsi_cmd nv_hbm_read_cmds3;
static struct dsi_cmd hbm_read_cmd;
static struct dsi_cmd hbm_control_cmds;
char default_d4_value = 0x01;
#endif
static struct mdss_samsung_driver_data msd;
extern int poweroff_charging;

static int mdss_dsi_panel_dimming_init(struct mdss_panel_data *pdata);
u32 mdss_dsi_cmd_receive(struct mdss_dsi_ctrl_pdata *ctrl,
	struct dsi_cmd_desc *cmd, int rlen);

static struct  panel_hrev panel_supp_cdp[]= {
	{"SDC_AMS549BU01", PANEL_720P_OCTA_EA8061},
	{NULL}
};

static struct dsi_cmd_desc brightness_packet[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 0}, NULL},
};
#define MAX_BR_PACKET_SIZE sizeof(brightness_packet)/sizeof(struct dsi_cmd_desc)
static int mipi_samsung_disp_send_cmd(
		enum mipi_samsung_cmd_list cmd,
		unsigned char lock);
void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;

	if (!gpio_is_valid(ctrl->pwm_pmic_gpio)) {
		pr_err("%s: pwm_pmic_gpio=%d Invalid\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ret = gpio_request(ctrl->pwm_pmic_gpio, "disp_pwm");
	if (ret) {
		pr_err("%s: pwm_pmic_gpio=%d request failed\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: lpg_chan=%d pwm request failed", __func__,
				ctrl->pwm_lpg_chan);
		gpio_free(ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
	}
}
static int update_bright_packet(int cmd_count, struct dsi_cmd *cmd_set) {
	int i = 0;
	if (cmd_count > (MAX_BR_PACKET_SIZE - 1))
		/*cmd_count is index, if cmd_count >12 then panic*/
		panic("over max brightness_packet size(%d).. !!", MAX_BR_PACKET_SIZE);
	for (i = 0; i < cmd_set->num_of_cmds; i++) {
		brightness_packet[cmd_count].payload = \
			cmd_set->cmd_desc[i].payload;
		brightness_packet[cmd_count].dchdr.dlen = \
			cmd_set->cmd_desc[i].dchdr.dlen;
		brightness_packet[cmd_count].dchdr.dtype = \
			cmd_set->cmd_desc[i].dchdr.dtype;
		brightness_packet[cmd_count].dchdr.wait = \
			cmd_set->cmd_desc[i].dchdr.wait;
		cmd_count++;
	}
	return cmd_count;
}
static int get_candela_value(int bl_level)
{
	return candela_map_table_350.lux_tab[candela_map_table_350.bkl[bl_level]];
}
static int get_cmd_idx(int bl_level)
{
	return candela_map_table_350.cmd_idx[candela_map_table_350.bkl[bl_level]];
}
/*
This function takes aid_map_table and uses cd_idx,
to get the index of the command in aid command list.
*/
static struct dsi_cmd get_aid_aor_control_set(int cd_idx)
{
	struct dsi_cmd aid_control = {0,};
	int cmd_idx = 0, payload_size = 0;
	char *c_payload, *p_payload;
	int p_idx = msd.dstat.curr_aid_idx;

	if (!aid_map_table.size || !(cd_idx < aid_map_table.size))
		goto end;

	/* Get index in the aid command list*/
	cmd_idx = aid_map_table.cmd_idx[cd_idx];
	c_payload = aid_cmds_list_350.cmd_desc[cmd_idx].payload;

	/* Check if current & previous commands are same */
	if (p_idx >= 0) {
		p_payload = aid_cmds_list_350.cmd_desc[p_idx].payload;
		payload_size = aid_cmds_list_350.cmd_desc[p_idx].dchdr.dlen;
		if (!memcmp(p_payload, c_payload, payload_size))
			goto end;
	}
	/* Get the command desc */
	aid_control.cmd_desc = &(aid_cmds_list_350.cmd_desc[cmd_idx]);
	aid_control.num_of_cmds = 1;
	msd.dstat.curr_aid_idx = cmd_idx;
end:
	return aid_control;
}
/*
This function takes acl_map_table and uses cd_idx,
to get the index of the command in elvss command list.
*/
#if !defined(NOT_USING)
static struct dsi_cmd get_aclcont_control_set(void)
{
	struct dsi_cmd aclcont_control = {0,};
	int cmd_idx = 0;
	int acl_cond = msd.dstat.curr_acl_cond;
	if (acl_cond)
		/* already acl condition setted */
		goto end;
	/* Get the command desc */
	aclcont_control.cmd_desc = &(aclcont_cmds_list.cmd_desc[cmd_idx]);
	aclcont_control.num_of_cmds = 1;
	msd.dstat.curr_acl_cond = 1;
end:
	return aclcont_control;
}
#endif
/*
This function takes acl_map_table and uses cd_idx,
to get the index of the command in elvss command list.
*/
static struct dsi_cmd get_acl_control_set(int cd_idx)
{
	struct dsi_cmd acl_control = {0,};
	int cmd_idx = 0, payload_size = 0;
	char *c_payload, *p_payload;
	int p_idx = msd.dstat.curr_acl_idx;

	if (!acl_map_table.size || !(cd_idx < acl_map_table.size))
		goto end;

	/* Get index in the acl command list*/
	cmd_idx = acl_map_table.cmd_idx[cd_idx];
	c_payload = acl_cmds_list.cmd_desc[cmd_idx].payload;

	/* Check if current & previous commands are same */
	if (p_idx >= 0) {
		p_payload = acl_cmds_list.cmd_desc[p_idx].payload;
		payload_size = acl_cmds_list.cmd_desc[p_idx].dchdr.dlen;
		if (!memcmp(p_payload, c_payload, payload_size))
			goto end;
	}
	/* Get the command desc */
	acl_control.cmd_desc = &(acl_cmds_list.cmd_desc[cmd_idx]);
	acl_control.num_of_cmds = 1;
	msd.dstat.curr_acl_idx = cmd_idx;

end:
	return acl_control;
}
static struct dsi_cmd get_acl_control_off_set(void)
{
	struct dsi_cmd acl_control = {0,};
	int p_idx = msd.dstat.curr_acl_idx;

	/* Check if current & previous commands are same */
	if (p_idx == 0) {
		/* already acl off */
		goto end;
	}

	/* Get the command desc */
	acl_control.cmd_desc = &(acl_cmds_list.cmd_desc[0]); /* idx 0 : ACL OFF */

	acl_control.num_of_cmds = 1;
	msd.dstat.curr_acl_idx = 0;
	msd.dstat.curr_acl_cond = 0;
end:
	return acl_control;
}
#if defined(TEMPERATURE_ELVSS_S6E3FA0)
// ELVSS TEMPERATURE COMPENSATION for S6E3FA0
static struct dsi_cmd get_elvss_tempcompen_control_set(void)
{
	struct dsi_cmd elvss_tempcompen_control = {0,};
	/* Get the command desc */
	if (msd.dstat.temperature >= 0) {
		pr_debug("%s temp >= 0 \n",__func__);
		elvss_lowtemp_cmds_list.cmd_desc[1].payload[1] = 0x19;
		elvss_lowtemp_cmds_list.cmd_desc[2].payload[1] = 0x88;
	} else if (msd.dstat.temperature > -20) {
	pr_debug("%s 0 > temp > -20 \n",__func__);
	elvss_lowtemp_cmds_list.cmd_desc[1].payload[1] = 0x00;
	elvss_lowtemp_cmds_list.cmd_desc[2].payload[1] = 0x8C;
	} else {
		pr_debug("%s temp <= -20 \n",__func__);
		elvss_lowtemp_cmds_list.cmd_desc[1].payload[1] = 0x94;
		elvss_lowtemp_cmds_list.cmd_desc[2].payload[1] = 0x8C;
	}
	elvss_tempcompen_control.cmd_desc = elvss_lowtemp_cmds_list.cmd_desc;
	elvss_tempcompen_control.num_of_cmds = elvss_lowtemp_cmds_list.num_of_cmds;

	return elvss_tempcompen_control;
}
#endif
static struct dsi_cmd get_elvss_control_set(int cd_idx)
{
	struct dsi_cmd elvss_control = {0,};
	int cmd_idx = 0;
	char *payload;
	pr_info("%s for SMART_ACL size %d\n",__func__, elvss_map_table.size);
	if (!elvss_map_table.size || !(cd_idx < elvss_map_table.size) ||
		!smart_acl_elvss_map_table.size ||
		!(cd_idx < smart_acl_elvss_map_table.size)) {
		pr_err("%s failed mapping elvss table\n",__func__);
		goto end;
	}
	/* Get the command desc */
	if (msd.dstat.acl_on || msd.dstat.siop_status) {
		cmd_idx = smart_acl_elvss_map_table.cmd_idx[cd_idx];
		payload = smart_acl_elvss_cmds_list.cmd_desc[cmd_idx].payload;
		elvss_control.cmd_desc = &(smart_acl_elvss_cmds_list.cmd_desc[cmd_idx]);
		pr_info("ELVSS for SMART_ACL cd_idx=%d, cmd_idx=%d\n", cd_idx, cmd_idx);
	} else {
		cmd_idx = elvss_map_table.cmd_idx[cd_idx];
		payload = elvss_cmds_list.cmd_desc[cmd_idx].payload;
		elvss_control.cmd_desc = &(elvss_cmds_list.cmd_desc[cmd_idx]);
		pr_info("ELVSS for normal cd_idx=%d, cmd_idx=%d\n", cd_idx, cmd_idx);
	}
#if defined(TEMPERATURE_ELVSS)
// ELVSS lOW TEMPERATURE
	if (msd.dstat.auto_brightness != 6) { // if HBM is not set
		if (msd.dstat.temperature <= -20)
			elvss_control.cmd_desc = &(elvss_lowtemp_cmds_list.cmd_desc[cmd_idx]);
		 else
	}
#endif
	elvss_control.num_of_cmds = 1;
	msd.dstat.curr_elvss_idx = cmd_idx;
end:
	return elvss_control;
}
#if defined(HMB_RE)
static struct dsi_cmd get_hbm_control_set(void)
{
	struct dsi_cmd hbm_control = {0,};
	hbm_control.cmd_desc = &(hbm_control_cmds.cmd_desc[0]);
	hbm_control.num_of_cmds = hbm_control_cmds.num_of_cmds;
	return hbm_control;
}
#endif
static struct dsi_cmd get_mps_control_set(void)
{
	struct dsi_cmd mps_control = {0,};

	mps_control.cmd_desc = &(mps_control_cmds.cmd_desc[0]);
	mps_control.num_of_cmds = mps_control_cmds.num_of_cmds;
	return mps_control;
}

static struct dsi_cmd get_gamma_control_set(int candella)
{
	struct dsi_cmd gamma_control = {0,};
	/*  Just a safety check to ensure smart dimming data is initialised well */
	BUG_ON(msd.sdimconf->generate_gamma == NULL);
	msd.sdimconf->generate_gamma(candella, &gamma_cmds_list.cmd_desc[1].payload[1]);

	gamma_control.cmd_desc = &(gamma_cmds_list.cmd_desc[0]);
	gamma_control.num_of_cmds = gamma_cmds_list.num_of_cmds;
	return gamma_control;
}

static struct dsi_cmd get_gamma_update_set(void)
{
	struct dsi_cmd gamma_update = {0,};

	gamma_update.cmd_desc = &(gamma_update_cmds.cmd_desc[0]);
	gamma_update.num_of_cmds =gamma_update_cmds.num_of_cmds;
	return gamma_update;
}
static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	ret = pwm_config(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
}
#if defined(HMB_RE)
static int make_brightcontrol_hbm_set(int bl_level)
{
	struct dsi_cmd hbm_control = {0,};
	int cmd_count = 0;
	int a = 0;

	if (msd.dstat.hbm_mode) {
		pr_err("%s : already hbm mode! return \n", __func__);
		return 0;
	}
	pr_info(" %s++ \n", __func__);

	mipi_samsung_disp_send_cmd(PANEL_HBM_READ, false);
	mdss_dsi_cmd_receive(msd.ctrl_pdata, &nv_hbm_read_cmds.cmd_desc[0], 21);
	for (a = 0; a < 21; a++)
		hbm_control_cmds.cmd_desc[1].payload[a + 1] = msd.ctrl_pdata->rx_buf.data[a];

	hbm_read_cmd.cmd_desc[0].payload[1] = 0xD4;
	mipi_samsung_disp_send_cmd(PANEL_HBM_READ, false);
	mdss_dsi_cmd_receive(msd.ctrl_pdata, &nv_hbm_read_cmds2.cmd_desc[0], 1);
	default_d4_value = msd.ctrl_pdata->rx_buf.data[0];

	hbm_read_cmd.cmd_desc[0].payload[1] = 0xDB;
	mipi_samsung_disp_send_cmd(PANEL_HBM_READ, false);
	mdss_dsi_cmd_receive(msd.ctrl_pdata, &nv_hbm_read_cmds3.cmd_desc[0], 1);
	hbm_control_cmds.cmd_desc[7].payload[18] = msd.ctrl_pdata->rx_buf.data[0];

#if defined(CMD_DEBUG)
{
	int i,j;
	for (i = 0; i < hbm_control_cmds.num_of_cmds; i++) {
		printk("[HBM] hbm_gamma_cmds_list : ");
		for (j = 0; j < hbm_control_cmds.cmd_desc[i].dchdr.dlen; j++)
			printk("%02x ",hbm_control_cmds.cmd_desc[i].payload[j]);
		printk("\n");
	}
}
#endif
	hbm_control = get_hbm_control_set();
	cmd_count = update_bright_packet(cmd_count, &hbm_control);

	/* for non hbm mode : reset */
	msd.dstat.curr_elvss_idx = -1;
	msd.dstat.curr_acl_idx = -1;
	msd.dstat.curr_aid_idx = -1;
	msd.dstat.curr_acl_cond = 0;
	pr_info(" %s-- \n", __func__);
	return cmd_count;
}
#endif
static int make_brightcontrol_set(int bl_level) {
	struct dsi_cmd elvss_control = {0, 0, 0, 0, 0};
	struct dsi_cmd acl_control = {0, 0, 0, 0, 0};
	struct dsi_cmd aid_control = {0, 0, 0, 0, 0};
	struct dsi_cmd gamma_control = {0, 0, 0, 0, 0};
	struct dsi_cmd gamma_update = {0, 0, 0, 0, 0};
	struct dsi_cmd aclcont_control = {0, 0, 0, 0, 0};
	struct dsi_cmd mps_control = {0, 0, 0, 0, 0};
#if defined(TEMPERATURE_ELVSS_S6E3FA0)
	struct dsi_cmd temperature_elvss_control = {0, 0, 0, 0, 0};
#endif
#if defined(TEMPERATURE_ELVSS)
	struct dsi_cmd elvss_compen_control = {0, 0, 0, 0, 0};
#endif
	int cmd_count = 0, cd_idx = 0, cd_level =0;
	/*gamma*/

	cd_idx = get_cmd_idx(bl_level);
	cd_level = get_candela_value(bl_level);

	/* gamma control */
	gamma_control = get_gamma_control_set(cd_level);
	cmd_count = update_bright_packet(cmd_count, &gamma_control);
	/* aid/aor */
	aid_control = get_aid_aor_control_set(cd_idx);
	cmd_count = update_bright_packet(cmd_count, &aid_control);
	/* gamma update */
	gamma_update = get_gamma_update_set();
	cmd_count = update_bright_packet(cmd_count, &gamma_update);
	/*elvss*/
	elvss_control = get_elvss_control_set(cd_idx);
	cmd_count = update_bright_packet(cmd_count, &elvss_control);

	mps_control = get_mps_control_set();
	cmd_count = update_bright_packet(cmd_count, &mps_control);
#if defined(TEMPERATURE_ELVSS_S6E3FA0)
	// ELVSS TEMPERATURE COMPENSATION
	// ELVSS for Temperature set cmd should be sent after normal elvss set cmd
	temperature_elvss_control = get_elvss_tempcompen_control_set();
	cmd_count = update_bright_packet(cmd_count, &temperature_elvss_control);
#endif
#if defined(TEMPERATURE_ELVSS)
	// ELVSS TEMPERATURE COMPENSATION
	if (msd.dstat.temper_need_update) {
		elvss_compen_control = get_elvss_tempcompen_control_set();
		cmd_count = update_bright_packet(cmd_count, &elvss_compen_control);
		msd.dstat.temper_need_update = 0;
	}
#endif
	/* acl */
	if (msd.dstat.acl_on || msd.dstat.siop_status) {
#if !defined(NOT_USING)
		aclcont_control = get_aclcont_control_set();
		cmd_count = update_bright_packet(cmd_count, &aclcont_control);
#endif
		acl_control = get_acl_control_set(cd_idx);
		cmd_count = update_bright_packet(cmd_count, &acl_control);
	} else {
		/* acl off (hbm off) */
		acl_control = get_acl_control_off_set();
		cmd_count = update_bright_packet(cmd_count, &acl_control);
	}
#if defined(TEMPERATURE_ELVSS_S6E3FA0)
	LCD_DEBUG("bright_level: %d, candela_idx: %d( %d cd ), "\
	"cmd_count(aid,acl,acl_ctrl,elvss,temperature,gamma)::(%d,%d,%d,%d,%d,%d)%d\n",
#else
	LCD_DEBUG("bright_level: %d, candela_idx: %d( %d cd ), "\
	"cmd_count(aid,acl,acl_ctrl,elvss,temperature,gamma)::(%d,%d,%d,%d,%d)%d\n",
#endif
	msd.dstat.bright_level, cd_idx, cd_level,
	aid_control.num_of_cmds,
	acl_control.num_of_cmds,
	aclcont_control.num_of_cmds,
	elvss_control.num_of_cmds,
#if defined(TEMPERATURE_ELVSS_S6E3FA0)
	temperature_elvss_control.num_of_cmds,
#endif
	gamma_control.num_of_cmds, cmd_count);
	return cmd_count;
}

void mdss_dsi_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_cmd_desc *cmds, int cnt,int flag)
{
	struct dcs_cmd_req cmdreq;
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return;
	}
#endif

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmds;
	cmdreq.cmds_cnt = cnt;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;


	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

}

u32 mdss_dsi_cmd_receive(struct mdss_dsi_ctrl_pdata *ctrl,
	struct dsi_cmd_desc *cmd, int rlen)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rbuf = ctrl->rx_buf.data;
	cmdreq.rlen = rlen;
	cmdreq.cb = NULL; /* call back */
	/*
	 * This mutex is to sync up with dynamic FPS changes
	 * so that DSI lockups shall not happen
	 */
	BUG_ON(msd.ctrl_pdata == NULL);

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	/*
	 * blocked here, untill call back called
	 */
	return ctrl->rx_buf.len;
}

static int samsung_nv_read(struct dsi_cmd_desc *desc, char *destBuffer,
	int srcLength, struct mdss_panel_data *pdata, int startoffset)
{
	int loop_limit = 0;
	/* first byte is size of Register */
	static char packet_size[] = { 0x07, 0 };
	static struct dsi_cmd_desc s6e8aa0_packet_size_cmd = {
		{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0, sizeof(packet_size)},
		packet_size };
	/* second byte is Read-position */
	static char reg_read_pos[] = { 0xB0, 0x00 };
	static struct dsi_cmd_desc s6e8aa0_read_pos_cmd = {
		{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(reg_read_pos)},
		reg_read_pos };

	int read_pos = startoffset;
	int read_count = 0;
	int show_cnt;
	int i, j;
	char show_buffer[256];
	int show_buffer_pos = 0;
	int read_size = 0;

	show_buffer_pos +=
		snprintf(show_buffer, 256, "read_reg : %X[%d] : ",
		desc[0].payload[0], srcLength);
	loop_limit = (srcLength + packet_size[0] - 1)
		/ packet_size[0];
	mdss_dsi_cmds_send(msd.ctrl_pdata, &(s6e8aa0_packet_size_cmd), 1, 0);
	show_cnt = 0;
	for (j = 0; j < loop_limit; j++) {
		reg_read_pos[1] = read_pos;
		read_size = ((srcLength - read_pos + startoffset) < packet_size[0]) ?
			(srcLength - read_pos + startoffset) : packet_size[0];
		mdss_dsi_cmds_send(msd.ctrl_pdata, &(s6e8aa0_read_pos_cmd), 1, 0);
		read_count = mdss_dsi_cmd_receive(msd.ctrl_pdata, desc, read_size);
		for (i = 0; i < read_count; i++, show_cnt++) {
			show_buffer_pos += snprintf(show_buffer +
				show_buffer_pos, 256, "%02x ",
				msd.ctrl_pdata->rx_buf.data[i]);
			if (destBuffer != NULL && show_cnt < srcLength) {
				destBuffer[show_cnt] =
					msd.ctrl_pdata->rx_buf.data[i];
			}
		}
		show_buffer_pos += snprintf(show_buffer +
			show_buffer_pos, 256, ".");
		read_pos += read_count;
		if (read_pos-startoffset >= srcLength)
			break;
	}
	pr_info("%s\n", show_buffer);
	return read_pos-startoffset;
}

static int mipi_samsung_read_nv_mem(struct mdss_panel_data *pdata, struct dsi_cmd *nv_read_cmds, char *buffer)
{
	int nv_size = 0;
	int nv_read_cnt = 0;
	int i = 0;

	mipi_samsung_disp_send_cmd(PANEL_MTP_READ, true);
	mdss_dsi_cmd_receive(msd.ctrl_pdata,
		&manufature_cmd2.cmd_desc[0],
		33);

	mipi_samsung_disp_send_cmd(PANEL_MTP_ENABLE, true);
	for (i = 0; i < nv_read_cmds->num_of_cmds; i++)
		nv_size += nv_read_cmds->read_size[i];
	pr_info("nv_size= %d, nv_read_cmds->num_of_cmds = %d", nv_size, nv_read_cmds->num_of_cmds);
	for (i = 0; i < nv_read_cmds->num_of_cmds; i++) {
		int count = 0;
		int read_size = nv_read_cmds->read_size[i];
		int read_startoffset = nv_read_cmds->read_startoffset[i];
		count = samsung_nv_read(&(nv_read_cmds->cmd_desc[i]),
			&buffer[nv_read_cnt], read_size, pdata, read_startoffset);
		nv_read_cnt += count;
		if (count != read_size)
			pr_err("Error reading LCD NV data !!!!%d, %d\n", count, read_size);
	}
	mipi_samsung_disp_send_cmd(PANEL_MTP_DISABLE, true);
	return nv_read_cnt;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc=0;
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return 0;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return 0;
	}

	pr_info("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);
	if (enable) {
		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->lcd_ldo_en1, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL,GPIO_CFG_8MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("enabling disp_en_gpio_n failed, rc=%d\n",rc);

		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->lcd_ldo_en2, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL,GPIO_CFG_8MA),
					GPIO_CFG_ENABLE);

		if (rc)
			pr_err("enabling disp_en_gpio_n failed, rc=%d\n",rc);

		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->rst_gpio, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_PULL_UP,GPIO_CFG_8MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("disabling rst_gpio failed, rc=%d\n",rc);

		mdelay(20);
		gpio_set_value(ctrl_pdata->lcd_ldo_en1, 1);
		mdelay(5);
		gpio_set_value(ctrl_pdata->lcd_ldo_en2, 1);
		mdelay(1);

		gpio_set_value((ctrl_pdata->rst_gpio), 1);
		msleep(20);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		msleep(1);
		gpio_set_value((ctrl_pdata->rst_gpio), 1);
		msleep(20);

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->lcd_ldo_en1, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("disabling lcd_ldo_en1 failed, rc=%d\n",rc);

		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->lcd_ldo_en2, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("disabling lcd_ldo_en2 failed, rc=%d\n",rc);

		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->rst_gpio, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("disabling rst_gpio failed, rc=%d\n",rc);

		gpio_set_value(ctrl_pdata->lcd_ldo_en1, 0);
		gpio_set_value(ctrl_pdata->lcd_ldo_en2, 0);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}
	return 0;
}

static unsigned int mipi_samsung_manufacture_id(struct mdss_panel_data *pdata)
{
	struct dsi_buf *rp, *tp;
	unsigned int id = 0;

#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return -1;
	}
#endif

	if (!manufacture_id_cmds.num_of_cmds)
		return 0;
	tp = &dsi_panel_tx_buf;
	rp = &dsi_panel_rx_buf;

	mipi_samsung_disp_send_cmd(MANUFACTURE_READ, true);
	mdss_dsi_cmd_receive(msd.ctrl_pdata,
		&manufature_cmd2.cmd_desc[0],
		manufature_cmd2.read_size[0]);
	pr_info("%s: manufacture_id1=%x, %x, %x \n", __func__, msd.ctrl_pdata->rx_buf.data[0],
		msd.ctrl_pdata->rx_buf.data[1], msd.ctrl_pdata->rx_buf.data[2]);
	id = ((msd.ctrl_pdata->rx_buf.data[0]) << 16);
	id |= ((msd.ctrl_pdata->rx_buf.data[1]) << 8);
	id |= (msd.ctrl_pdata->rx_buf.data[2]);

	return id;
}
int first_boot = 0;
static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return;
	}
#endif

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	if( msd.mfd->panel_power_on == false){
		pr_err("%s: panel power off no bl ctrl\n", __func__);
		return;
	}
	if (!first_boot) {
		mdss_dsi_panel_dimming_init(pdata);
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
		enable_irq(err_fg_gpio);
#endif
		first_boot = 1;
	}

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	if (err_fg_working) {
		pr_info("[LCD] %s : esd is working!! return.. \n", __func__);
		return;
	}
#endif

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		msd.dstat.bright_level = bl_level;
		mipi_samsung_disp_send_cmd(PANEL_BRIGHT_CTRL, true);
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

static int mipi_samsung_disp_send_cmd(
		enum mipi_samsung_cmd_list cmd,
		unsigned char lock)
{
	struct dsi_cmd_desc *cmd_desc;
	int cmd_size = 0;
	int flag = 0;
#ifdef CMD_DEBUG
	int i,j;
#endif
	if (lock)
		mutex_lock(&msd.lock);

	switch (cmd) {
		case PANEL_READY_TO_ON:
			cmd_desc = display_qcom_on_cmds.cmd_desc;
			cmd_size = display_qcom_on_cmds.num_of_cmds;
			break;
		case PANEL_DISP_OFF:
			cmd_desc = display_qcom_off_cmds.cmd_desc;
			cmd_size = display_qcom_off_cmds.num_of_cmds;
			break;
		case PANEL_DISPLAY_UNBLANK:
			cmd_desc = display_unblank_cmd.cmd_desc;
			cmd_size = display_unblank_cmd.num_of_cmds;
			break;
		case PANEL_DISPLAY_BLANK:
			cmd_desc = display_blank_cmd.cmd_desc;
			cmd_size = display_blank_cmd.num_of_cmds;
			break;
		case PANEL_BRIGHT_CTRL:
			cmd_desc = brightness_packet;
			flag = 0;
			if (msd.dstat.bright_level)
				msd.dstat.recent_bright_level = msd.dstat.bright_level;
#if defined(HMB_RE)
			if (msd.dstat.auto_brightness == 6) {
				cmd_size = make_brightcontrol_hbm_set(msd.dstat.bright_level);
				msd.dstat.hbm_mode = 1;
			} else {
				msd.dstat.hbm_mode = 0;
				hbm_control_cmds.cmd_desc[7].payload[18] = default_d4_value;
				cmd_size = make_brightcontrol_set(msd.dstat.bright_level);
			}
#else
			cmd_size = make_brightcontrol_set(msd.dstat.bright_level);
#endif
			if (msd.mfd->resume_state != MIPI_RESUME_STATE) {
				pr_info("%s : panel is off state!!\n", __func__);
				goto unknown_command;
			}
			break;
		case PANEL_MTP_ENABLE:
			cmd_desc = nv_enable_cmds.cmd_desc;
			cmd_size = nv_enable_cmds.num_of_cmds;
			break;
		case PANEL_MTP_DISABLE:
			cmd_desc = nv_disable_cmds.cmd_desc;
			cmd_size = nv_disable_cmds.num_of_cmds;
			break;
		case PANEL_ACL_OFF:
			cmd_desc = acl_off_cmd.cmd_desc;
			cmd_size = acl_off_cmd.num_of_cmds;
			break;
		case PANEL_MTP_READ:
			cmd_desc = mtp_read_cmd.cmd_desc;
			cmd_size = mtp_read_cmd.num_of_cmds;
			break;
		case MANUFACTURE_READ:
			cmd_desc = manufature_cmd.cmd_desc;
			cmd_size = manufature_cmd.num_of_cmds;
			break;
#if defined(HBM_RE)
		case PANEL_HBM_READ:
			cmd_desc = hbm_read_cmd.cmd_desc;
			cmd_size = hbm_read_cmd.num_of_cmds;
			break;
#endif
		default:
			pr_err("%s : unknown_command.. \n", __func__);
			goto unknown_command;
			;
	}

	if (!cmd_size) {
		pr_err("%s : cmd_size is zero!.. \n", __func__);
		goto unknown_command;
	}

#ifdef CMD_DEBUG
	for (i = 0; i < cmd_size; i++)
	{
		for (j = 0; j < cmd_desc[i].dchdr.dlen; j++)
			printk("%x ",cmd_desc[i].payload[j]);
		printk("\n");
	}
#endif

	mdss_dsi_cmds_send(msd.ctrl_pdata, cmd_desc, cmd_size, flag);

	if (lock)
		mutex_unlock(&msd.lock);
	return 0;

unknown_command:
	LCD_DEBUG("Undefined command\n");

	if (lock)
		mutex_unlock(&msd.lock);

	return -EINVAL;
}
static int mdss_dsi_panel_registered(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	if (pdata == NULL) {
		pr_err("%s : Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
		panel_data);
	msd.mfd = (struct msm_fb_data_type *)registered_fb[0]->par;
	msd.pdata = pdata;
	msd.ctrl_pdata = ctrl_pdata;
#if defined(CONFIG_MDNIE_LITE_TUNING)
	pr_info("[%s] CONFIG_MDNIE_LITE_TUNING ok ! init class called!\n",
		__func__);
	mdnie_lite_tuning_init(&msd);
#endif
/* Set the initial state to Suspend until it is switched on */
	msd.mfd->resume_state = MIPI_RESUME_STATE;
	pr_info("%s:%d, Panel registered succesfully\n", __func__, __LINE__);
	return 0;
}
static int mdss_dsi_panel_dimming_init(struct mdss_panel_data *pdata)
{
#if defined(CONFIG_MDNIE_LITE_TUNING)
	char temp[4];
	int x, y;
#endif
	if (is_lcd_attached() == 0) {
		printk("%s: get_lcd_attached(0)!\n", __func__);
		return 0;
	}
	/* If the ID is not read yet, then read it*/
	pr_info(" %s ++\n", __func__);
	if (!msd.manufacture_id)
		msd.manufacture_id = mipi_samsung_manufacture_id(pdata);
	if (!msd.dstat.is_smart_dim_loaded) {
		switch (msd.panel) {
		case PANEL_720P_OCTA_EA8061:
			pr_info("%s : EA8061 \n", __func__);
			msd.sdimconf = smart_EA8061_get_conf();
			break;
		}
		/* Just a safety check to ensure smart dimming data is initialised well */
		BUG_ON(msd.sdimconf == NULL);

		/* Set the mtp read buffer pointer and read the NVM value*/
		mipi_samsung_read_nv_mem(pdata, &nv_mtp_read_cmds, msd.sdimconf->mtp_buffer);

		/* lux_tab setting for 350cd */
		msd.sdimconf->lux_tab = &candela_map_table_350.lux_tab[0];
		msd.sdimconf->lux_tabsize = candela_map_table_350.lux_tab_size;
		msd.sdimconf->man_id = msd.manufacture_id;

		/* Just a safety check to ensure smart dimming data is initialised well */
		BUG_ON(msd.sdimconf->init == NULL);
		msd.sdimconf->init();
		msd.dstat.is_smart_dim_loaded = true;

		/*
		 * Since dimming is loaded, we can assume that device is out of suspend state
		 * and can accept backlight commands.
		 */
		msd.mfd->resume_state = MIPI_RESUME_STATE;
	}
#if defined(CONFIG_MDNIE_LITE_TUNING)
	/* MDNIe tuning initialisation*/
	if (!msd.dstat.is_mdnie_loaded) {
		mipi_samsung_read_nv_mem(pdata, &nv_mdnie_read_cmds, temp);
		x =  temp[0] << 8 | temp[1];	/* X */
		y = temp[2] << 8 | temp[3];	/* Y */
		coordinate_tunning(x, y);
		msd.dstat.is_mdnie_loaded = true;
	}
#endif
	pr_info(" %s --\n", __func__);
	return 0;
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	pr_info(" %s ++\n", __func__);
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (!msd.manufacture_id)
		msd.manufacture_id = mipi_samsung_manufacture_id(pdata);

	mipi_samsung_disp_send_cmd(PANEL_READY_TO_ON, true);

	msd.dstat.curr_elvss_idx = -1;
	msd.dstat.curr_acl_idx = -1;
	msd.dstat.curr_aid_idx = -1;
	msd.dstat.hbm_mode = 0;
	msd.mfd->resume_state = MIPI_RESUME_STATE;

#if defined(CONFIG_MDNIE_VIDEO_ENHANCED)
	is_negative_on();
#endif
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	enable_irq(err_fg_gpio);
#endif
	if(msd.dstat.recent_bright_level)
	{
		msd.dstat.bright_level = msd.dstat.recent_bright_level;
		mipi_samsung_disp_send_cmd(PANEL_BRIGHT_CTRL, true);
	}
	pr_info(" %s --\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	pr_info(" %s ++\n", __func__);
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	if (!err_fg_working) {
		disable_irq_nosync(err_fg_gpio);
		cancel_work_sync(&err_fg_work);
	}
#endif
	msd.dstat.on = 0;
	msd.mfd->resume_state = MIPI_SUSPEND_STATE;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi_samsung_disp_send_cmd(PANEL_DISP_OFF, true);

	pr_info(" %s --\n", __func__);
	return 0;
}

static int mdss_samsung_parse_candella_lux_mapping_table(struct device_node *np,
	struct candella_lux_map *table, char *keystring)
{
	const __be32 *data;
	int  data_offset, len = 0 , i = 0;
	int  cdmap_start=0, cdmap_end=0;
	data = of_get_property(np, keystring, &len);
	if (!data) {
		pr_err("%s:%d, Unable to read table %s ",
			__func__, __LINE__, keystring);
		return -EINVAL;
	}
	if ((len % 4) != 0) {
		pr_err("%s:%d, Incorrect table entries for %s",
			__func__, __LINE__, keystring);
		return -EINVAL;
	}
	table->lux_tab_size = len / (sizeof(int)*4);
	table->lux_tab = kzalloc((sizeof(int) * table->lux_tab_size), GFP_KERNEL);
	if (!table->lux_tab)
		return -ENOMEM;
	table->cmd_idx = kzalloc((sizeof(int) * table->lux_tab_size), GFP_KERNEL);
	if (!table->cmd_idx)
		goto error;
	data_offset = 0;
	for (i = 0 ; i < table->lux_tab_size; i++) {
		table->cmd_idx[i]= be32_to_cpup(&data[data_offset++]);
		/* 1rst field => <idx> */
		cdmap_start = be32_to_cpup(&data[data_offset++]);
		/* 2nd field => <from> */
		cdmap_end = be32_to_cpup(&data[data_offset++]);
		/* 3rd field => <till> */
		table->lux_tab[i] = be32_to_cpup(&data[data_offset++]);
		/* 4th field => <candella> */
		/* Fill the backlight level to lux mapping array */
		do{
			table->bkl[cdmap_start++] = i;
		} while(cdmap_start <= cdmap_end);
	}
	return 0;
error:
	kfree(table->lux_tab);
	return -ENOMEM;
}

static int mdss_samsung_parse_panel_table(struct device_node *np,
	struct cmd_map *table, char *keystring)
{
	const __be32 *data;
	int  data_offset, len = 0 , i = 0;
	data = of_get_property(np, keystring, &len);
	if (!data) {
		pr_err("%s:%d, Unable to read table %s ",
			__func__, __LINE__, keystring);
		return -EINVAL;
	}
	if ((len % 2) != 0) {
		pr_err("%s:%d, Incorrect table entries for %s",
			__func__, __LINE__, keystring);
		return -EINVAL;
	}
	table->size = len / (sizeof(int)*2);
	table->bl_level = kzalloc((sizeof(int) * table->size), GFP_KERNEL);
	if (!table->bl_level)
		return -ENOMEM;
	table->cmd_idx = kzalloc((sizeof(int) * table->size), GFP_KERNEL);
	if (!table->cmd_idx)
		goto error;

	data_offset = 0;
	for (i = 0 ; i < table->size; i++) {
		table->bl_level[i] = be32_to_cpup(&data[data_offset++]);
		table->cmd_idx[i] = be32_to_cpup(&data[data_offset++]);
	}
	return 0;
error:
	kfree(table->cmd_idx);
	return -ENOMEM;
}

static int mdss_panel_dt_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}

static int mdss_samsung_parse_panel_cmd(struct device_node *np,
	struct dsi_cmd *commands, char *keystring) {
	const char *data;
	int type, len = 0, i = 0;
	char *bp;
	struct dsi_ctrl_hdr *dchdr;
	int is_read = 0;
	data = of_get_property(np, keystring, &len);
	if (!data) {
		pr_info("%s:%d, Unable to read %s",
			__func__, __LINE__, keystring);
		return -ENOMEM;
	}
	commands->cmds_buff = kzalloc(sizeof(char) * len, GFP_KERNEL);
	if (!commands->cmds_buff)
		return -ENOMEM;
	memcpy(commands->cmds_buff, data, len);
	commands->cmds_len = len;
	/* scan dcs commands */
	bp = commands->cmds_buff;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen >200)
			goto error2;
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		commands->num_of_cmds++;
		type = dchdr->dtype;
		if (type == DTYPE_GEN_READ ||
			type == DTYPE_GEN_READ1 ||
			type == DTYPE_GEN_READ2 ||
			type == DTYPE_DCS_READ)	{
			/* Read command :last byte contain read size, read start */
			bp += 2;
			len -= 2;
			is_read = 1;
		}
	}
	if (len != 0) {
		pr_info("%s: dcs OFF command byte Error, len=%d", __func__, len);
		commands->cmds_len = 0;
		commands->num_of_cmds = 0;
		goto error2;
	}
	if (is_read) {
		/*
		Allocate an array which will store the number
		for bytes to read for each read command
		*/
		commands->read_size = kzalloc(sizeof(char) * \
		commands->num_of_cmds, GFP_KERNEL);
		if (!commands->read_size) {
			pr_err("%s:%d, Unable to read NV cmds",
				__func__, __LINE__);
			goto error2;
		}
		commands->read_startoffset = kzalloc(sizeof(char) * \
			commands->num_of_cmds, GFP_KERNEL);
		if (!commands->read_startoffset) {
			pr_err("%s:%d, Unable to read NV cmds",
				__func__, __LINE__);
			goto error1;
		}
	}
	commands->cmd_desc = kzalloc(commands->num_of_cmds
		* sizeof(struct dsi_cmd_desc),
		GFP_KERNEL);
	if (!commands->cmd_desc)
		goto error1;
	bp = commands->cmds_buff;
	len = commands->cmds_len;
	for (i = 0; i < commands->num_of_cmds; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		commands->cmd_desc[i].dchdr = *dchdr;
		commands->cmd_desc[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		if (is_read) {
			commands->read_size[i] = *bp++;
			commands->read_startoffset[i] = *bp++;
			len -= 2;
		}
	}
	return 0;
error1:
	kfree(commands->read_size);
error2:
	kfree(commands->cmds_buff);
	return -EINVAL;
}

static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32	tmp;
	int rc, i, len;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);
	static const char *on_cmds_state, *off_cmds_state;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	pinfo->xres = (!rc ? tmp : 640);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-pixel-packing", &tmp);
	tmp = (!rc ? tmp : 0);
	rc = mdss_panel_dt_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, tmp,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}

	pdest = of_get_property(np,
			"qcom,mdss-dsi-panel-destination", NULL);
	if (strlen(pdest) != 9) {
		pr_err("%s: Unknown pdest specified\n", __func__);
		return -EINVAL;
	}
	if (!strncmp(pdest, "display_1", 9))
		pinfo->pdest = DISPLAY_1;
	else if (!strncmp(pdest, "display_2", 9))
		pinfo->pdest = DISPLAY_2;
	else {
		pr_debug("%s: pdest not specified. Set Default\n",
							__func__);
		pinfo->pdest = DISPLAY_1;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	pinfo->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
					__func__, __LINE__);
			return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			rc = of_property_read_u32(np,
					"qcom,mdss-dsi-bl-pmic-bank-select", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, dsi lpg channel\n",
 						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_lpg_chan = tmp;
			tmp = of_get_named_gpio(np,
				"qcom,mdss-dsi-pwm-gpio", 0);
			ctrl_pdata->pwm_pmic_gpio = tmp;
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
 		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-traffic-mode", &tmp);
	pinfo->mipi.traffic_mode =
			(!rc ? tmp : DSI_NON_BURST_SYNCH_PULSE);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-v-sync-continue-lines", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-v-sync-rd-ptr-irq-line", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-color-order", &tmp);
	pinfo->mipi.rgb_swap = (!rc ? tmp : DSI_RGB_SWAP_RGB);

	rc = of_property_read_u32(np, "qcom,mdss-force-clk-lane-hs", &tmp);
	pinfo->mipi.force_clk_lane_hs = (!rc ? tmp : 0);

	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-lane-map", &tmp);
	pinfo->mipi.dlane_swap = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-mdp-trigger", &tmp);
	pinfo->mipi.mdp_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (pinfo->mipi.mdp_trigger > 6) {
		pr_err("%s:%d, Invalid mdp trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		pinfo->mipi.mdp_trigger =
					DSI_CMD_TRIGGER_SW;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-dma-trigger", &tmp);
	pinfo->mipi.dma_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (pinfo->mipi.dma_trigger > 6) {
		pr_err("%s:%d, Invalid dma trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		pinfo->mipi.dma_trigger =
					DSI_CMD_TRIGGER_SW;
	}
	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", &tmp);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-frame-rate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clock-rate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	on_cmds_state = of_get_property(np,
		"qcom,on-cmds-dsi-state", NULL);
	if (!strncmp(on_cmds_state, "DSI_LP_MODE", 11)) {
		ctrl_pdata->dsi_on_state = DSI_LP_MODE;
	} else if (!strncmp(on_cmds_state, "DSI_HS_MODE", 11)) {
		ctrl_pdata->dsi_on_state = DSI_HS_MODE;
	} else {
		pr_debug("%s: ON cmds state not specified. Set Default\n",
			__func__);
		ctrl_pdata->dsi_on_state = DSI_LP_MODE;
	}

	off_cmds_state = of_get_property(np, "qcom,off-cmds-dsi-state", NULL);
	if (!strncmp(off_cmds_state, "DSI_LP_MODE", 11)) {
		ctrl_pdata->dsi_off_state = DSI_LP_MODE;
	} else if (!strncmp(off_cmds_state, "DSI_HS_MODE", 11)) {
		ctrl_pdata->dsi_off_state = DSI_HS_MODE;
	} else {		pr_debug("%s: ON cmds state not specified. Set Default\n",							__func__);		ctrl_pdata->dsi_off_state = DSI_LP_MODE;	}

	mdss_dsi_parse_fbc_params(np, pinfo);

	mdss_samsung_parse_panel_cmd(np, &display_qcom_on_cmds,
				"qcom,mdss-dsi-on-command");
	mdss_samsung_parse_panel_cmd(np, &display_qcom_off_cmds,
				"qcom,mdss-dsi-off-command");

	mdss_samsung_parse_panel_cmd(np, &display_unblank_cmd,
				"qcom,panel-display-unblank-cmds");
	mdss_samsung_parse_panel_cmd(np, &display_blank_cmd,
				"qcom,panel-display-blank-cmds");

	mdss_samsung_parse_panel_cmd(np, &nv_mtp_read_cmds,
				"samsung,panel-nv-mtp-read-cmds");


	mdss_samsung_parse_panel_cmd(np, &nv_enable_cmds,
				"samsung,panel-nv-read-enable-cmds");
	mdss_samsung_parse_panel_cmd(np, &nv_disable_cmds,
				"samsung,panel-nv-read-disable-cmds");
	mdss_samsung_parse_panel_cmd(np, &mtp_read_cmd,
				"samsung,panel-mtp-read-cmds");
	mdss_samsung_parse_panel_cmd(np, &manufacture_id_cmds,
				"samsung,panel-manufacture-id-read-cmds");

	mdss_samsung_parse_panel_cmd(np, &manufature_cmd,
				"samsung,panel-manufacture-read-cmds");
	mdss_samsung_parse_panel_cmd(np, &manufature_cmd2,
				"samsung,panel-manufacture-read2-cmds");
	mdss_samsung_parse_panel_cmd(np, &acl_off_cmd,
				"samsung,panel-acl-off-cmds");
	mdss_samsung_parse_panel_cmd(np, &elvss_lowtemp_cmds_list,
				"samsung,panel-elvss-lowtemp-cmds-list");

	mdss_samsung_parse_panel_cmd(np, &gamma_cmds_list,
				"samsung,panel-gamma-cmds-list");

	mdss_samsung_parse_panel_cmd(np, &gamma_update_cmds,
				"samsung,panel-gamma-update-cmd");

	mdss_samsung_parse_panel_cmd(np, &acl_cmds_list,
				"samsung,panel-acl-cmds-list");
	mdss_samsung_parse_panel_cmd(np, &elvss_cmds_list,
				"samsung,panel-elvss-cmds-list");
	mdss_samsung_parse_panel_cmd(np, &aid_cmds_list_350,
				"samsung,panel-aid-cmds-list-350");
	mdss_samsung_parse_panel_cmd(np, &aclcont_cmds_list,
				"samsung,panel-aclcont-cmds-list");
	mdss_samsung_parse_panel_cmd(np, &smart_acl_elvss_cmds_list,
				"samsung,panel-smart-acl-elvss-cmds-list");
	mdss_samsung_parse_panel_cmd(np, &mps_control_cmds,
				"samsung,panel-mps-control-cmds");

	/* Process the mapping tables */
	mdss_samsung_parse_panel_table(np, &aid_map_table,
				"samsung,panel-aid-map-table");
	mdss_samsung_parse_panel_table(np, &acl_map_table,
				"samsung,panel-acl-map-table");
	mdss_samsung_parse_panel_table(np, &elvss_map_table,
				"samsung,panel-elvss-map-table");
	mdss_samsung_parse_panel_table(np, &smart_acl_elvss_map_table,
				"samsung,panel-smart-acl-elvss-map-table");

	/* Process the lux value table */
	mdss_samsung_parse_candella_lux_mapping_table(np, &candela_map_table_350,
		"samsung,panel-candella-mapping-table-350");

#if defined(HBM_RE)
	mdss_samsung_parse_panel_cmd(np, &nv_hbm_read_cmds,
				"samsung,panel-nv-hbm-read-cmds");
	mdss_samsung_parse_panel_cmd(np, &nv_hbm_read_cmds2,
				"samsung,panel-nv-hbm-read2-cmds");
	mdss_samsung_parse_panel_cmd(np, &nv_hbm_read_cmds3,
				"samsung,panel-nv-hbm-read3-cmds");
	mdss_samsung_parse_panel_cmd(np, &hbm_read_cmd,
				"samsung,panel-hbm-read-cmds");
	mdss_samsung_parse_panel_cmd(np, &hbm_control_cmds,
				"samsung,panel-gamma-hbm-cmds-list");
#endif
	return 0;
error:
	return -EINVAL;
}
static struct lcd_ops mdss_sharp_disp_props = {
	.get_power = NULL,
	.set_power = NULL,

};
#if defined(CONFIG_LCD_CLASS_DEVICE)
static ssize_t mdss_disp_get_power(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	pr_info("mipi_samsung_disp_get_power(0)\n");
	return 0;
}
static ssize_t mdss_disp_set_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int power;
	if (sscanf(buf, "%u", &power) != 1)
		return -EINVAL;
	pr_info("mipi_samsung_disp_set_power:%d\n",power);
	return size;
}
static DEVICE_ATTR(lcd_power, S_IRUGO | S_IWUSR | S_IWGRP,
			mdss_disp_get_power,
			mdss_disp_set_power);

static ssize_t mipi_disp_siop_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rc;
	rc = snprintf((char *)buf, sizeof(buf), "%d\n", msd.dstat.siop_status);
	pr_info("siop status: %d\n", *buf);
	return rc;
}
static ssize_t mipi_disp_siop_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd = msd.mfd;
	int siop_set;

	siop_set = msd.dstat.siop_status;
	if (sysfs_streq(buf, "1"))
		siop_set = true;
	else if (sysfs_streq(buf, "0"))
		siop_set = false;
	else
		pr_info("%s: Invalid argument!!", __func__);
	if (mfd->panel_power_on) {
		if (siop_set && !(msd.dstat.acl_on||msd.dstat.siop_status)) {
			msd.dstat.siop_status = true;
			mipi_samsung_disp_send_cmd(PANEL_BRIGHT_CTRL, true);
			pr_info("%s: acl on  : acl %d, siop %d", __func__,
				msd.dstat.acl_on, msd.dstat.siop_status);
		} else if (!siop_set && !msd.dstat.acl_on && msd.dstat.siop_status) {
			mutex_lock(&msd.lock);
			msd.dstat.siop_status = false;
			msd.dstat.curr_acl_idx = -1;
			if (msd.dstat.auto_brightness == 6)
				pr_info("%s: HBM mode No ACL off!!", __func__);
			mutex_unlock(&msd.lock);
			pr_info("%s: acl off : acl %d, siop %d", __func__,
			msd.dstat.acl_on, msd.dstat.siop_status);
		} else {
			msd.dstat.siop_status = siop_set;
			pr_info("%s: skip but siop update!! acl %d, siop %d", __func__,
				msd.dstat.acl_on, msd.dstat.siop_status);
		}
	} else {
		msd.dstat.siop_status = siop_set;
		pr_info("%s: panel is off state. updating state value.\n",
			__func__);
	}
	return size;
}

static DEVICE_ATTR(siop_enable, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_disp_siop_show,
			mipi_disp_siop_store);

static ssize_t mdss_sharp_auto_brightness_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf(buf, sizeof(buf), "%d\n",
					msd.dstat.auto_brightness);
	pr_info("%s : auto_brightness : %d\n", __func__, msd.dstat.auto_brightness);

	return rc;
}

static ssize_t mdss_sharp_auto_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	static unsigned char prev_auto_brightness;
	struct mdss_panel_data *pdata = msd.pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return size;
	}
#endif

	if (sysfs_streq(buf, "0"))
		msd.dstat.auto_brightness = 0;
	else if (sysfs_streq(buf, "1"))
		msd.dstat.auto_brightness = 1;
	else if (sysfs_streq(buf, "2"))
		msd.dstat.auto_brightness = 2;
	else if (sysfs_streq(buf, "3"))
		msd.dstat.auto_brightness = 3;
	else if (sysfs_streq(buf, "4"))
		msd.dstat.auto_brightness = 4;
	else if (sysfs_streq(buf, "5"))
		msd.dstat.auto_brightness = 5;
	else if (sysfs_streq(buf, "6"))
		msd.dstat.auto_brightness = 6;
	else
		pr_info("%s: Invalid argument!!", __func__);

	if(prev_auto_brightness == msd.dstat.auto_brightness)
		return size;

	mdelay(1);

	if (msd.mfd == NULL) {
		pr_err("%s: mfd not initialized\n", __func__);
		return size;
	}

	if (msd.mfd->panel_power_on == false) {
		pr_err("%s: panel power off no bl ctrl\n", __func__);
		return size;
	}

	if(pdata == NULL){
		pr_err("%s: pdata not available... skipping update\n", __func__);
		return size;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	prev_auto_brightness = msd.dstat.auto_brightness;
	if (msd.mfd->resume_state == MIPI_RESUME_STATE && first_boot == 1) {
		mipi_samsung_disp_send_cmd(PANEL_BRIGHT_CTRL, true);
		pr_info(" %s %d %d\n", __func__, msd.dstat.auto_brightness, msd.dstat.siop_status);
	}
	else
		pr_info(" %s : panel is off state \n", __func__);
	return size;
}

static DEVICE_ATTR(auto_brightness, S_IRUGO | S_IWUSR | S_IWGRP,
			mdss_sharp_auto_brightness_show,
			mdss_sharp_auto_brightness_store);

static ssize_t mdss_disp_acl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rc;
	rc = snprintf((char *)buf, sizeof(buf), "%d\n", msd.dstat.acl_on);
	pr_info("acl status: %d\n", *buf);
	return rc;
}
static ssize_t mdss_disp_acl_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd = msd.mfd;
	int acl_set;
	acl_set = msd.dstat.acl_on;
	if (sysfs_streq(buf, "1"))
		acl_set = true;
	else if (sysfs_streq(buf, "0"))
		acl_set = false;
	else
		pr_info("%s: Invalid argument!!", __func__);
	if (mfd->panel_power_on) {
		if (acl_set && !(msd.dstat.acl_on||msd.dstat.siop_status)) {
			msd.dstat.acl_on = true;
			pr_info("%s: acl on  : acl %d, siop %d", __func__,
				msd.dstat.acl_on, msd.dstat.siop_status);
			mipi_samsung_disp_send_cmd(PANEL_BRIGHT_CTRL, true);
		} else if (!acl_set && msd.dstat.acl_on && !msd.dstat.siop_status) {
			msd.dstat.acl_on = false;
			msd.dstat.curr_acl_idx = -1;
			pr_info("%s: acl off : acl %d, siop %d", __func__,
			msd.dstat.acl_on, msd.dstat.siop_status);
			if(msd.dstat.auto_brightness == 6)
				pr_info("%s: HBM mode No ACL off!!", __func__);
			else
				mipi_samsung_disp_send_cmd(PANEL_ACL_OFF, true);

			mipi_samsung_disp_send_cmd(PANEL_BRIGHT_CTRL, true);
		} else {
			msd.dstat.acl_on = acl_set;
			pr_info("%s: skip but acl update!! acl %d, siop %d", __func__,
			msd.dstat.acl_on, msd.dstat.siop_status);
		}
	} else {
		pr_info("%s: panel is off state. updating state value.\n",
		__func__);
		msd.dstat.acl_on = acl_set;
	}
	return size;
}
static DEVICE_ATTR(power_reduce, S_IRUGO | S_IWUSR | S_IWGRP,
		mdss_disp_acl_show,
		mdss_disp_acl_store);

#if defined(TEMPERATURE_ELVSS) || defined(TEMPERATURE_ELVSS_S6E3FA0)
static ssize_t mdss_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;	rc = snprintf((char *)buf, 40,"-20, -19, 0, 1, 30, 40\n");
	pr_info("%s msd.mpd->temperature : %d msd.mpd->temperature_value : 0x%x", __func__,
		msd.dstat.temperature, msd.dstat.temperature_value);
	return rc;
}
static ssize_t mdss_temperature_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int temp;
	sscanf(buf, "%d" , &msd.dstat.temperature);
	temp = msd.dstat.temperature;
	if (temp > 0)
		msd.dstat.temperature_value = (char)temp;
	else {
		temp *= -1;
		msd.dstat.temperature_value = (char)temp;
		msd.dstat.temperature_value |=0x80;
	}
	msd.dstat.temper_need_update = 1;
	if (msd.mfd->resume_state == MIPI_RESUME_STATE) {
		mipi_samsung_disp_send_cmd(PANEL_BRIGHT_CTRL, true);
		pr_info("mipi_samsung_temperature_store %d\n", msd.dstat.bright_level);
		pr_info("%s msd.dstat.temperature : %d msd.dstat.temperature_value : 0x%x", __func__,
			msd.dstat.temperature, msd.dstat.temperature_value);
	} else {
		pr_info("%s: skip but temperature update!! temperature %d, temperature_value %d", __func__,
			msd.dstat.temperature, msd.dstat.temperature_value);
	}
	return size;
}
static DEVICE_ATTR(temperature, S_IRUGO | S_IWUSR | S_IWGRP,
		mdss_temperature_show,
		mdss_temperature_store);
#endif

static ssize_t mdss_disp_lcdtype_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char temp[20];

	snprintf(temp, 20, "MAG_AMS549BU01");
	strncat(buf, temp, 20);
	return strnlen(buf, 20);
}
static DEVICE_ATTR(lcd_type, S_IRUGO, mdss_disp_lcdtype_show, NULL);
unsigned int mdss_dsi_show_cabc(void )
{
	return msd.dstat.siop_status;
}
#endif

#if defined(CONFIG_LCD_CLASS_DEVICE)
static struct attribute *panel_sysfs_attributes[] = {
	&dev_attr_lcd_power.attr,
	&dev_attr_lcd_type.attr,
	&dev_attr_power_reduce.attr,
	&dev_attr_siop_enable.attr,
#if defined(TEMPERATURE_ELVSS) || defined(TEMPERATURE_ELVSS_S6E3FA0)
	&dev_attr_temperature.attr,
#endif
NULL
};
#endif
static const struct attribute_group panel_sysfs_group = {
	.attrs = panel_sysfs_attributes,
};
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
static irqreturn_t err_fg_irq_handler(int irq, void *handle)
{
	pr_info("%s : handler start", __func__);
	disable_irq_nosync(err_fg_gpio);
	schedule_work(&err_fg_work);
	pr_info("%s : handler end", __func__);

	return IRQ_HANDLED;
}
static void err_fg_work_func(struct work_struct *work)
{
	int bl_backup;
	struct mdss_panel_data *pdata = msd.pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if(msd.mfd == NULL){
		pr_err("%s: mfd not initialized Skip ESD recovery\n", __func__);
		return;
	}
	if(pdata == NULL){
		pr_err("%s: pdata not available... skipping update\n", __func__);
		return;
	}
	bl_backup = msd.mfd->bl_level;
	if( msd.mfd->panel_power_on == false){
		pr_err("%s: Display off Skip ESD recovery\n", __func__);
		return;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	pr_info("%s : start", __func__);
	err_fg_working = 1;
	mdss_dsi_panel_off(pdata);
	mdss_dsi_panel_reset(pdata, 1);
	mdss_dsi_panel_on(pdata);
//	msd.mfd->fbi->fbops->fb_blank(FB_BLANK_POWERDOWN, msd.mfd->fbi);
//	msd.mfd->fbi->fbops->fb_blank(FB_BLANK_UNBLANK, msd.mfd->fbi);
	esd_count++;
	err_fg_working = 0;
	msd.mfd->bl_level = bl_backup;
	mdss_dsi_panel_bl_ctrl(pdata,msd.mfd->bl_level);
	pr_info("%s : end", __func__);
	return;
}
#ifdef ESD_DEBUG
static ssize_t mipi_samsung_esd_check_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, 20, "esd count:%d \n",esd_count);

	return rc;
}
static ssize_t mipi_samsung_esd_check_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(msd.msm_pdev);

	err_fg_irq_handler(0, mfd);
	return 1;
}

static DEVICE_ATTR(esd_check, S_IRUGO , mipi_samsung_esd_check_show,\
			 mipi_samsung_esd_check_store);
#endif
#endif
#ifdef DDI_VIDEO_ENHANCE_TUNING
#define MAX_FILE_NAME 128
#define TUNING_FILE_PATH "/sdcard/"
#define TUNE_FIRST_SIZE 5
#define TUNE_SECOND_SIZE 92
static char tuning_file[MAX_FILE_NAME];
static char mdni_tuning1[TUNE_FIRST_SIZE];
static char mdni_tuning2[TUNE_SECOND_SIZE];
static struct dsi_cmd_desc mdni_tune_cmd[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mdni_tuning2)}, mdni_tuning2},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(mdni_tuning1)}, mdni_tuning1},
};
static char char_to_dec(char data1, char data2)
{
	char dec;

	dec = 0;

	if (data1 >= 'a') {
		data1 -= 'a';
		data1 += 10;
	} else if (data1 >= 'A') {
		data1 -= 'A';
		data1 += 10;
	} else
		data1 -= '0';

	dec = data1 << 4;

	if (data2 >= 'a') {
		data2 -= 'a';
		data2 += 10;
	} else if (data2 >= 'A') {
		data2 -= 'A';
		data2 += 10;
	} else
		data2 -= '0';

	dec |= data2;

	return dec;
}
static void sending_tune_cmd(char *src, int len)
{
	int data_pos;
	int cmd_step;
	int cmd_pos;
	struct mdss_panel_data *pdata = msd.pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	cmd_step = 0;
	cmd_pos = 0;
	for (data_pos = 0; data_pos < len;) {
		if (*(src + data_pos) == '0') {
			if (*(src + data_pos + 1) == 'x') {
				if (!cmd_step) {
					mdni_tuning1[cmd_pos] =
					char_to_dec(*(src + data_pos + 2),
							*(src + data_pos + 3));
				} else {
					mdni_tuning2[cmd_pos] =
					char_to_dec(*(src + data_pos + 2),
							*(src + data_pos + 3));
				}
				data_pos += 3;
				cmd_pos++;
				if (cmd_pos == TUNE_FIRST_SIZE && !cmd_step) {
					cmd_pos = 0;
					cmd_step = 1;
				}
			} else
				data_pos++;
		} else {
			data_pos++;
		}
	}
	printk(KERN_INFO "\n");
	for (data_pos = 0; data_pos < TUNE_FIRST_SIZE ; data_pos++)
		printk(KERN_INFO "0x%x ", mdni_tuning1[data_pos]);
	printk(KERN_INFO "\n");
	for (data_pos = 0; data_pos < TUNE_SECOND_SIZE ; data_pos++)
		printk(KERN_INFO"0x%x ", mdni_tuning2[data_pos]);
	printk(KERN_INFO "\n");
	mutex_lock(&msd.lock);
	mdss_dsi_cmds_send(ctrl_pdata, mdni_tune_cmd, ARRAY_SIZE(mdni_tune_cmd),0);
	mutex_unlock(&msd.lock);

}
static void load_tuning_file(char *filename)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret;
	mm_segment_t fs;

	pr_info("%s called loading file name : [%s]\n", __func__,
	       filename);

	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		printk(KERN_ERR "%s File open failed\n", __func__);
		return;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	pr_info("%s Loading File Size : %ld(bytes)", __func__, l);

	dp = kmalloc(l + 10, GFP_KERNEL);
	if (dp == NULL) {
		pr_info("Can't not alloc memory for tuning file load\n");
		filp_close(filp, current->files);
		return;
	}
	pos = 0;
	memset(dp, 0, l);

	pr_info("%s before vfs_read()\n", __func__);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	pr_info("%s after vfs_read()\n", __func__);

	if (ret != l) {
		pr_info("vfs_read() filed ret : %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	sending_tune_cmd(dp, l);

	kfree(dp);
}

static ssize_t tuning_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = snprintf(buf, MAX_FILE_NAME, "tuned file name : %s\n",
								tuning_file);

	return ret;
}

static ssize_t tuning_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t size)
{
	char *pt;
	memset(tuning_file, 0, sizeof(tuning_file));
	snprintf(tuning_file, MAX_FILE_NAME, "%s%s", TUNING_FILE_PATH, buf);

	pt = tuning_file;
	while (*pt) {
		if (*pt == '\r' || *pt == '\n') {
			*pt = 0;
			break;
		}
		pt++;
	}

	pr_info("%s:%s\n", __func__, tuning_file);

	load_tuning_file(tuning_file);

	return size;
}
static DEVICE_ATTR(tuning, S_IRUGO | S_IWUSR | S_IWGRP,
			tuning_show,
			tuning_store);
#endif
#if defined(CONFIG_LCD_CONNECTION_CHECK)
int is_lcd_attached(void)
{
	return lcd_connected_status;
}
EXPORT_SYMBOL(is_lcd_attached);

static int __init lcd_attached_status(char *state)
{
	/*
	*	1 is lcd attached
	*	0 is lcd detached
	*/

	if (strncmp(state, "1", 1) == 0)
		lcd_connected_status = 1;
	else
		lcd_connected_status = 0;

	pr_info("%s %s", __func__, lcd_connected_status == 1 ?
				"lcd_attached" : "lcd_detached");
	lcd_connected_status = 1;
	return 1;
}
__setup("lcd_attached=", lcd_attached_status);

static int __init detect_lcd_panel_vendor(char* read_id)
{
	int lcd_id = simple_strtol(read_id, NULL, 16);
	pr_info("%s: detected panel vendor --> [0x%x]\n", __func__, lcd_id);
	return 1;
}
__setup("lcd_id=0x", detect_lcd_panel_vendor);
#endif
static int is_panel_supported(const char *panel_name)
{
	int i = 0;

	if (panel_name == NULL)
		return -EINVAL;
	while (panel_supp_cdp[i].name != NULL) {
		if (!strcmp(panel_name, panel_supp_cdp[i].name))
			break;
		i++;
	}
	if ( i < ARRAY_SIZE(panel_supp_cdp)) {
		memcpy(msd.panel_name, panel_name, MAX_PANEL_NAME_SIZE);
		msd.panel = panel_supp_cdp[i].panel_code;
		return 0;
	}
	return -EINVAL;
}

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	bool cont_splash_enabled;
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	int disp_esd_gpio;
#endif
#if defined(CONFIG_LCD_CLASS_DEVICE)
	struct lcd_device *lcd_device;
#if defined(CONFIG_BACKLIGHT_CLASS_DEVICE)
	struct backlight_device *bd = NULL;
#endif
#endif
#if defined(CONFIG_LCD_CLASS_DEVICE)
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	np = of_parse_phandle(node,
			"qcom,mdss-dsi-panel-controller", 0);
	if (!np) {
		pr_err("%s: Dsi controller node not initialized\n", __func__);
		return -EPROBE_DEFER;
	}

	pdev = of_find_device_by_node(np);
#endif

#if defined(CONFIG_LCD_CONNECTION_CHECK)
	printk("%s: LCD attached status: %d !\n",
				__func__, is_lcd_attached());
#endif
#ifdef DDI_VIDEO_ENHANCE_TUNING
	mutex_init(&msd.lock);
#endif
	mutex_init(&msd.lock);
	if (!node) {
		pr_err("%s: no panel node\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s:%d\n", __func__, __LINE__);
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name)
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	else
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);

	if (is_panel_supported(panel_name)) {
		LCD_DEBUG("Panel : %s is not supported:", panel_name);
		return -1;
	}

	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	if (cmd_cfg_cont_splash)
		cont_splash_enabled = of_property_read_bool(node,
				"qcom,cont-splash-enabled");
	else
		cont_splash_enabled = false;
	if (!cont_splash_enabled) {
		pr_info("%s:%d Continuous splash flag not found.\n",
				__func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
	} else {
		pr_info("%s:%d Continuous splash flag enabled.\n",
				__func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 1;
	}
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected.... Disabling Continous Splash!\n",__func__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
	}
#endif
	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->panel_reset = mdss_dsi_panel_reset;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->registered = mdss_dsi_panel_registered;

#if defined(CONFIG_LCD_CLASS_DEVICE)
	lcd_device = lcd_device_register("panel", &pdev->dev, NULL,
					&mdss_sharp_disp_props);

	if (IS_ERR(lcd_device)) {
		rc = PTR_ERR(lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return rc;
	}

	sysfs_remove_file(&lcd_device->dev.kobj,&dev_attr_lcd_power.attr);

	rc = sysfs_create_group(&lcd_device->dev.kobj, &panel_sysfs_group);
	if (rc) {
		pr_err("Failed to create panel sysfs group..\n");
		sysfs_remove_group(&lcd_device->dev.kobj, &panel_sysfs_group);
	}
#if defined(CONFIG_BACKLIGHT_CLASS_DEVICE)
	bd = backlight_device_register("panel", &lcd_device->dev,
			NULL, NULL, NULL);
	if (IS_ERR(bd)) {
		rc = PTR_ERR(bd);
		pr_info("backlight : failed to register device\n");
		return rc;
	}
	rc = sysfs_create_file(&bd->dev.kobj,
			&dev_attr_auto_brightness.attr);
	if (rc){
		pr_info("sysfs create fail - %s\n",
			dev_attr_auto_brightness.attr.name);
	}
#endif
#endif
#if defined(DDI_VIDEO_ENHANCE_TUNING)
	rc = sysfs_create_file(&lcd_device->dev.kobj,
			&dev_attr_tuning.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_tuning.attr.name);
	}
#endif
#if defined(CONFIG_MDNIE_VIDEO_ENHANCED)
		pr_info("[%s] CONFIG_MDNIE_VIDEO_ENHANCED ok ! initclass called!\n",__func__);
		init_mdnie_class();
#endif
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
#ifdef ESD_DEBUG
	rc = sysfs_create_file(&lcd_device->dev.kobj,
							&dev_attr_esd_check.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_esd_check.attr.name);
	}
#endif
	msd.msm_pdev = pdev;
	INIT_WORK(&err_fg_work, err_fg_work_func);
	disp_esd_gpio =of_get_named_gpio(node,"qcom,oled-esd-gpio", 0);
	err_fg_gpio = gpio_to_irq(disp_esd_gpio);
	rc = gpio_request(disp_esd_gpio, "err_fg");
	if (rc) {
		pr_err("request gpio GPIO_ESD failed, ret=%d\n",rc);
		gpio_free(disp_esd_gpio);
		return rc;
	}
	gpio_tlmm_config(GPIO_CFG(disp_esd_gpio,  0, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	rc = gpio_direction_input(disp_esd_gpio);
	if (unlikely(rc < 0)) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, disp_esd_gpio, rc);
	}
	rc = request_threaded_irq(err_fg_gpio, NULL, err_fg_irq_handler,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, "esd_detect", NULL);
	if (rc) {
		pr_err("%s : Failed to request_irq. :ret=%d", __func__, rc);
	}
	disable_irq(err_fg_gpio);
#endif
	return 0;
}
