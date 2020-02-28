/*
 * drivers/usb/otg/msm_otg_sec.c
 *
 * Copyright (c) 2013, Samsung Electronics
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

#include <linux/gpio.h>
#include <linux/host_notify.h>
#ifdef CONFIG_USB_SWITCH_FSA9485
#include <linux/i2c/fsa9485.h>
#endif
#ifdef CONFIG_MFD_MAX77693
#include <linux/mfd/max77693.h>
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "otg %s %d: " fmt, __func__, __LINE__

#define SM_WORK_TIMER_FREQ	(jiffies + msecs_to_jiffies(1000))

#if 0
#define MSM_OTG_LATE_INIT
#define MSM_OTG_DIRECT_VBUS
#endif

static void msm_otg_set_vbus_state(int online);
static void msm_hsusb_vbus_power(struct msm_otg *motg, bool on);
static int ulpi_write(struct usb_phy *phy, u32 val, u32 reg);
static int ulpi_read(struct usb_phy *phy, u32 reg);

#if defined(CONFIG_CHARGER_SMB358)
int sec_battery_otg_control(int enable)
{
	union power_supply_propval value;
	struct power_supply *psy;
	int current_cable_type;
	int ret = 0;

	pr_info("%s: enable(%d)\n", __func__, enable);

	psy = power_supply_get_by_name("battery");
	if (!psy) {
		pr_err("%s: ERROR! failed to get battery!\n", __func__);
		return -1;
	}

	if (enable)
		current_cable_type = POWER_SUPPLY_TYPE_OTG;
	else
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;

	value.intval = current_cable_type;
	ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);

	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	}
	return ret;
}

struct booster_data sec_booster = {
	.name = "sec_battery",
	.boost = sec_battery_otg_control,
};
#endif

static int msm_otg_sec_power(bool on)
{
	pr_info("msm_otg_sec_power: %d\n", on);
#ifdef CONFIG_MFD_MAX77693
	muic_otg_control(on);
#elif defined(CONFIG_CHARGER_SMB358)
        sec_battery_otg_control(on);
#else
	return 0;
#endif
        return 0;
}

#if !defined(CONFIG_CHARGER_SMB358)
static int msm_otg_set_id_state_pbatest(int id, struct host_notify_dev *ndev)
{
	struct msm_otg *motg = container_of(ndev, struct msm_otg, ndev);

#ifdef MSM_OTG_DIRECT_VBUS
	pr_info("[OTG] %s %d, id: %d\n", __func__, __LINE__, id);
	msm_hsusb_vbus_power(motg, id);
#else
	struct usb_phy *phy = &motg->phy;

	pr_info("[OTG] %s %d, id: %d\n", __func__, __LINE__, id);
	if (atomic_read(&motg->in_lpm))
		pm_runtime_resume(phy->dev);

	if (!id)
		set_bit(ID, &motg->inputs);
	else
		clear_bit(ID, &motg->inputs);

	schedule_work(&motg->sm_work);
#endif

#ifdef CONFIG_USB_SWITCH_FSA9485
	if (!id)
		fsa9485_otg_detach();
#endif
	return 0;
}
#endif

static void msm_otg_host_phy_tune(struct msm_otg *otg,
		u32 paramb, u32 paramc)
{
	ulpi_write(&otg->phy, paramb, 0x81);
	ulpi_write(&otg->phy, paramc, 0x82);

	pr_info("ULPI 0x%x: 0x%x: 0x%x: 0x%x\n",
			ulpi_read(&otg->phy, 0x80),
			ulpi_read(&otg->phy, 0x81),
			ulpi_read(&otg->phy, 0x82),
			ulpi_read(&otg->phy, 0x83));
	mdelay(100);
}

static int msm_otg_host_notify_set(struct msm_otg *motg, int state)
{
	pr_info("boost : %d\n", state);

	if (state)
		sec_otg_notify(HNOTIFY_OTG_POWER_ON);
	else
		sec_otg_notify(HNOTIFY_OTG_POWER_OFF);

	return state;
}

static void msm_otg_host_notify(struct msm_otg *motg, int on)
{
	pr_info("host_notify: %d, dock %d\n", on, motg->smartdock);

	if (on)
		msm_otg_host_phy_tune(motg, 0x33, 0x14);
}

static int msm_host_notify_init(struct device *dev, struct msm_otg *motg)
{
#if defined(CONFIG_CHARGER_SMB358)
	sec_otg_register_booster(&sec_booster);
#else
	sec_otg_set_booster(msm_otg_set_id_state_pbatest);
#endif
	return 0;
}

/*
 * Exported functions
 */

void sec_otg_set_dock_state(int enable)
{
	struct msm_otg *motg = the_msm_otg;
	struct usb_phy *phy = &motg->phy;

	if (enable) {
		pr_info("DOCK : attached\n");
		motg->smartdock = true;
		clear_bit(ID, &motg->inputs);

		if (atomic_read(&motg->in_lpm)) {
			pr_info("DOCK : in LPM\n");
			pm_runtime_resume(phy->dev);
		}

		if (test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_info("clear B_SESS_VLD\n");
			clear_bit(B_SESS_VLD, &motg->inputs);
		}

		schedule_work(&motg->sm_work);

	} else {
		pr_info("DOCK : detached\n");
		motg->smartdock = false;
		set_bit(ID, &motg->inputs);
	}

}
EXPORT_SYMBOL(sec_otg_set_dock_state);

void sec_otg_set_id_state(int id)
{
	struct msm_otg *motg = the_msm_otg;
	struct usb_phy *phy = &motg->phy;

	pr_info("msm_otg_set_id_state is called, ID =%d\n", id);

	if (id)
		set_bit(ID, &motg->inputs);
	else
		clear_bit(ID, &motg->inputs);

	if (atomic_read(&motg->in_lpm)) {
		pr_info("msm_otg_set_id_state : in LPM\n");
		pm_runtime_resume(phy->dev);
	}

	schedule_work(&motg->sm_work);
}
EXPORT_SYMBOL(sec_otg_set_id_state);

void msm_otg_set_smartdock_state(bool online)
{
	struct msm_otg *motg = the_msm_otg;

	if (online) {
		dev_info(motg->phy.dev, "SMARTDOCK : ID set\n");
		motg->smartdock = false;
		set_bit(ID, &motg->inputs);
	} else {
		dev_info(motg->phy.dev, "SMARTDOCK : ID clear\n");
		motg->smartdock = true;
		clear_bit(ID, &motg->inputs);
	}
	if (test_bit(B_SESS_VLD, &motg->inputs))
		clear_bit(B_SESS_VLD, &motg->inputs);
	if (atomic_read(&motg->pm_suspended))
		motg->sm_work_pending = true;
	else
		queue_work(system_nrt_wq, &motg->sm_work);
}
EXPORT_SYMBOL_GPL(msm_otg_set_smartdock_state);


int sec_handle_event(int enable)
{
	sec_otg_set_id_state(!enable);
	return 0;
}
EXPORT_SYMBOL(sec_handle_event);
