/*
 * Maxim MAX77620 Regulator driver
 *
 * Copyright (C) 2014 NVIDIA CORPORATION. All rights reserved.
 *
 * Author: Mallikarjun Kasoju <mkasoju@nvidia.com>
 * 	   Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77620.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>

#define max77620_rails(_name)	"max77620-"#_name

/* Power Mode */
#define MAX77620_POWER_MODE_NORMAL		3
#define MAX77620_POWER_MODE_LPM			2
#define MAX77620_POWER_MODE_GLPM		1
#define MAX77620_POWER_MODE_DISABLE		0

/* SD Slew Rate */
#define MAX77620_SD_SR_13_75			0
#define MAX77620_SD_SR_27_5			1
#define MAX77620_SD_SR_55			2
#define MAX77620_SD_SR_100			3

#define MAX77620_FPS_SRC_NUM			3

struct max77620_regulator_info {
	u8 type;
	u32 min_uV;
	u32 max_uV;
	u32 step_uV;
	u8 fps_addr;
	u8 volt_addr;
	u8 cfg_addr;
	u8 volt_mask;
	u8 power_mode_mask;
	u8 power_mode_shift;
	struct regulator_desc desc;
};

struct max77620_regulator_pdata {
	bool glpm_enable;
	bool en2_ctrl_sd0;
	bool sd_fsrade_disable;
	struct regulator_init_data *reg_idata;
	int fps_src;
	int fps_pd_period;
	int fps_pu_period;
};

struct max77620_regulator {
	struct device *dev;
	struct max77620_chip *max77620_chip;
	struct max77620_regulator_info *rinfo[MAX77620_NUM_REGS];
	struct max77620_regulator_pdata reg_pdata[MAX77620_NUM_REGS];
	struct regulator_dev *rdev[MAX77620_NUM_REGS];
	int enable_power_mode[MAX77620_NUM_REGS];
	int current_power_mode[MAX77620_NUM_REGS];
	int fps_src[MAX77620_NUM_REGS];
};

#define fps_src_name(fps_src)	\
	(fps_src == FPS_SRC_0 ? "FPS_SRC_0" :	\
	fps_src == FPS_SRC_1 ? "FPS_SRC_1" :	\
	fps_src == FPS_SRC_2 ? "FPS_SRC_2" : "FPS_SRC_NONE")

static int max77620_regulator_set_fps_src(struct max77620_regulator *reg,
		       int fps_src, int id)
{
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	struct device *parent = reg->max77620_chip->dev;
	u8 val;
	int ret;

	switch (fps_src) {
	case FPS_SRC_0:
	case FPS_SRC_1:
	case FPS_SRC_2:
	case FPS_SRC_NONE:
		break;

	case FPS_SRC_DEF:
		ret = max77620_reg_read(parent, MAX77620_PWR_SLAVE,
			rinfo->fps_addr, &val);
		if (ret < 0) {
			dev_err(reg->dev, "Reg 0x%02x read failed %d\n",
				rinfo->fps_addr, ret);
			return ret;
		}
		ret = (val & MAX77620_FPS_SRC_MASK) >> MAX77620_FPS_SRC_SHIFT;
		reg->fps_src[id] = ret;
		return 0;

	default:
		dev_err(reg->dev, "Invalid FPS %d for regulator %d\n",
			fps_src, id);
		return -EINVAL;
	}
	ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
			rinfo->fps_addr, MAX77620_FPS_SRC_MASK,
			fps_src << MAX77620_FPS_SRC_SHIFT);
	if (ret < 0) {
		dev_err(reg->dev, "Reg 0x%02x update failed %d\n",
			rinfo->fps_addr, ret);
		return ret;
	}
	reg->fps_src[id] = fps_src;
	return 0;
}

static int max77620_regulator_set_fps_slots(struct max77620_regulator *reg,
			int id)
{
	struct max77620_regulator_pdata *rpdata = &reg->reg_pdata[id];
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	struct device *parent = reg->max77620_chip->dev;
	unsigned int val = 0;
	unsigned int mask = 0;
	int ret = 0;

	/* FPS power up period setting */
	if (rpdata->fps_pu_period >= 0) {
		val |= (rpdata->fps_pu_period << MAX77620_FPS_PU_PERIOD_SHIFT);
		mask |= MAX77620_FPS_PU_PERIOD_MASK;
	}

	/* FPS power down period setting */
	if (rpdata->fps_pd_period >= 0) {
		val |= (rpdata->fps_pd_period << MAX77620_FPS_PD_PERIOD_SHIFT);
		mask |= MAX77620_FPS_PD_PERIOD_MASK;
	}

	if (mask) {
		ret =  max77620_reg_update(parent, MAX77620_PWR_SLAVE,
				rinfo->fps_addr, mask, val);
		if (ret < 0) {
			dev_err(reg->dev, "Reg 0x%02x update faild, %d\n",
				rinfo->fps_addr, ret);
			return ret;
		}
	}
	return ret;
}

static int max77620_regulator_set_power_mode(struct max77620_regulator *reg,
	int power_mode, int id)
{
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	int ret;
	struct device *parent = reg->max77620_chip->dev;
	u8 mask = rinfo->power_mode_mask;
	u8 shift = rinfo->power_mode_shift;
	u8 addr = rinfo->volt_addr;

	if (rinfo->type == MAX77620_REGULATOR_TYPE_SD)
		addr = rinfo->cfg_addr;

	ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
			addr, mask, power_mode << shift);
	if (ret < 0) {
		dev_err(reg->dev, "Regulator mode set failed. ret %d\n", ret);
		return ret;
	}
	reg->current_power_mode[id] = power_mode;
	return ret;
}

static int max77620_regulator_get_power_mode(struct max77620_regulator *reg,
			int id)
{
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	struct device *parent = reg->max77620_chip->dev;
	u8 val;
	u8 mask = rinfo->power_mode_mask;
	u8 shift = rinfo->power_mode_shift;
	u8 addr = rinfo->volt_addr;
	int ret;

	if (rinfo->type == MAX77620_REGULATOR_TYPE_SD)
		addr = rinfo->cfg_addr;

	ret = max77620_reg_read(parent, MAX77620_PWR_SLAVE, addr, &val);
	if (ret < 0) {
		dev_err(reg->dev, "Reg 0x%02x read failed %d\n",
			addr, ret);
		return ret;
	}

	return (val & mask) >> shift;
}

static int max77620_regulator_enable(struct regulator_dev *rdev)
{
	struct max77620_regulator *reg = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77620_regulator_pdata *rpdata = &reg->reg_pdata[id];
	int ret;

	if (reg->fps_src[id] != FPS_SRC_NONE)
		return 0;

	if ((id == MAX77620_REGULATOR_ID_SD0) && rpdata->en2_ctrl_sd0)
		return 0;

	ret = max77620_regulator_set_power_mode(reg, reg->enable_power_mode[id],
					id);
	if (ret < 0) {
		dev_err(reg->dev, "Regulator %d power mode config failed: %d\n",
				id, ret);
		return ret;
	}
	return 0;
}

static int max77620_regulator_disable(struct regulator_dev *rdev)
{
	struct max77620_regulator *reg = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77620_regulator_pdata *rpdata = &reg->reg_pdata[id];
	int ret;

	if (reg->fps_src[id] != FPS_SRC_NONE)
		return 0;

	if ((id == MAX77620_REGULATOR_ID_SD0) && rpdata->en2_ctrl_sd0)
		return 0;

	ret =  max77620_regulator_set_power_mode(reg,
			MAX77620_POWER_MODE_DISABLE, id);
	if (ret < 0) {
		dev_err(reg->dev, "Regulator %d power mode config failed: %d\n",
				id, ret);
		return ret;
	}
	return 0;
}

static int max77620_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct max77620_regulator *reg = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77620_regulator_pdata *rpdata = &reg->reg_pdata[id];
	int ret = 1;

	if (reg->fps_src[id] != FPS_SRC_NONE)
		return 1;

	if ((id == MAX77620_REGULATOR_ID_SD0) && rpdata->en2_ctrl_sd0)
		return 1;

	ret = max77620_regulator_get_power_mode(reg, id);
	if (ret < 0) {
		dev_err(reg->dev, "Regulator %d power mode read failed: %d\n",
				id, ret);
		return ret;
	}
	if (ret != MAX77620_POWER_MODE_DISABLE)
		return 1;

	return 0;
}

static int max77620_regulator_set_mode(struct regulator_dev *rdev,
				       unsigned int mode)
{
	struct max77620_regulator *reg = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	struct max77620_regulator_pdata *rpdata = &reg->reg_pdata[id];
	struct device *parent = reg->max77620_chip->dev;
	int power_mode;
	int ret;
	bool fpwm = false;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		fpwm = true;
		power_mode = MAX77620_POWER_MODE_NORMAL;
		break;

	case REGULATOR_MODE_NORMAL:
		power_mode = MAX77620_POWER_MODE_NORMAL;
		break;

	case REGULATOR_MODE_IDLE:
	case REGULATOR_MODE_STANDBY:
		if (rpdata->glpm_enable)
			power_mode = MAX77620_POWER_MODE_GLPM;
		else
			power_mode = MAX77620_POWER_MODE_LPM;
		break;

	default:
		dev_err(reg->dev, "The regulator id %d mode %d not supported\n",
			id, mode);
		return -EINVAL;
	}

	if (rinfo->type != MAX77620_REGULATOR_TYPE_SD)
		goto skip_fpwm;

	if (fpwm)
		ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, MAX77620_SD_FPWM_MASK,
				MAX77620_SD_FPWM_MASK);
	else
		ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, MAX77620_SD_FPWM_MASK, 0);
	if (ret < 0) {
		dev_err(reg->dev, "Reg 0x%02x update failed: %d\n",
				rinfo->cfg_addr, ret);
		return ret;
	}

skip_fpwm:
	ret =  max77620_regulator_set_power_mode(reg, power_mode, id);
	if (ret < 0) {
		dev_err(reg->dev, "Power mode of regualtor %d failed %d\n",
				id, ret);
		return ret;
	}
	reg->enable_power_mode[id] = power_mode;
	return 0;
}

static unsigned int max77620_regulator_get_mode(struct regulator_dev *rdev)
{
	struct max77620_regulator *reg = rdev_get_drvdata(rdev);
	struct device *parent = reg->max77620_chip->dev;
	int id = rdev_get_id(rdev);
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	int fpwm = 0;
	int ret;
	int pm_mode, reg_mode;
	u8 val;

	ret = max77620_regulator_get_power_mode(reg, id);
	if (ret < 0)
		return 0;
	pm_mode = ret;

	if (rinfo->type == MAX77620_REGULATOR_TYPE_SD) {
		ret = max77620_reg_read(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, &val);
		if (ret < 0) {
			dev_err(reg->dev, "Reg 0x%02x read failed: %d\n",
				rinfo->cfg_addr, ret);
			return ret;
		}
		fpwm = !!(val & MAX77620_SD_FPWM_MASK);
	}

	switch (pm_mode) {
	case MAX77620_POWER_MODE_NORMAL:
	case MAX77620_POWER_MODE_DISABLE:
		if (fpwm)
			reg_mode = REGULATOR_MODE_FAST;
		else
			reg_mode = REGULATOR_MODE_NORMAL;
		break;
	case MAX77620_POWER_MODE_LPM:
	case MAX77620_POWER_MODE_GLPM:
		reg_mode = REGULATOR_MODE_IDLE;
		break;
	default:
		return 0;
	}
	return reg_mode;
}

static int max77620_regulator_set_ramp_delay(struct regulator_dev *rdev,
				       int ramp_delay)
{
	struct max77620_regulator *reg = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	struct device *parent = reg->max77620_chip->dev;
	int ret, val;
	int retval;

	if (rinfo->type == MAX77620_REGULATOR_TYPE_SD) {
		if (ramp_delay <= 13750) {
			val = 0;
			retval = 13750;
		} else if (ramp_delay <= 27500) {
			val = 1;
			retval = 27500;
		} else if (ramp_delay <= 55000) {
			val = 2;
			retval = 55000;
		} else {
			val = 3;
			retval = 100000;
		}
		ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, MAX77620_SD_SR_MASK,
				val << MAX77620_SD_SR_SHIFT);
		if (ret < 0) {
			dev_err(reg->dev, "Reg 0x%02x update failed: %d\n",
					rinfo->cfg_addr, ret);
			return ret;
		}
	} else {
		if (ramp_delay <= 5000) {
			val = 1;
			retval = 5000;
		} else {
			val = 0;
			retval = 100000;
		}
		ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, MAX77620_LDO_SLEW_RATE_MASK,
				val);
		if (ret < 0) {
			dev_err(reg->dev, "Reg 0x%02x update failed: %d\n",
					rinfo->cfg_addr, ret);
			return ret;
		}
	}
	return retval;
}

static struct regulator_ops max77620_regulator_ops = {
	.is_enabled = max77620_regulator_is_enabled,
	.enable = max77620_regulator_enable,
	.disable = max77620_regulator_disable,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.set_mode = max77620_regulator_set_mode,
	.get_mode = max77620_regulator_get_mode,
	.set_ramp_delay = max77620_regulator_set_ramp_delay,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static int max77620_regulator_preinit(struct max77620_regulator *reg, int id)
{
	struct max77620_regulator_pdata *rpdata = &reg->reg_pdata[id];
	struct max77620_regulator_info *rinfo = reg->rinfo[id];
	struct device *parent = reg->max77620_chip->dev;
	struct regulator_init_data *ridata = reg->reg_pdata[id].reg_idata;
	u8 val, mask;
	int ret;

	/* Update power mode */
	ret = max77620_regulator_get_power_mode(reg, id);
	if (ret < 0)
		return ret;
	reg->current_power_mode[id] = ret;
	reg->enable_power_mode[id] = MAX77620_POWER_MODE_NORMAL;

	/*
	 * If rails are externally control of FPS control then enable it
	 * always.
	 */
	if ((rpdata->fps_src != FPS_SRC_NONE) &&
		(reg->current_power_mode[id] != reg->enable_power_mode[id])) {
		ret = max77620_regulator_set_power_mode(reg,
				reg->enable_power_mode[id], id);
		if (ret < 0) {
			dev_err(reg->dev, "Reg %d pm mode config failed %d\n",
				id, ret);
			return ret;
		}
	}

	/* Enable rail before changing FPS to NONE to avoid glitch */
	if (ridata && ridata->constraints.boot_on &&
		(rpdata->fps_src == FPS_SRC_NONE)) {
		ret = max77620_regulator_set_power_mode(reg,
				reg->enable_power_mode[id], id);
		if (ret < 0) {
			dev_err(reg->dev, "Reg %d pm mode config failed %d\n",
				id, ret);
			return ret;
		}
	}

	ret = max77620_regulator_set_fps_src(reg, rpdata->fps_src, id);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPSSRC to %d\n",
			rpdata->fps_src);
		return ret;
	}

	ret = max77620_regulator_set_fps_slots(reg, id);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPS Slots\n");
		return ret;
	}

	if (rinfo->type == MAX77620_REGULATOR_TYPE_SD) {
		int slew_rate;
		u8 val_u8;

		ret = max77620_reg_read(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, &val_u8);
		if (ret < 0) {
			dev_err(reg->dev, "Register 0x%02x read failed: %d\n",
					rinfo->cfg_addr, ret);
			return ret;
		}

		slew_rate = (val_u8 >> MAX77620_SD_SR_SHIFT) & 0x3;
		switch (slew_rate) {
		case 0:
			slew_rate = 13750;
			break;
		case 1:
			slew_rate = 27500;
			break;
		case 2:
			slew_rate = 55000;
			break;
		case 3:
			slew_rate = 100000;
			break;
		}
		rinfo->desc.ramp_delay = slew_rate;

		mask = MAX77620_SD_FSRADE_MASK;
		val = 0;
		if (rpdata->sd_fsrade_disable)
			val |= MAX77620_SD_FSRADE_MASK;

		ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, mask, val);
		if (ret < 0) {
			dev_err(reg->dev, "Reg 0x%02x update failed: %d\n",
				rinfo->cfg_addr, ret);
			return ret;
		}
	} else {
		int slew_rate;
		u8 val_u8;

		ret = max77620_reg_read(parent, MAX77620_PWR_SLAVE,
				rinfo->cfg_addr, &val_u8);
		if (ret < 0) {
			dev_err(reg->dev, "Register 0x%02x read failed: %d\n",
					rinfo->cfg_addr, ret);
			return ret;
		}
		slew_rate = (val_u8) & 0x1;
		switch (slew_rate) {
		case 0:
			slew_rate = 100000;
			break;
		case 1:
			slew_rate = 5000;
			break;
		}
		rinfo->desc.ramp_delay = slew_rate;
	}

	if ((id == MAX77620_REGULATOR_ID_SD0) && rpdata->en2_ctrl_sd0) {
		ret = max77620_regulator_set_power_mode(reg,
				MAX77620_POWER_MODE_NORMAL, id);
		if (ret < 0) {
			dev_err(reg->dev, "Reg %d pm mode set failed: %d\n",
				id, ret);
			return ret;
		}
		ret = max77620_regulator_set_fps_src(reg, FPS_SRC_0, id);
		if (ret < 0) {
			dev_err(reg->dev, "Reg %d fps src set failed: %d\n",
				id, ret);
			return ret;
		}
	}
	return 0;
}

#define REGULATOR_SD(_id, _name, _sname, _volt_mask, _min_uV, _max_uV,	\
		_step_uV)						\
	[MAX77620_REGULATOR_ID_##_id] = {			\
		.type = MAX77620_REGULATOR_TYPE_SD,			\
		.volt_mask =  MAX77620_##_volt_mask##_VOLT_MASK,	\
		.volt_addr = MAX77620_REG_##_id,		\
		.cfg_addr = MAX77620_REG_##_id##_CFG,		\
		.fps_addr = MAX77620_REG_FPS_##_id,		\
		.min_uV = _min_uV,				\
		.max_uV = _max_uV,				\
		.step_uV = _step_uV,				\
		.power_mode_mask = MAX77620_SD_POWER_MODE_MASK,		\
		.power_mode_shift = MAX77620_SD_POWER_MODE_SHIFT,	\
		.desc = {					\
			.name = max77620_rails(_name),		\
			.supply_name = _sname,			\
			.id = MAX77620_REGULATOR_ID_##_id,	\
			.ops = &max77620_regulator_ops,		\
			.n_voltages = ((_max_uV - _min_uV) / _step_uV) + 1, \
			.min_uV = _min_uV,	\
			.uV_step = _step_uV,	\
			.enable_time = 500,	\
			.vsel_mask = MAX77620_##_volt_mask##_VOLT_MASK,	\
			.vsel_reg = MAX77620_REG_##_id,	\
			.type = REGULATOR_VOLTAGE,	\
			.owner = THIS_MODULE,	\
		},						\
	}

#define REGULATOR_LDO(_id, _name, _sname, _type, _min_uV, _max_uV, _step_uV) \
	[MAX77620_REGULATOR_ID_##_id] = {			\
		.type = MAX77620_REGULATOR_TYPE_LDO_##_type,		\
		.volt_mask = MAX77620_LDO_VOLT_MASK,			\
		.volt_addr = MAX77620_REG_##_id##_CFG,		\
		.cfg_addr = MAX77620_REG_##_id##_CFG2,		\
		.fps_addr = MAX77620_REG_FPS_##_id,		\
		.min_uV = _min_uV,				\
		.max_uV = _max_uV,				\
		.step_uV = _step_uV,				\
		.power_mode_mask = MAX77620_LDO_POWER_MODE_MASK,	\
		.power_mode_shift = MAX77620_LDO_POWER_MODE_SHIFT,	\
		.desc = {					\
			.name = max77620_rails(_name),		\
			.supply_name = _sname,			\
			.id = MAX77620_REGULATOR_ID_##_id,	\
			.ops = &max77620_regulator_ops,		\
			.n_voltages = ((_max_uV - _min_uV) / _step_uV) + 1, \
			.min_uV = _min_uV,	\
			.uV_step = _step_uV,	\
			.enable_time = 500,	\
			.vsel_mask = MAX77620_LDO_VOLT_MASK,	\
			.vsel_reg = MAX77620_REG_##_id##_CFG, \
			.type = REGULATOR_VOLTAGE,		\
			.owner = THIS_MODULE,			\
		},						\
	}

static struct max77620_regulator_info max77620_regs_info[MAX77620_NUM_REGS] = {
	REGULATOR_SD(SD0, sd0, "in-sd0", SD0, 600000, 1400000, 12500),
	REGULATOR_SD(SD1, sd1, "in-sd1", SD1, 600000, 1550000, 12500),
	REGULATOR_SD(SD2, sd2, "in-sd2", SDX, 600000, 3787500, 12500),
	REGULATOR_SD(SD3, sd3, "in-sd3", SDX, 600000, 3787500, 12500),

	REGULATOR_LDO(LDO0, ldo0, "in-ldo0-1", N, 800000, 2375000, 25000),
	REGULATOR_LDO(LDO1, ldo1, "in-ldo0-1", N, 800000, 2375000, 25000),
	REGULATOR_LDO(LDO2, ldo2, "in-ldo2",   P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO3, ldo3, "in-ldo3-5", P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO4, ldo4, "in-ldo4-6", P, 800000, 1587500, 12500),
	REGULATOR_LDO(LDO5, ldo5, "in-ldo3-5", P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO6, ldo6, "in-ldo4-6", P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO7, ldo7, "in-ldo7-8", N, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO8, ldo8, "in-ldo7-8", N, 800000, 3950000, 50000),
};

static struct of_regulator_match max77620_regulator_matches[] = {
	{ .name = "sd0", },
	{ .name = "sd1", },
	{ .name = "sd2", },
	{ .name = "sd3", },
	{ .name = "ldo0", },
	{ .name = "ldo1", },
	{ .name = "ldo2", },
	{ .name = "ldo3", },
	{ .name = "ldo4", },
	{ .name = "ldo5", },
	{ .name = "ldo6", },
	{ .name = "ldo7", },
	{ .name = "ldo8", },
};

static int max77620_get_regulator_dt_data(struct platform_device *pdev,
		struct max77620_regulator *max77620_regs)
{
	struct device_node *np;
	u32 prop;
	int id;
	int ret;

	np = of_get_child_by_name(pdev->dev.parent->of_node, "regulators");
	if (!np) {
		dev_err(&pdev->dev, "Device is not having regulators node\n");
		return -ENODEV;
	}
	pdev->dev.of_node = np;

	ret = of_regulator_match(&pdev->dev, np, max77620_regulator_matches,
			ARRAY_SIZE(max77620_regulator_matches));
	if (ret < 0) {
		dev_err(&pdev->dev, "Parsing of regulator node failed: %d\n",
			ret);
		return ret;
	}

	for (id = 0; id < ARRAY_SIZE(max77620_regulator_matches); ++id) {
		struct device_node *reg_node;
		struct max77620_regulator_pdata *reg_pdata =
					&max77620_regs->reg_pdata[id];

		reg_node = max77620_regulator_matches[id].of_node;
		reg_pdata->reg_idata = max77620_regulator_matches[id].init_data;
		reg_pdata->glpm_enable = of_property_read_bool(reg_node,
					"maxim,enable-group-low-power");
		reg_pdata->en2_ctrl_sd0 = of_property_read_bool(reg_node,
					"maxim,enable-sd0-en2-control");

		reg_pdata->sd_fsrade_disable = of_property_read_bool(reg_node,
						"maxim,disable-active-discharge");

		ret = of_property_read_u32(reg_node, "maxim,fps-source", &prop);
		if (!ret)
			reg_pdata->fps_src = prop;
		else
			reg_pdata->fps_src = FPS_SRC_NONE;

		ret = of_property_read_u32(reg_node,
					"maxim,fps-power-up-period", &prop);
		if (!ret)
			reg_pdata->fps_pu_period = prop;
		else
			reg_pdata->fps_pu_period = -1;

		ret = of_property_read_u32(reg_node,
					"maxim,fps-power-down-period", &prop);
		if (!ret)
			reg_pdata->fps_pd_period = prop;
		else
			reg_pdata->fps_pd_period = -1;
	}
	return 0;
}

static int max77620_regulator_probe(struct platform_device *pdev)
{
	struct max77620_chip *max77620_chip = dev_get_drvdata(pdev->dev.parent);
	struct regulator_desc *rdesc;
	struct max77620_regulator *pmic;
	struct regulator_config config = { };
	int ret = 0;
	int id;

	pmic = devm_kzalloc(&pdev->dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(&pdev->dev, "memory alloc failed\n");
		return -ENOMEM;
	}

	max77620_get_regulator_dt_data(pdev, pmic);

	platform_set_drvdata(pdev, pmic);
	pmic->max77620_chip = max77620_chip;
	pmic->dev = &pdev->dev;

	for (id = 0; id < MAX77620_NUM_REGS; ++id) {
		rdesc = &max77620_regs_info[id].desc;
		pmic->rinfo[id] = &max77620_regs_info[id];
		pmic->enable_power_mode[id] = MAX77620_POWER_MODE_NORMAL;

		config.regmap = max77620_chip->rmap[MAX77620_PWR_SLAVE];
		config.dev = &pdev->dev;
		config.init_data = pmic->reg_pdata[id].reg_idata;
		config.driver_data = pmic;
		config.of_node = max77620_regulator_matches[id].of_node;

		ret = max77620_regulator_preinit(pmic, id);
		if (ret < 0) {
			dev_err(&pdev->dev, "Preinit regualtor %s failed: %d\n",
				rdesc->name, ret);
			return ret;
		}

		pmic->rdev[id] = devm_regulator_register(&pdev->dev,
						rdesc, &config);
		if (IS_ERR(pmic->rdev[id])) {
			ret = PTR_ERR(pmic->rdev[id]);
			dev_err(&pdev->dev,
				"regulator %s register failed: %d\n",
				rdesc->name, ret);
			return ret;
		}
	}

	return 0;
}

static struct platform_driver max77620_regulator_driver = {
	.probe = max77620_regulator_probe,
	.driver = {
		.name = "max77620-pmic",
		.owner = THIS_MODULE,
	},
};

static int __init max77620_regulator_init(void)
{
	return platform_driver_register(&max77620_regulator_driver);
}
subsys_initcall(max77620_regulator_init);

static void __exit max77620_reg_exit(void)
{
	platform_driver_unregister(&max77620_regulator_driver);
}
module_exit(max77620_reg_exit);

MODULE_AUTHOR("Mallikarjun Kasoju <mkasoju@nvidia.com>");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("max77620 regulator driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:max77620-pmic");
