/*
 * imx585.c - imx585 sensor driver
 *
 * Copyright (c) 2022, ScaleAQ. All rights reserved.
 *
 * Contact us: post@scaleaq.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/imx585.h>

#if 1
#define print_dbg(fmt,args...) pr_info("imx585 %s(): DBG " fmt "\n", __func__, ##args)
#else
#define print_dbg(fmt,args...) pr_debug("imx585 %s(): DBG " fmt "\n", __func__, ##args)
#endif

#include "../platform/tegra/camera/camera_gpio.h"
#include "imx585_mode_tbls.h"

#define REGHOLD          0x3001
#define GAIN_LOW         0x306c
#define VMAX_LOW         0x3028
#define HMAX_LOW         0x302c
#define SHR0_LOW         0x3050
#define TPG_PATSEL_DUOUT 0x30e2
#define XVS_XHS_DRV      0x30a6


#define IMX585_K_FACTOR 1000LL
#define IMX585_M_FACTOR 1000000LL
#define IMX585_G_FACTOR 1000000000LL
#define IMX585_T_FACTOR 1000000000000LL

#define IMX585_MAX_BLACK_LEVEL_12BIT 0xFFC
#define IMX585_MAX_BLACK_LEVEL_10BIT 0x3FF

#define IMX585_MIN_SHR0_LENGTH 6
#define IMX585_MIN_INTEGRATION_LINES 2

#define IMX585_4_CSI_LANES 4
#define IMX585_TWO_LANE_MODE 2

#define IMX585_INCK 74250000LL

#define IMX585_DEFAULT_WIDTH            3856
#define IMX585_DEFAULT_HEIGHT           2180
#define IMX585_MIN_FRAME_LENGTH_DELTA 70

static const struct of_device_id imx585_of_match[] = {
	{.compatible = "scaleaq,imx585",},
	{},
};

MODULE_DEVICE_TABLE(of, imx585_of_match);

static int imx585_set_custom_ctrls(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops imx585_custom_ctrl_ops = {
	.s_ctrl = imx585_set_custom_ctrls,
};

static const char * const imx585_test_pattern_menu[] = {
	[0] = "No pattern",
	[1] = "000h Pattern",
	[2] = "3FF/FFFh Pattern",
	[3] = "155/555h Pattern",
	[4] = "2AA/AAAh Pattern",
	[5] = "155h/2AAh 555/AAAh Pattern",
	[6] = "2AAh/155h AAA/555h Pattern",
	[7] = "000h/155h 000/555h Pattern",
	[8] = "155h/000h 555/000h Pattern",
	[9] = "000h/3FFh 000/FFFh Pattern",
	[10] = "3FFh/000h FFF/000h Pattern",
	[11] = "H Color-bar",
	[12] = "V Color-bar",
};

static struct v4l2_ctrl_config imx585_custom_ctrl_list[] = {
	{
		.ops = &imx585_custom_ctrl_ops,
		.id = TEGRA_CAMERA_CID_TEST_PATTERN,
		.name = "Test Pattern",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(imx585_test_pattern_menu) - 1,
		.def = 0,
		.qmenu = imx585_test_pattern_menu,
	},
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct imx585 {
	struct i2c_client *i2c_client;
	struct v4l2_subdev *subdev;
	u16 fine_integ_time;
	u32 frame_length;
	u64 min_frame_length;
	u32 line_time;
	struct camera_common_data *s_data;
	struct tegracam_device *tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static void imx585_update_ctrl_range(struct tegracam_device *tc_dev,
				     int ctrl_id, u64 min, u64 max);

// static inline void imx585_get_frame_length_regs(imx585_reg * regs,
// 						u32 frame_length)
// {
// 	regs->addr = 0x302a;
// 	regs->val = (frame_length >> 16) & 0x0f;
// 	(regs + 1)->addr = 0x3029;
// 	(regs + 1)->val = (frame_length >> 8) & 0xff;
// 	(regs + 2)->addr = 0x3028;
// 	(regs + 2)->val = (frame_length) & 0xff;
// }

// static inline void imx585_get_coarse_integ_time_regs(imx585_reg * regs,
// 						     u32 coarse_time)
// {
// 	regs->addr = 0x3052;
// 	regs->val = (coarse_time >> 16) & 0x0f;
// 	(regs + 1)->addr = 0x3051;
// 	(regs + 1)->val = (coarse_time >> 8) & 0xff;
// 	(regs + 2)->addr = 0x3050;
// 	(regs + 2)->val = (coarse_time) & 0xff;
// }

// static inline void imx585_get_gain_reg(imx585_reg * reg, u16 gain)
// {
// 	reg->addr = 0x306d;
// 	reg->val = (gain >> IMX585_SHIFT_8_BITS) & IMX585_MASK_LSB_2_BITS;

// 	(reg + 1)->addr = 0x306c;
// 	(reg + 1)->val = (gain) & IMX585_MASK_LSB_8_BITS;
// }

static bool imx585_is_binning_mode(struct camera_common_data *s_data)
{
	switch (s_data->mode) {
	case IMX585_MODE_1928x1090_30FPS:
		return true;
	default:
		return false;
	}
}

static inline int imx585_read_reg(struct camera_common_data *s_data,
				  u16 addr, u8 * val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static inline int imx585_write_reg(struct camera_common_data *s_data,
				   u16 addr, u8 val)
{
	int err = 0;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

static int imx585_read_buffered_reg(struct camera_common_data *s_data,
				    u16 addr_low, u8 number_of_registers, u64 *val)
{
	struct device *dev = s_data->dev;
	int err, i;
	u8 reg;

	*val = 0;

	if (!s_data->group_hold_active){
		err = imx585_write_reg(s_data, REGHOLD, 0x01);
		if (err) {
			dev_err(dev, "%s: error setting register hold\n", __func__);
			return err;
		}
	}

	for (i = 0; i < number_of_registers; i++) {
		err = imx585_read_reg(s_data, addr_low + i, &reg);
		*val += reg << (i * 8);
		if (err) {
			dev_err(dev, "%s: error reading buffered registers\n", __func__);
			return err;
		}
	}

	if (!s_data->group_hold_active){
		err = imx585_write_reg(s_data, REGHOLD, 0x00);
		if (err) {
			dev_err(dev, "%s: error unsetting register hold\n", __func__);
			return err;
		}
	}

	return err;
}

static int imx585_write_buffered_reg(struct camera_common_data *s_data,
				     u16 addr_low, u8 number_of_registers, u64 val)
{
	int err, i;
	struct device *dev = s_data->dev;

	if (!s_data->group_hold_active){
		err = imx585_write_reg(s_data, REGHOLD, 0x01);
		if (err) {
			dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
			return err;
		}
	}

	for (i = 0; i < number_of_registers; i++) {
		err = imx585_write_reg(s_data, addr_low + i, (u8)(val >> (i * 8)));
		if (err) {
			dev_err(dev, "%s: BUFFERED register write error\n", __func__);
			return err;
		}
	}

	if (!s_data->group_hold_active){
		err = imx585_write_reg(s_data, REGHOLD, 0x00);
		if (err) {
			dev_err(dev, "%s: GRP_PARAM_HOLD erroror\n", __func__);
			return err;
		}
	}

	return err;
}

static int imx585_write_table(struct imx585 *priv, const imx585_reg table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
					 IMX585_TABLE_WAIT_MS,
					 IMX585_TABLE_END);
}

static int imx585_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	// print_dbg(" Setting group hold control to: %u", val);
	s_data->group_hold_active = val;

	err = imx585_write_reg(s_data, REGHOLD, val);
	if (err) {
		dev_err(dev, "%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

// static int imx585_get_fine_integ_time(struct imx585 *priv, u16 * fine_time)
// {
// 	struct camera_common_data *s_data = priv->s_data;
// 	int err = 0;
// 	u8 reg_val[2];

// 	err = imx585_read_reg(s_data, 0x302d,
// 			      &reg_val[0]);
// 	if (err)
// 		goto done;

// 	err = imx585_read_reg(s_data, 0x302c,
// 			      &reg_val[1]);
// 	if (err)
// 		goto done;

// 	*fine_time = (reg_val[0] << 8) | reg_val[1];
// 	// *fine_time = (hmax*IMX585_G_FACTOR) / (IMX585_INCK);

// done:
// 	return err;
// }

static int imx585_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
	    &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int err = 0;
	s16 gain;

	// print_dbg(" Setting gain control to: %lld", val);

	if (val < mode->control_properties.min_gain_val)
		val = mode->control_properties.min_gain_val;
	else if (val > mode->control_properties.max_gain_val)
		val = mode->control_properties.max_gain_val;

	/* Gain Formula:
	   Gain = (IMX585_GAIN_C0 - (IMX585_GAIN_C0 * gain_factor / val))
	 */
	gain = val * IMX585_MAX_GAIN_DEC /
                 (IMX585_MAX_GAIN_DB *
                     mode->control_properties.gain_factor);

	// if (gain < IMX585_MIN_GAIN)
	// 	gain = IMX585_MIN_GAIN;
	// else if (gain > IMX585_MAX_GAIN_DEC)
	// 	gain = IMX585_MAX_GAIN_DEC;

	// print_dbg(" val: %lld (/%d) [times], gain: %u",val, mode->control_properties.gain_factor, gain);

	err = imx585_write_buffered_reg(s_data, GAIN_LOW, 2, gain);
	if (err) {
		dev_err(dev, "%s: gain control error\n", __func__);
		return err;
	}

	return 0;
}

static struct v4l2_ctrl *imx585_find_v4l2_ctrl(struct tegracam_device *tc_dev,
					int ctrl_id)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct tegracam_ctrl_handler *handler = s_data->tegracam_ctrl_hdl;
	struct v4l2_ctrl *ctrl;
	int i;

	for (i = 0; i < handler->numctrls; i++) {
		ctrl = handler->ctrls[i];

		if ( ctrl->id == ctrl_id )
			return ctrl;
	}

	dev_warn(dev, "%s: Couldn't find control with [ %x ] id\n",
		 __func__, ctrl_id);
	return NULL;
}

static void imx585_update_ctrl_range(struct tegracam_device *tc_dev,
				     int ctrl_id, u64 min, u64 max)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct sensor_control_properties *ctrlprops = NULL;
	struct v4l2_ctrl *ctrl;
	const struct tegracam_ctrl_ops *ops = tc_dev->tcctrl_ops;
	int err = 0;

	ctrlprops =
		&s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

	ctrl = imx585_find_v4l2_ctrl(tc_dev, ctrl_id);
	if (ctrl == NULL) {
		return;
	}

	switch (ctrl_id) {
	case TEGRA_CAMERA_CID_EXPOSURE:
		s_data->exposure_min_range = min;
		s_data->exposure_max_range = max;

		dev_dbg(dev, "%s: recalculate exposure for set frame length.\n", __func__);
		err = ops->set_exposure(tc_dev, *ctrl->p_cur.p_s64);

		dev_dbg(dev, "%s:  mode: %u, exposure range [%llu, %llu]\n",
			 __func__, s_data->mode, s_data->exposure_min_range,
			s_data->exposure_max_range);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE_SHORT:
		s_data->short_exposure_min_range = min;
		s_data->short_exposure_max_range = max;

		/* Current value must be recalculated */
		dev_dbg(dev, "%s: recalculate short exposure for set frame length.\n", __func__);
		err = ops->set_exposure_short(tc_dev, *ctrl->p_cur.p_s64);

		dev_dbg(dev, "%s:  mode: %u, short exposure range [%llu, %llu]\n",
			__func__, s_data->mode, s_data->short_exposure_min_range,
			s_data->short_exposure_max_range);
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		ctrlprops->min_framerate = min;
		ctrlprops->max_framerate = max;
		/* default value must be in range */
		ctrlprops->default_framerate = clamp_val(ctrlprops->default_framerate,
							 ctrlprops->min_framerate,
							 ctrlprops->max_framerate);

		dev_dbg(dev, "%s:  mode: %u, framerate range [%u, %u]\n",
			 __func__, s_data->mode,
			ctrlprops->min_framerate, ctrlprops->max_framerate);
		break;
	}

	if (err) {
		dev_err(dev,
			"%s: ctrl %s range update failed\n", __func__, ctrl->name);
	}
}

static int imx585_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx585 *priv = (struct imx585 *)tc_dev->priv;
	const struct sensor_mode_properties *mode =
	    &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int err = 0;
	u64 frame_length;
	u64 exposure_max_range, exposure_min_range;

	pr_err(" Setting framerate control to: %lld\n", val);

	frame_length = (((u64)mode->control_properties.framerate_factor *
			IMX585_G_FACTOR) / (val * priv->line_time));

	// Value must be multiple of 2
	frame_length = (frame_length % 2) ? frame_length + 1 : frame_length;

	if (frame_length < priv->min_frame_length)
		frame_length = priv->min_frame_length;

	priv->frame_length = frame_length;

	exposure_min_range = (IMX585_MIN_INTEGRATION_LINES * priv->line_time) / IMX585_K_FACTOR;
	exposure_max_range = ((priv->frame_length - IMX585_MIN_INTEGRATION_LINES) * priv->line_time) / IMX585_K_FACTOR;
	imx585_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_EXPOSURE, exposure_min_range, exposure_max_range);

	print_dbg("val: %lld [fps], frame_length: %llu [lines], exposure time range [%llu-%llu]", val, frame_length, exposure_min_range, exposure_max_range);

	err = imx585_write_buffered_reg(s_data,
					VMAX_LOW, 3, priv->frame_length);

	return err;
}

static int imx585_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx585 *priv = (struct imx585 *)tc_dev->priv;
	struct v4l2_ctrl *ctrl;
	int err;
	u32 integration_time_line;
	u32 reg_shr0;

	// print_dbg(" Setting exposure control to: %lld", val);

	// coarse_time = (val - fine_integ_time_factor)
	//     * mode->signal_properties.pixel_clock.val
	//     / mode->control_properties.exposure_factor
	//     / mode->image_properties.line_length;

	// if (coarse_time < IMX585_MIN_COARSE_EXPOSURE)
	// 	coarse_time = IMX585_MIN_COARSE_EXPOSURE;
	// else if (coarse_time > max_coarse_time) {
	// 	coarse_time = max_coarse_time;
	// 	// print_dbg(" exposure limited by frame_length: %d [lines]", max_coarse_time);
	// }

	// print_dbg(" val: %lld [us], coarse_time: %d [lines]", val, coarse_time);

	// imx585_get_coarse_integ_time_regs(ct_regs, coarse_time);

	// for (i = 0; i < 3; i++) {
	// 	err = imx585_write_reg(s_data, ct_regs[i].addr, ct_regs[i].val);
	// 	if (err) {
	// 			dev_err(tc_dev->dev, "%s: coarse_time control error\n", __func__);
	// 		return err;
	// 	}
	// }

	// Check value with internal range
	if (val > s_data->exposure_max_range) {
		val = s_data->exposure_max_range;
	}
	else if (val < s_data->exposure_min_range) {
		val = s_data->exposure_min_range;
	}

	integration_time_line = (val * IMX585_K_FACTOR) / priv->line_time ;

	reg_shr0 = priv->frame_length - integration_time_line;

	// Value must be multiple of 2
	reg_shr0 = (reg_shr0 % 2) ? reg_shr0 + 1 : reg_shr0;

	if (reg_shr0 < IMX585_MIN_SHR0_LENGTH)
		reg_shr0 = IMX585_MIN_SHR0_LENGTH;
	else if (reg_shr0 > (priv->frame_length - IMX585_MIN_INTEGRATION_LINES))
		reg_shr0 = priv->frame_length - IMX585_MIN_INTEGRATION_LINES;

	err = imx585_write_buffered_reg (s_data, SHR0_LOW, 3, reg_shr0);
	if (err) {
		dev_err(tc_dev->dev, "%s: failed to set frame length\n", __func__);
		return err;
	}

	// Update new ctrl value
	ctrl = imx585_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_EXPOSURE);
	if (ctrl) {
		/* Value could be adjusted, set the right value */
		*ctrl->p_new.p_s64 = val;
		/* This ctrl is affected on FRAME RATE control also */
		*ctrl->p_cur.p_s64 = val;
	}

	// print_dbg("set integration time: %lld [us], coarse1:%d [line], shr0: %d [line], frame length: %u [line]",
	// 	val, integration_time_line, reg_shr0, priv->frame_length);

	return err;
}

static int imx585_set_test_pattern(struct tegracam_device *tc_dev, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx585 *priv = (struct imx585 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	if (val) {
		err = imx585_write_table(priv, mode_table[IMX585_MODE_TEST_PATTERN_EN]);
		if (err)
			goto pattern_err;
		err = imx585_write_reg(s_data, TPG_PATSEL_DUOUT, (u8)(val - 1));
		if (err)
			goto pattern_err;
	} else {
		err = imx585_write_table(priv, mode_table[IMX585_MODE_TEST_PATTERN_DIS]);
		if (err)
			goto pattern_err;
	}

	print_dbg("++ Test mode pattern: %u", val-1);

	return 0;
pattern_err:
	dev_err(dev, "%s: error setting test pattern\n", __func__);
	return err;
}

static int imx585_set_custom_ctrls(struct v4l2_ctrl *ctrl)
{
	struct tegracam_ctrl_handler *handler = container_of(ctrl->handler, struct tegracam_ctrl_handler, ctrl_handler);
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;
	struct tegracam_device *tc_dev = handler->tc_dev;
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_TEST_PATTERN:
		err = ops->set_test_pattern(tc_dev, *ctrl->p_new.p_u32);
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static struct tegracam_ctrl_ops imx585_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx585_set_gain,
	.set_exposure = imx585_set_exposure,
	.set_frame_rate = imx585_set_frame_rate,
	.set_group_hold = imx585_set_group_hold,
	.set_test_pattern = imx585_set_test_pattern,
};

static int imx585_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	print_dbg("");
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 0);
		else
			gpio_set_value(pw->reset_gpio, 0);
		usleep_range(10,20);
	}

	if (!pw->mclk) {
		dev_err(dev, "%s: mclk not available\n",  __func__);
		goto imx585_avdd_fail;
	}

	if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
		goto skip_power_seqn;

	usleep_range(10, 20);

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto imx585_avdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto imx585_iovdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto imx585_dvdd_fail;
	}

	// err = clk_prepare_enable(pw->mclk);
	if(err) {
		dev_err(dev, "%s: failed to enable mclk\n",  __func__);
		goto imx585_dvdd_fail;
	}

	usleep_range(10, 20);

skip_power_seqn:
	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 1);
		else
			gpio_set_value(pw->reset_gpio, 1);
	}

	/* Need to wait for t4 + t5 + t9 + t10 time as per the data sheet */
	/* t4 - 200us, t5 - 21.2ms, t9 - 1.2ms t10 - 270 ms */
	usleep_range(300000, 300100);

	pw->state = SWITCH_ON;

	return 0;

imx585_dvdd_fail:
	regulator_disable(pw->iovdd);

imx585_iovdd_fail:
	regulator_disable(pw->avdd);

imx585_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int imx585_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	print_dbg("");
	err = imx585_write_reg(s_data, XVS_XHS_DRV, 0x0F);
	if (err) {
		dev_err(dev, "%s: error setting XVS XHS to Hi-Z\n", __func__);
	}

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		// clk_disable_unprepare(pw->mclk);

		if (pw->reset_gpio) {
			if (gpio_cansleep(pw->reset_gpio))
				gpio_set_value_cansleep(pw->reset_gpio, 0);
			else
				gpio_set_value(pw->reset_gpio, 0);
		}

		usleep_range(10, 10);

		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->avdd)
			regulator_disable(pw->avdd);
	}

	pw->state = SWITCH_OFF;

	return 0;
}

static int imx585_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	print_dbg("");

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	return 0;
}

static int imx585_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	print_dbg("");

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "extperiph1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
						   &pw->avdd,
						   pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
						   &pw->iovdd,
						   pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
						   &pw->dvdd,
						   pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	/* Reset or ENABLE GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
			__func__, err);
		goto done;
	}

done:
	pw->state = SWITCH_OFF;

	return err;
}

static struct camera_common_pdata *imx585_parse_dt(struct tegracam_device
						   *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err = 0;
	int gpio;

	print_dbg("");

	if (!np)
		return NULL;

	match = of_match_device(imx585_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(dev,
					 board_priv_pdata);
	if (err) {
		dev_err(dev, "Failed to find clocks\n");
		goto error;
	}

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found\n");
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	// err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	// if (err)
	// 	dev_err(dev, "mclk name not present, "
	// 		"assume sensor driven externally\n");

	err = of_property_read_string(np, "avdd-reg",
				      &board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
				       &board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
				       &board_priv_pdata->regulators.dvdd);
	if (err)
		dev_err(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
			"assume sensor powered independently\n");

	// board_priv_pdata->has_eeprom = of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

/**
 * Calculate 1H time
 */
static int imx585_calculate_line_time(struct tegracam_device *tc_dev)
{
	struct imx585 *priv = (struct imx585 *)tc_dev->priv;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	u64 hmax;
	int err;


	err = imx585_read_buffered_reg(s_data, HMAX_LOW, 2, &hmax);
	if (err) {
		dev_err(dev, "%s: unable to read hmax\n", __func__);
		return err;
	}

	priv->line_time = (hmax * IMX585_G_FACTOR) / (IMX585_INCK);

	print_dbg("hmax: %llu [inck], INCK: %u [Hz], line_time: %u [ns]",
		  hmax, s_data->def_clk_freq, priv->line_time);

    return 0;
}

static int imx585_update_framerate_range(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx585 *priv = (struct imx585 *)tegracam_get_privdata(tc_dev);
	struct sensor_control_properties *ctrlprops = NULL;
	u64 max_framerate;

	ctrlprops =
		&s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

	if (imx585_is_binning_mode(s_data))
		priv->min_frame_length =  IMX585_DEFAULT_HEIGHT + IMX585_MIN_FRAME_LENGTH_DELTA;
	else
		priv->min_frame_length = s_data->fmt_height + IMX585_MIN_FRAME_LENGTH_DELTA;

	max_framerate = (IMX585_G_FACTOR * IMX585_M_FACTOR) / (priv->min_frame_length * priv->line_time);
	print_dbg("frame rate range [%u-%llu]", ctrlprops->min_framerate, max_framerate);
	imx585_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_FRAME_RATE, ctrlprops->min_framerate, max_framerate);

	return 0;
}

static int imx585_set_mode(struct tegracam_device *tc_dev)
{
	struct imx585 *priv = (struct imx585 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;

	int err = 0;

	print_dbg("mode: %d", s_data->mode);

	err = imx585_write_table(priv, mode_table[IMX585_MODE_COMMON]);
	if (err)
		return err;

	err = imx585_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	err = imx585_calculate_line_time(tc_dev);
	if (err)
		return err;

	imx585_update_framerate_range(tc_dev);

	return 0;
}

static int imx585_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx585 *priv = (struct imx585 *)tegracam_get_privdata(tc_dev);

	print_dbg("");
	return imx585_write_table(priv, mode_table[IMX585_START_STREAM]);
}

static int imx585_stop_streaming(struct tegracam_device *tc_dev)
{
	int err;
	struct imx585 *priv = (struct imx585 *)tegracam_get_privdata(tc_dev);

	print_dbg("");
	err = imx585_write_table(priv, mode_table[IMX585_STOP_STREAM]);

	return err;
}

static struct camera_common_sensor_ops imx585_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx585_frmfmt),
	.frmfmt_table = imx585_frmfmt,
	.power_on = imx585_power_on,
	.power_off = imx585_power_off,
	.write_reg = imx585_write_reg,
	.read_reg = imx585_read_reg,
	.parse_dt = imx585_parse_dt,
	.power_get = imx585_power_get,
	.power_put = imx585_power_put,
	.set_mode = imx585_set_mode,
	.start_streaming = imx585_start_streaming,
	.stop_streaming = imx585_stop_streaming,
};

static int imx585_board_setup(struct imx585 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	u8 reg_val[2];
	int err = 0;

	print_dbg(" enable MCLK with %d Hz", s_data->def_clk_freq);

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = imx585_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto done;
	}

	usleep_range(1000,2000);
	/* Probe sensor model id registers */
	err = imx585_read_reg(s_data, 0x5a1d, &reg_val[0]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n", __func__, err);
		goto err_reg_probe;
	}
	err = imx585_read_reg(s_data, 0x5a1e, &reg_val[1]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n", __func__, err);
		goto err_reg_probe;
	}
	dev_info(dev, "Identification registers: 0x%02x 0x%02x\n", reg_val[0], reg_val[1]);

	if (!((reg_val[0] != 0x49) || ((reg_val[1] & 0x03) != 0x02)))
		dev_err(dev, "%s: sensor model id: %x%x is NOT IMX585\n",
			__func__, reg_val[0], reg_val[1]);

	err = imx585_calculate_line_time(priv->tc_dev);
	if (err) {
		dev_err(dev, "%s: unable to calculate line time\n", __func__);
	}

	priv->min_frame_length = IMX585_DEFAULT_HEIGHT + IMX585_MIN_FRAME_LENGTH_DELTA;

err_reg_probe:
	print_dbg(" return value %d.", err);
	imx585_power_off(s_data);
	camera_common_mclk_disable(s_data);

done:
	print_dbg(" return value %d.", err);
	return err;
}

static int imx585_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	print_dbg(" client %p", client);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx585_subdev_internal_ops = {
	.open = imx585_open,
};

static int imx585_ctrls_init(struct tegracam_device *tc_dev)
{
	struct imx585 *priv = (struct imx585 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = priv->s_data;
	struct v4l2_ctrl_config *ctrl_cfg;
	struct v4l2_ctrl *ctrl;
	struct tegracam_ctrl_handler *handler = s_data->tegracam_ctrl_hdl;
	int numctrls;
	int err, i;

	numctrls = ARRAY_SIZE(imx585_custom_ctrl_list);

	for (i = 0; i < numctrls; i++) {
		ctrl_cfg = &imx585_custom_ctrl_list[i];

		ctrl = v4l2_ctrl_new_custom(&handler->ctrl_handler, ctrl_cfg, NULL);
		if (ctrl == NULL) {
			dev_err(dev, "%s: Failed to create control %s\n", __func__, ctrl_cfg->name);
			continue;
		}

		if (ctrl_cfg->type == V4L2_CTRL_TYPE_STRING && ctrl_cfg->flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(tc_dev->dev, ctrl_cfg->max + 1, GFP_KERNEL);
			if (!ctrl->p_new.p_char) {
				dev_err(dev, "%s: failed to allocate memory\n", __func__);
				return -ENOMEM;
			}
		}
		handler->ctrls[handler->numctrls + i] = ctrl;
		print_dbg("Added custom control %s to handler index: %d",
			  ctrl_cfg->name,  handler->numctrls + i);
	}

	handler->numctrls = handler->numctrls + numctrls;

	err = handler->ctrl_handler.error;
	if (err) {
		dev_err(dev, "Error %d adding controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&handler->ctrl_handler);
	return err;
}

static int imx585_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx585 *priv;
	int err;

	print_dbg(" probing v4l2 sensor at addr 0x%0x", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct imx585), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx585", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx585_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx585_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx585_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx585_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	err = imx585_ctrls_init(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera custom ctrl init failed\n");
		return err;
	}

	err = v4l2_async_register_subdev(priv->subdev);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(dev, "detected imx585 sensor\n");

	return 0;
}

static int imx585_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx585 *priv = (struct imx585 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx585_id[] = {
	{"imx585", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, imx585_id);

static struct i2c_driver imx585_i2c_driver = {
	.driver = {
		   .name = "imx585",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(imx585_of_match),
		   },
	.probe = imx585_probe,
	.remove = imx585_remove,
	.id_table = imx585_id,
};

module_i2c_driver(imx585_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX585");
MODULE_AUTHOR("Scale Aqualculture AS");
MODULE_LICENSE("GPL v2");
