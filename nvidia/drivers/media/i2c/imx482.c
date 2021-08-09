/*
 * imx482.c - imx482 sensor driver
 *
 * Copyright (c) 2021. ScaleAQ.  All rights reserved.
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
//#define DEBUG 1
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

#include "imx482_mode_tbls.h"
#include "framos_sensor_common.h"

#define IMX482_K_FACTOR 1000LL
#define IMX482_M_FACTOR 1000000LL
#define IMX482_G_FACTOR 1000000000LL
#define IMX482_T_FACTOR 1000000000000LL

#define IMX482_MAX_GAIN_DEC 240
#define IMX482_MAX_GAIN_DB  72

#define IMX482_MAX_BLACK_LEVEL 0x3FF

#define IMX482_MIN_SHR0_LENGTH 6
#define IMX482_MIN_INTEGRATION_LINES 2

#define IMX482_4_CSI_LANES 4
#define IMX482_TWO_LANE_MODE 2

#define IMX482_INCK 74250000LL

/**
 * list HEAD of private data of all probed sensors on the platform
 */
LIST_HEAD(imx482_sensor_list);

static int test_pattern = 0;
module_param(test_pattern, int, 0644);

/**
 * Declaration
 */
static void imx482_configure_second_slave_address(void);
static int imx482_set_exposure(struct tegracam_device *tc_dev, s64 val);


static const struct of_device_id imx482_of_match[] = {
	{ .compatible = "framos,imx482",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx482_of_match);


static int imx482_set_custom_ctrls(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops imx482_custom_ctrl_ops = {
	.s_ctrl = imx482_set_custom_ctrls,
};

const char * const imx482_data_rate_menu[] = {
	[IMX482_1782_MBPS] = "1782 Mbps/lane",
	[IMX482_1440_MBPS] = "1440 Mbps/lane",
};

static const char * const imx482_test_pattern_menu[] = {
	[0] = "No pattern",
	[1] = "H Color-bar",
	[2] = "V Color-bar",
};

static struct v4l2_ctrl_config imx482_custom_ctrl_list[] = {
	{
		.ops = &imx482_custom_ctrl_ops,
		.id = TEGRA_CAMERA_CID_DATA_RATE,
		.name = "Data Rate",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(imx482_data_rate_menu) - 1,
		.def = 0,
		.qmenu = imx482_data_rate_menu,
	},
	{
		.ops = &imx482_custom_ctrl_ops,
		.id = TEGRA_CAMERA_CID_TEST_PATTERN,
		.name = "Test Pattern",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(imx482_test_pattern_menu) - 1,
		.def = 0,
		.qmenu = imx482_test_pattern_menu,
	},
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
	TEGRA_CAMERA_CID_STREAMING_MODE,
	TEGRA_CAMERA_CID_OPERATION_MODE,
	TEGRA_CAMERA_CID_SYNC_FUNCTION,
	TEGRA_CAMERA_CID_BROADCAST,
	TEGRA_CAMERA_CID_BLACK_LEVEL,
};

struct imx482 {
	struct i2c_client           *i2c_client;
	struct v4l2_subdev          *subdev;
	u64                         frame_length;
	u64                         min_frame_length;
	u64                         black_level;
	u32                         line_time;
	u8                          data_rate;
	streaming_mode              current_streaming_mode;
	operation_mode              current_operation_mode;
	sync_mode                   current_sync_mode;
	i2c_broadcast_ctrl          broadcast_ctrl;
	struct mutex                pw_mutex;
	struct list_head            entry;
	struct camera_common_data   *s_data;
	struct tegracam_device      *tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline int imx482_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx482_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

/**
 * I2C command is braodcasted using the 2nd i2c address
 */
static int imx482_write_reg_broadcast(struct camera_common_data *s_data,
										u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->broadcast_regmap, addr, val);
	if (err) {
		dev_err(dev, "%s: i2c write failed, %x = %x\n",
			__func__, addr, val);
	}

	return err;
}

/**
 * Reads multiple sequential registers at the same time using grouphold
 */
static int imx482_read_buffered_reg(struct camera_common_data *s_data,
								 u16 addr_low, u8 number_of_registers, u64 *val)
{
	struct device *dev = s_data->dev;
	int err, i;
	u8 reg;

	*val = 0;

	if (!s_data->group_hold_active){
		err = imx482_write_reg(s_data, REGHOLD, 0x01);
		if (err) {
			dev_err(dev, "%s: error setting register hold\n", __func__);
			return err;
		}
	}

	for (i = 0; i < number_of_registers; i++) {
		err = imx482_read_reg(s_data, addr_low + i, &reg);
		*val += reg << (i * 8);
		if (err) {
			dev_err(dev, "%s: error reading buffered registers\n", __func__);
			return err;
		}
	}

	if (!s_data->group_hold_active){
		err = imx482_write_reg(s_data, REGHOLD, 0x00);
		if (err) {
			dev_err(dev, "%s: error unsetting register hold\n", __func__);
			return err;
		}
	}

	return err;
}

/**
 * Writes multiple sequential registers at the same time using reghold
 */
static int imx482_write_buffered_reg(struct camera_common_data *s_data,
								u16 addr_low, u8 number_of_registers, u64 val)
{
	int err, i;
	struct device *dev = s_data->dev;

	if (!s_data->group_hold_active){
		err = imx482_write_reg(s_data, REGHOLD, 0x01);
		if (err) {
			dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
			return err;
		}
	}

	for (i = 0; i < number_of_registers; i++) {
		err = imx482_write_reg(s_data, addr_low + i, (u8)(val >> (i * 8)));
		if (err) {
			dev_err(dev, "%s: BUFFERED register write error\n", __func__);
			return err;
		}
	}

	if (!s_data->group_hold_active){
		err = imx482_write_reg(s_data, REGHOLD, 0x00);
		if (err) {
			dev_err(dev, "%s: GRP_PARAM_HOLD erroror\n", __func__);
			return err;
		}
	}

	return err;
}

/**
 * Broadcast multiple sequential registers at the same time using reghold
 */
static int imx482_broadcast_buffered_reg(struct camera_common_data *s_data,
								u16 addr_low, u8 number_of_registers, u32 val)
{
	int err, i;
	struct device *dev = s_data->dev;

	if (!s_data->group_hold_active){
		err = imx482_write_reg_broadcast(s_data, REGHOLD, 0x01);
		if (err) {
			dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
			return err;
		}
	}

	for (i = 0; i < number_of_registers; i++) {
		err = imx482_write_reg_broadcast(s_data, addr_low + i,
					(u8)(val >> (i * 8)));
		if (err) {
			dev_err(dev, "%s: BUFFERED register write error\n", __func__);
			return err;
		}
	}

	if (!s_data->group_hold_active){
		err = imx482_write_reg_broadcast(s_data, REGHOLD, 0x00);
		if (err) {
			dev_err(dev, "%s: GRP_PARAM_HOLD erroror\n", __func__);
			return err;
		}
	}

	return err;
}

static int imx482_write_table(struct imx482 *priv,
				const imx482_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 IMX482_TABLE_WAIT_MS,
					 IMX482_TABLE_END);
}

static int imx482_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	s_data->group_hold_active = val;

	err = imx482_write_reg(s_data, REGHOLD, val);
	if (err) {
		dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
		return err;
	}

	return err;
}

static int imx482_update_ctrl(struct tegracam_device *tc_dev, u64 val)
{
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
    struct v4l2_ctrl *ctrl;

	/* Update Black level control*/
    ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL);
    if (ctrl) {
        *ctrl->p_new.p_s64 = val;
        *ctrl->p_cur.p_s64 = val;
		ctrl->default_value = priv->black_level;
    }

    return 0;
}

static int imx482_set_black_level(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;
	s64 black_level_reg;

	switch (s_data->colorfmt->code) {
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		    black_level_reg = val >> 2;
			break;
		default:
			if (val > IMX482_MAX_BLACK_LEVEL)
				val = IMX482_MAX_BLACK_LEVEL;
			err = imx482_update_ctrl(tc_dev, val);
			black_level_reg = val;
	}

    err = imx482_write_buffered_reg(s_data, BLKLEVEL_LOW, 2, black_level_reg);
	if (err){
		dev_dbg(dev, "%s: BLACK LEVEL control error\n", __func__);
		return err;
	}

	dev_dbg(dev, "%s: black level: %lld\n",  __func__, val);

	return 0;
}

static int imx482_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	int err;
	u32 gain;

	/* translate value */
	gain = val * IMX482_MAX_GAIN_DEC /
				 (IMX482_MAX_GAIN_DB *
					 mode->control_properties.gain_factor);


	if (priv->broadcast_ctrl == BROADCAST)
		err = imx482_broadcast_buffered_reg(s_data,
											 GAIN_LOW, 2, gain);
	else {
		err = imx482_write_buffered_reg(s_data,
											GAIN_LOW, 2, gain);
	}
	if (err){
		dev_dbg(dev, "%s: GAIN control error\n", __func__);
		return err;
	}

	dev_dbg(dev, "%s:  gain val [%lld] reg [%d]\n",  __func__, val, gain);

	return 0;
}


static int imx482_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	struct v4l2_ctrl *ctrl;
	int err;
	u32 integration_time_line;
	u32 reg_shr0;

	dev_dbg(dev, "%s: integration time: %lld [us]\n", __func__, val);

	/* Check value with internal range */
	if (val > s_data->exposure_max_range) {
		val = s_data->exposure_max_range;
	}
	else if (val < s_data->exposure_min_range) {
		val = s_data->exposure_min_range;
	}

	integration_time_line = (val * IMX482_K_FACTOR) / priv->line_time ;

	reg_shr0 = priv->frame_length - integration_time_line;

	/* Value must be multiple of 2 */
	reg_shr0 = (reg_shr0 % 2) ? reg_shr0 + 1 : reg_shr0;

	if (reg_shr0 < IMX482_MIN_SHR0_LENGTH)
		reg_shr0 = IMX482_MIN_SHR0_LENGTH;
	else if (reg_shr0 > (priv->frame_length - IMX482_MIN_INTEGRATION_LINES))
		reg_shr0 = priv->frame_length - IMX482_MIN_INTEGRATION_LINES;

	if (priv->broadcast_ctrl == BROADCAST)
		err = imx482_broadcast_buffered_reg(s_data, SHR0_LOW, 3, reg_shr0);
	else
		err = imx482_write_buffered_reg (s_data, SHR0_LOW, 3, reg_shr0);
	if (err) {
		dev_err(dev, "%s: failed to set frame length\n", __func__);
		return err;
	}

	/* Update new ctrl value */
	ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_EXPOSURE);
	if (ctrl) {
		/* Value could be adjusted, set the right value */
		*ctrl->p_new.p_s64 = val;
		/* This ctrl is affected on FRAME RATE control also */
		*ctrl->p_cur.p_s64 = val;
	}

	dev_dbg(dev,
	 "%s: set integration time: %lld [us], coarse1:%d [line], shr0: %d [line], frame length: %llu [line]\n",
	 __func__, val, integration_time_line, reg_shr0, priv->frame_length);

	return err;
}

static int imx482_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err=0;
	u64 frame_length;
	u64 exposure_max_range, exposure_min_range;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];

	frame_length = (((u64)mode->control_properties.framerate_factor *
							  IMX482_G_FACTOR) / (val * priv->line_time));

	/* Value must be multiple of 2 */
	frame_length = (frame_length % 2) ? frame_length + 1 : frame_length;

	if (frame_length < priv->min_frame_length)
		frame_length = priv->min_frame_length;

	priv->frame_length = frame_length;

	/* Update exposure range, before writing the new frame length */
	exposure_min_range = IMX482_MIN_INTEGRATION_LINES
											* priv->line_time / IMX482_K_FACTOR;

	exposure_max_range = (priv->frame_length - IMX482_MIN_INTEGRATION_LINES)
											* priv->line_time / IMX482_K_FACTOR;
	fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_EXPOSURE,
							 exposure_min_range, exposure_max_range);

	if (priv->broadcast_ctrl == BROADCAST)
		err = imx482_broadcast_buffered_reg(s_data,
											  VMAX_LOW, 3, priv->frame_length);
	else
		err = imx482_write_buffered_reg(s_data,
										  VMAX_LOW, 3, priv->frame_length);
	if (err) {
		dev_err(dev, "%s: failed to set frame length\n", __func__);
		return err;
	}

	dev_dbg(dev,
		"%s: val: %lld, frame_length set: %llu\n",
			 __func__, val, priv->frame_length);

	return 0;
}

/**
 * Test pattern is described in the "Pattern Generator (PG)" chapter
 * in the IMX482 support package
 */
static int imx482_set_test_pattern(struct tegracam_device *tc_dev, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	if (val) {
		err = imx482_write_table(priv, mode_table[IMX482_EN_PATTERN_GEN]);
		if (err)
			goto fail;
		err = imx482_write_reg(s_data, TPG_PATSEL_DUOUT, (u8)(val - 1 + 0x0A));
		if (err)
			goto fail;
	} else {
		err = imx482_write_table(priv, mode_table[IMX482_DIS_PATTERN_GEN]);
		if (err)
			goto fail;
	}

	dev_dbg(dev, "%s++ Test mode pattern: %u\n", __func__, val-1);

	return 0;
fail:
	dev_err(dev, "%s: error setting test pattern\n", __func__);
	return err;
}

/**
 * Update max framerate range
 */
static int imx482_update_framerate_range(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct sensor_control_properties *ctrlprops = NULL;
	u64 max_framerate;

	ctrlprops =
		&s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

	priv->min_frame_length =  IMX482_DEFAULT_HEIGHT
										+ IMX482_MIN_FRAME_LENGTH_DELTA;

	max_framerate = (IMX482_G_FACTOR * IMX482_M_FACTOR) /
									 (priv->min_frame_length * priv->line_time);

	fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_FRAME_RATE,
							ctrlprops->min_framerate, max_framerate);

	return 0;
}

/**
 * Set streaming mode
 */
static int imx482_set_streaming_mode(struct tegracam_device *tc_dev, u32 val)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;

	priv->current_streaming_mode = val;

	return 0;
}

/**
 * Set operation mode of sensor
 */
static int imx482_set_operation_mode(struct tegracam_device *tc_dev, u32 val)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;

	priv->current_operation_mode = val;

	return 0;
}

static int imx482_set_sync_feature(struct tegracam_device *tc_dev, u32 val)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;

	priv->current_sync_mode = val;

	return 0;
}

static int imx482_set_broadcast_ctrl(struct tegracam_device *tc_dev,
										struct v4l2_ctrl *ctrl)
{
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	dev_dbg(dev, "%s++\n", __func__);

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE) {
		ctrl->val = UNICAST;
		dev_info(dev, "%s: Broadcast control is inactive\n", __func__);
		return 0;
	}

	err = common_get_broadcast_client(tc_dev, ctrl, &sensor_regmap_config);
	if (err) {
		return err;
	}

	priv->broadcast_ctrl =  *ctrl->p_new.p_u8;
	imx482_configure_second_slave_address();

	return 0;
}

static int imx482_set_custom_ctrls(struct v4l2_ctrl *ctrl)
{
	struct tegracam_ctrl_handler *handler =
								container_of(ctrl->handler,
									struct tegracam_ctrl_handler, ctrl_handler);
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;
	struct tegracam_device *tc_dev = handler->tc_dev;
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_DATA_RATE:
		priv->data_rate = *ctrl->p_new.p_u8;
		break;
	case TEGRA_CAMERA_CID_TEST_PATTERN:
		err = ops->set_test_pattern(tc_dev, *ctrl->p_new.p_u32);
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static struct tegracam_ctrl_ops imx482_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx482_set_gain,
	.set_exposure = imx482_set_exposure,
	.set_frame_rate = imx482_set_frame_rate,
	.set_group_hold = imx482_set_group_hold,
	.set_test_pattern = imx482_set_test_pattern,
	.set_streaming_mode = imx482_set_streaming_mode,
	.set_operation_mode = imx482_set_operation_mode,
	.set_sync_feature = imx482_set_sync_feature,
	.set_broadcast_ctrl = imx482_set_broadcast_ctrl,
	.set_black_level = imx482_set_black_level,
};

static int imx482_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	struct imx482 *priv = (struct imx482 *)s_data->priv;

	dev_dbg(dev, "%s: power on\n", __func__);

	mutex_lock(&priv->pw_mutex);

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		mutex_unlock(&priv->pw_mutex);
		return err;
	}

	if (!pw->mclk) {
		dev_err(dev, "%s: mclk not available\n",  __func__);
		goto imx482_mclk_fail;
	}

	/* Power ON sequence according to IMX482 datasheet */

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto imx482_dvdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto imx482_iovdd_fail;
	}

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto imx482_avdd_fail;
	}

	usleep_range(1, 2);

	if (gpio_is_valid(pw->reset_gpio))
		fm_gpio_set(s_data, pw->reset_gpio, 1);

	err = clk_prepare_enable(pw->mclk);
	if(err) {
		dev_err(dev, "%s: failed to enable mclk\n",  __func__);
		return err;
	}

	/* MUST be before sleep */
	pw->state = SWITCH_ON;

	/* Additional sleep required in the case of hardware power-on sequence */
	usleep_range(40000, 41000);

	mutex_unlock(&priv->pw_mutex);

	imx482_configure_second_slave_address();

	return 0;

imx482_avdd_fail:
	regulator_disable(pw->iovdd);

imx482_iovdd_fail:
	regulator_disable(pw->dvdd);

imx482_dvdd_fail:
imx482_mclk_fail:
	mutex_unlock(&priv->pw_mutex);
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int imx482_power_off(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	struct imx482 *priv = (struct imx482 *)s_data->priv;
	int err = 0;

	dev_dbg(dev, "%s: power off\n", __func__);

	/**
	 * Put XVS & XHS pin to Hi-Z before power-off sequence, as described
	 * in the chapter "XVS/XHS IO setting" in the IMX482 Synchronizing
	 * Sensors App Note
	 */
	err = imx482_write_reg(s_data, XVS_XHS_DRV, 0xF);
	if (err) {
		dev_err(dev, "%s: error setting XVS XHS to Hi-Z\n", __func__);
	}

	mutex_lock(&priv->pw_mutex);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		mutex_unlock(&priv->pw_mutex);
		return err;
	}

	/* Power OFF sequence according to IMX482 datasheet */

	clk_disable_unprepare(pw->mclk);

	if (gpio_is_valid(pw->reset_gpio))
		fm_gpio_set(s_data, pw->reset_gpio, 0);

	if (pw->avdd)
		regulator_disable(pw->avdd);
	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

power_off_done:
	pw->state = SWITCH_OFF;
	mutex_unlock(&priv->pw_mutex);

	return 0;
}

/**
 * Acquires regulators, clock and GPIO defined in platform_data
 */
static int imx482_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

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

	/* Analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* Dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
				&pw->dvdd, pdata->regulators.dvdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	if (err) {
		dev_info(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	pw->reset_gpio = pdata->reset_gpio;

	if (pdata->use_cam_gpio) {
		err = cam_gpio_register(dev, pw->reset_gpio);
		if (err)
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->reset_gpio);
	}

done:
	pw->state = SWITCH_OFF;
	return err;
}

/**
 * Frees regulators acquired in power_get
 */
static int imx482_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd)) {
		regulator_put(pw->avdd);
	}

	if (likely(pw->iovdd)) {
		regulator_put(pw->iovdd);
	}

	if (likely(pw->dvdd)) {
		regulator_put(pw->dvdd);
	}

	pw->avdd = NULL;
	pw->iovdd = NULL;
	pw->dvdd = NULL;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_deregister(dev, pw->reset_gpio);
	else {
		if (gpio_is_valid(pw->reset_gpio))
			gpio_free(pw->reset_gpio);
	}

	return 0;
}

/**
 * Read frame length register to confirm communication
 */
static int imx482_communication_verify(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;
	u64 vmax, black_level;

	err = imx482_read_buffered_reg(s_data, VMAX_LOW, 2, &vmax);
	if (err) {
		dev_err(dev, "%s: failed to read VMAX\n", __func__);
		return err;
	}

	/* Initialize frame length */
	priv->frame_length = vmax;

	err = imx482_read_buffered_reg(s_data, BLKLEVEL_LOW, 2, &black_level);
    if (err) {
        dev_err(dev, "%s: failed to read BLKLEVEL\n", __func__);
        return err;
    }
	/* Initialize black level */
	priv->black_level = black_level;

	return err;
}

static struct camera_common_pdata *imx482_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(imx482_of_match, dev);
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
		dev_err(dev, "reset-gpios not found %d\n", err);
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(np, "vsync-gpios", 0);
	if (gpio > 0) {
		gpio_direction_input(gpio);
	}

	fm_get_gpio_ctrl(board_priv_pdata);

	err = of_property_read_string(np, "avdd-reg",
		&board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
		&board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
		&board_priv_pdata->regulators.dvdd);
	if (err)
		dev_dbg(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
			"assume sensor powered independently\n");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);
	return ret;
}

/**
 * Configure CSI lane mode registers
 */
static int imx482_set_csi_lane_mode(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	if (s_data->numlanes == IMX482_TWO_LANE_MODE) {
		err = imx482_write_reg(s_data, LANEMODE, 1);
		if (err) {
			dev_err(dev, "%s: error setting two lane mode\n", __func__);
			return err;
		}
	}

	dev_dbg(dev, "%s: sensor is in %d CSI lane mode\n",
				  __func__, s_data->numlanes);

	return 0;
}

/**
 * Calculate 1H time
 */
static int imx482_calculate_line_time(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	u64 hmax;
	int err;

	dev_dbg(dev, "%s:++\n", __func__);

	err = imx482_read_buffered_reg(s_data, HMAX_LOW, 2, &hmax);
	if (err) {
		dev_err(dev, "%s: unable to read hmax\n", __func__);
		return err;
	}

	priv->line_time = (hmax*IMX482_G_FACTOR) / (IMX482_INCK);

	dev_dbg(dev, "%s: hmax: %llu [inck], INCK: %u [Hz], line_time: %u [ns]\n",
			__func__, hmax, s_data->def_clk_freq, priv->line_time);

	return 0;
}

/**
 * Adjust HMAX register, and other properties for selected data rate
 */
static int imx482_adjust_hmax_register(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;
	u64 hmax = 0x226;
	u8 csi_lane_coef = IMX482_4_CSI_LANES / s_data->numlanes;

	dev_dbg(dev, "%s:++\n", __func__);

	switch (priv->data_rate) {
	case IMX482_1782_MBPS:
		if (s_data->numlanes == 2)
			hmax = 0x10C * csi_lane_coef;
		else {
			if (s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB10_1X10) {
				hmax = 0x1C6 * csi_lane_coef;
			} else {
				hmax = 0x226 * csi_lane_coef;
			}
		}
		break;

	case IMX482_1440_MBPS:
			hmax = 0x16E * csi_lane_coef;
		break;
	default:
		/* Adjusment isn't needed */
		return 0;
	}

	err = imx482_write_buffered_reg(s_data, HMAX_LOW, 2, hmax);
	if (err) {
		dev_err(dev, "%s: failed to set HMAX register\n", __func__);
		return err;
	}

	dev_dbg(dev, "%s:  HMAX: %llu\n", __func__, hmax);

	return 0;
}

/**
 * These limitation are described in the "Readout Drive mode" chapter
 * in the IMX482 datasheet
 */
static int imx482_verify_data_rate(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	struct v4l2_ctrl *ctrl;

	if (s_data->numlanes != IMX482_TWO_LANE_MODE
		&& priv->data_rate == IMX482_1782_MBPS) {
		priv->data_rate = IMX482_1440_MBPS;
		goto modify_ctrl;
	}

	dev_dbg(dev, "%s++\n", __func__);

	return 0;

modify_ctrl:
	   dev_warn(dev, "%s: Selected data rate is not supported in this mode, switching to default!\n",  __func__);
	ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_DATA_RATE);
	v4l2_ctrl_s_ctrl(ctrl, priv->data_rate);
	return 0;
}

/**
 * IMX482 allows for multiple data rates. Available data rates are described
 * in the chapter "Readout Drive mode" in the IMX482 datasheet
 */
static int imx482_set_data_rate(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	dev_dbg(dev, "%s:++\n", __func__);

	err = imx482_verify_data_rate(tc_dev);
	if (err)
		goto fail;

	err = imx482_write_table(priv, data_rate_table[priv->data_rate]);
	if (err)
		goto fail;

	dev_dbg(dev, "%s: Data rate: %u\n", __func__, priv->data_rate);

	return 0;

fail:
	dev_err(dev, "%s: unable to set data rate\n", __func__);
	return err;
}

/**
 * Synchronization mode is for Master mode
 * Sensor can be synchronized Externaly and Internaly in Master mode
 */
static int imx482_set_sync_mode(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;
	u8 extmode;

	if (priv->current_sync_mode == INTERNAL_SYNC)
		extmode = 0;
	else
		extmode = 1;

	err = imx482_write_reg(s_data, EXTMODE, extmode);
	if (err)
		dev_err(dev, "%s: error setting operation mode\n", __func__);

	return err;
}

/**
 * Finds first sensor with broadcast enabled
 */
static bool imx482_find_broadcast_sensor(struct imx482 *broadcast_private)
{
	struct imx482 *current_private;

	list_for_each_entry(current_private, &imx482_sensor_list, entry) {
		mutex_lock(&current_private->pw_mutex);
		if (current_private->broadcast_ctrl == BROADCAST
			&& current_private->s_data->power->state == SWITCH_ON) {
			mutex_unlock(&current_private->pw_mutex);
			memcpy(broadcast_private, current_private, sizeof(*broadcast_private));
			return true;
		}
		mutex_unlock(&current_private->pw_mutex);
	}
	return false;
}

/**
 * Enables second slave address on all sensors
 * Enables acknowledge on one sensor
 */
static void imx482_enable_second_slave_address(struct imx482 *ack_private)
{
	struct imx482 *current_private;
	int err;

	list_for_each_entry(current_private, &imx482_sensor_list, entry) {
		mutex_lock(&current_private->pw_mutex);
		/* Continue if sensor is in Power-off state */
		if (current_private->s_data->power->state != SWITCH_ON) {
			mutex_unlock(&current_private->pw_mutex);
			continue;
		}

		err = imx482_write_reg(current_private->s_data,
								SECOND_SLAVE_ADD, 1);
		if (err)
			dev_warn(&current_private->i2c_client->dev,
				"%s: Fail to write Second I2C register\n", __func__);

		mutex_unlock(&current_private->pw_mutex);

		dev_dbg(&current_private->i2c_client->dev,
				"%s: Sensors 2nd slave address configured\n", __func__);
	}

	err = imx482_write_reg(ack_private->s_data,
								SECOND_SLAVE_ADD, 3);
	if (err)
		dev_warn(&ack_private->i2c_client->dev,
			"%s: Fail to write Second I2C register\n", __func__);

	dev_dbg(&ack_private->i2c_client->dev,
			": Sensors 2nd slave address configured with acknowlege\n");
}

/**
 * Disables second slave address in all sensors
 */
static void imx482_disable_second_slave_address(void)
{
	struct imx482 *current_private;
	int err;

	list_for_each_entry(current_private, &imx482_sensor_list, entry) {
		mutex_lock(&current_private->pw_mutex);
		/* Continue if sensor is in Power-off state */
		if (current_private->s_data->power->state != SWITCH_ON) {
			mutex_unlock(&current_private->pw_mutex);
			continue;
		}

		err = imx482_write_reg(current_private->s_data,
								SECOND_SLAVE_ADD, 0);
		if (err)
			dev_warn(&current_private->i2c_client->dev,
				"%s: Fail to write Second I2C register\n", __func__);

		mutex_unlock(&current_private->pw_mutex);

		dev_dbg(&current_private->i2c_client->dev,
				"%s: Sensors 2nd slave address disabled\n", __func__);
	}
}

/**
 * If broadcast active, configrue 2nd slave address
 * Only one sensor can have valid 2nd slave address ACK enabled
 */
static void imx482_configure_second_slave_address(void)
{
	struct imx482 broadcast_private = {};

	if (imx482_find_broadcast_sensor(&broadcast_private)) {
		imx482_enable_second_slave_address(&broadcast_private);
	} else {
		imx482_disable_second_slave_address();
	}
}

/**
 * XVS & XHS are synchronizing/triggering pins
 * This sensor supports - Internal and External synchronization in master mode
 *                      - External synchronization in slave mode
 *       XVS     XHS
 * 0x0 - output, output
 * 0x3 - hi-z,   output
 * 0xC - output, hi-z
 * 0xF - hi-z,   hi-z
 */
static int imx482_configure_triggering_pins(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err = 0;
	u8  xvs_xhs_drv = 0xF;

	switch (priv->current_operation_mode) {
	case MASTER_MODE:
		if (priv->current_streaming_mode == SYNC_STREAM
			&& priv->current_sync_mode == INTERNAL_SYNC) {
			/* XVS - output, XHS - output */
			xvs_xhs_drv = 0;
			dev_dbg(dev,
					"%s: Sensor is in Internal sync Master mode\n", __func__);
		}
		else {
			/* XVS - hi-z, XHS - hi-z */
			xvs_xhs_drv = 0xF;
			dev_dbg(dev,
					"%s: Sensor is in External sync Master mode\n", __func__);
		}

		break;

	case SLAVE_MODE:
		/* XVS - hi-z, XHS - hi-z */
		xvs_xhs_drv = 0xF;
		dev_dbg(dev, "%s: Sensor is in Slave mode\n", __func__);

		break;

	default:
		pr_err("%s: unknown synchronizing function.\n", __func__);
		return -EINVAL;
	}

	err = imx482_write_reg(s_data, XVS_XHS_DRV, xvs_xhs_drv);
	if (err) {
		dev_err(dev, "%s: error setting Slave mode\n", __func__);
		return err;
	}

	dev_dbg(dev, "%s: XVS_XHS driver register: %x\n", __func__, xvs_xhs_drv);

	return 0;
}

/**
 * Check that all sensors have the same streaming mode
 * Streaming in different modes could cause hardware damage
 */
static bool imx482_is_stream_configuration_valid(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct imx482 *current_priv;
	struct device *dev = tc_dev->dev;
	u8 streaming_mode = priv->current_streaming_mode;

	list_for_each_entry(current_priv, &imx482_sensor_list, entry) {
		if (streaming_mode != current_priv->current_streaming_mode) {
			dev_err (dev,
				   "streaming mode not compatible with other sensors, all sensors must have the same streaming mode\n");
			return false;
		}
	}

	return true;
}

/**
 * Check that two pins never drive the same line
 * Two pins driving the same line could cause hardware damage
 */
static bool imx482_is_pin_configuration_valid(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tc_dev->priv;
	struct imx482 *current_priv;
	struct device *dev = tc_dev->dev;
	u8 xvs_drive = 0;
	u8 xhs_drive = 0;

	list_for_each_entry(current_priv, &imx482_sensor_list, entry) {
		if (current_priv->current_operation_mode == MASTER_MODE
			&& current_priv->current_sync_mode == INTERNAL_SYNC) {
			xvs_drive++;
			xhs_drive++;
			dev_dbg(dev, "%s: XVS & XHS as output\n", __func__);
		}
	}

	if (priv->current_streaming_mode == SYNC_STREAM
			&& (xvs_drive > 1 || xhs_drive > 1)) {
		dev_warn(dev, "More than one XVS/XHS are configured as OUTPUT!\n");
		return false;
	}
	else if (priv->current_streaming_mode == SYNC_STREAM
				&& (xvs_drive == 0 || xhs_drive == 0)) {
		dev_warn(dev, "Configure one sensor to drive XVS/XHS!\n");
		return false;
	}
	else if (priv->current_streaming_mode == EXTERNAL_HW_SYNC_STREAM
				&& (xvs_drive || xhs_drive)) {
		dev_warn(dev, "Configure all XVS/XHS as INPUT pin!\n");
		return false;
	}

	return true;
}

/**
 * According to the V4L2 documentation, driver should not return error when
 * invalid settings are detected.
 * It should apply the settings closest to the ones that the user has requested.
 */
int imx482_check_unsupported_mode(struct camera_common_data *s_data,
									 struct v4l2_mbus_framefmt *mf)
{
	struct device *dev = s_data->dev;
	bool unsupported_mode = false;

	dev_dbg(dev, "%s++\n", __func__);

	/**
	 * Binning is supported only in 12 bit mode according to the chapter "Operation mode"
	 * in the IMX482 datasheet
	 */

	if (mf->code == MEDIA_BUS_FMT_SRGGB10_1X10) {
		unsupported_mode = true;
		dev_warn(dev,
		"%s: selected mode is not supported with RAW10, switching to default\n",
				  __func__);
	}

	if (unsupported_mode){
		mf->width   = s_data->frmfmt[s_data->def_mode].size.width;
		mf->height  = s_data->frmfmt[s_data->def_mode].size.height;
	}

	return 0;
}

static int imx482_set_mode(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	err = imx482_write_table(priv, mode_table[IMX482_INIT_SETTINGS]);
	if (err) {
		dev_err(dev, "%s: unable to initialize sensor settings\n", __func__);
		return err;
	}

	err = imx482_set_csi_lane_mode(tc_dev);
	if (err) {
		dev_err(dev, "%s: error setting CSI lane mode\n", __func__);
		return err;
	}

	err = imx482_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	err = imx482_set_operation_mode(tc_dev, priv->current_operation_mode);
	if (err) {
		dev_err(dev, "%s: unable to operation mode\n", __func__);
		return err;
	}

	err = imx482_set_sync_mode(tc_dev);
	if (err) {
		dev_err(dev, "%s: unable to set sync mode\n", __func__);
		return err;
	}

	err = imx482_configure_triggering_pins(tc_dev);
	if (err) {
		dev_err(dev, "%s: unable configure XVS/XHS pins\n", __func__);
		return err;
	}

	err = imx482_set_data_rate(tc_dev);
	if (err) {
		dev_err(dev, "%s: unable to set data rate\n", __func__);
		return err;
	}

	err = imx482_adjust_hmax_register(tc_dev);
	if (err) {
		dev_err(dev, "%s: unable to adjust hmax\n", __func__);
		return err;
	}

	/* Override V4L GAIN, EXPOSURE and FRAME RATE controls */
	s_data->override_enable = true;

	if (!imx482_is_stream_configuration_valid(tc_dev)) {
		dev_err(dev, "%s: illegal stream configuration detected\n", __func__);
		return -EPERM;
	}

	if (!imx482_is_pin_configuration_valid(tc_dev)) {
		dev_err(dev, "%s: illegal pin configuration detected\n", __func__);
		return -EPERM;
	}

	err = imx482_calculate_line_time(tc_dev);
	if (err)
		return err;

	err = imx482_update_framerate_range(tc_dev);
	if (err)
		return err;

	dev_dbg(dev, "%s: set mode %u\n", __func__, s_data->mode);

	return 0;
}

static int imx482_start_streaming(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	if (test_pattern > 0) {
		err = imx482_write_table(priv,
			mode_table[IMX482_EN_PATTERN_GEN]);
		if (err) {
			dev_err(dev, "%s: write test pattern table failed, ret %d\n", __func__, err);
			return err;
		}

		// Set pattern
		err = imx482_write_reg(s_data, TPG_PATSEL_DUOUT, (u8)(test_pattern - 1 + 0x0A));
		if (err) {
			dev_err(dev, "%s: write test pattern value %d failed, ret %d\n", __func__, test_pattern - 1 + 0x0A, err);
			return err;
		}
	}

	err = imx482_write_table(priv,
		mode_table[IMX482_MODE_START_STREAM]);
	if (err)
		return err;

	return 0;
}

static int imx482_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx482_write_table(priv, mode_table[IMX482_MODE_STOP_STREAM]);
	if (err)
		return err;

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * delay = frame length rows * Tline
	 */
	usleep_range(priv->frame_length * priv->line_time / IMX482_K_FACTOR,
				priv->frame_length * priv->line_time / IMX482_K_FACTOR + 1000);

	return 0;
}


static struct camera_common_sensor_ops imx482_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx482_frmfmt),
	.frmfmt_table = imx482_frmfmt,
	.power_on = imx482_power_on,
	.power_off = imx482_power_off,
	.write_reg = imx482_write_reg,
	.read_reg = imx482_read_reg,
	.parse_dt = imx482_parse_dt,
	.power_get = imx482_power_get,
	.power_put = imx482_power_put,
	.set_mode = imx482_set_mode,
	.start_streaming = imx482_start_streaming,
	.stop_streaming = imx482_stop_streaming,
	.check_unsupported_mode = imx482_check_unsupported_mode,
};


static int imx482_board_setup(struct imx482 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = imx482_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

	err = imx482_communication_verify(priv->tc_dev);
	if (err) {
		dev_err(dev, "%s: unable to communicate with sensor\n",  __func__);
		goto error;
	}

	err = imx482_calculate_line_time(priv->tc_dev);
	if (err) {
		dev_err(dev, "%s: unable to calculate line time\n", __func__);
		goto error;
	}

	priv->min_frame_length = IMX482_DEFAULT_HEIGHT
								+ IMX482_MIN_FRAME_LENGTH_DELTA;

error:
	imx482_power_off(s_data);
	camera_common_mclk_disable(s_data);

	return err;
}

static int imx482_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx482_subdev_internal_ops = {
	.open = imx482_open,
};

static int imx482_ctrls_init(struct tegracam_device *tc_dev)
{
	struct imx482 *priv = (struct imx482 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = priv->s_data;
	struct v4l2_ctrl_config *ctrl_cfg;
	struct v4l2_ctrl *ctrl;
	struct tegracam_ctrl_handler *handler = s_data->tegracam_ctrl_hdl;
	int numctrls;
	int err, i;

	numctrls = ARRAY_SIZE(imx482_custom_ctrl_list);

	for (i = 0; i < numctrls; i++) {
		ctrl_cfg = &imx482_custom_ctrl_list[i];

		ctrl = v4l2_ctrl_new_custom(&handler->ctrl_handler, ctrl_cfg, NULL);
		if (ctrl == NULL) {
			dev_err(dev, "%s: Failed to create control %s\n", __func__,
															  ctrl_cfg->name);
			continue;
		}

		if (ctrl_cfg->type == V4L2_CTRL_TYPE_STRING &&
				ctrl_cfg->flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(tc_dev->dev,
									ctrl_cfg->max + 1, GFP_KERNEL);
			if (!ctrl->p_new.p_char) {
				dev_err(dev, "%s: failed to allocate memory\n", __func__);
				return -ENOMEM;
			}
		}
		handler->ctrls[handler->numctrls + i] = ctrl;
		dev_dbg(dev, "%s: Added custom control %s to handler index: %d\n",
			__func__, ctrl_cfg->name,  handler->numctrls + i);
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

static int imx482_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx482 *priv;
	struct sensor_control_properties *ctrlprops = NULL;
	int err;

	dev_info(dev, "probing v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx482), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	mutex_init(&priv->pw_mutex);
	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx482", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx482_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx482_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx482_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	priv->current_streaming_mode = STANDALONE_STREAM;
	priv->current_operation_mode = MASTER_MODE;
	priv->current_sync_mode = EXTERNAL_SYNC;
	priv->broadcast_ctrl = UNICAST;
	priv->s_data->broadcast_regmap = NULL;
	priv->s_data->blklvl_max_range = IMX482_MAX_BLACK_LEVEL;

	/* Get default device tree properties of first sensor mode */
	ctrlprops =
		&priv->s_data->sensor_props.sensor_modes[0].control_properties;

	priv->s_data->exposure_min_range = ctrlprops->min_exp_time.val;
	priv->s_data->exposure_max_range = ctrlprops->max_exp_time.val;

	INIT_LIST_HEAD(&priv->entry);

	err = imx482_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	err = imx482_ctrls_init(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera custom ctrl init failed\n");
		return err;
	}

    err = imx482_update_ctrl(tc_dev, priv->black_level);
    if (err)
		return err;

    err = v4l2_async_register_subdev(priv->subdev);
    if (err) {
        dev_err(dev, "tegra camera subdev registration failed\n");
        return err;
    }

	list_add_tail(&priv->entry, &imx482_sensor_list);

	dev_info(dev, "Detected imx482 sensor\n");

	return 0;
}

static int imx482_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx482 *priv = (struct imx482 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx482_id[] = {
	{ "imx482", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx482_id);

static struct i2c_driver imx482_i2c_driver = {
	.driver = {
		.name = "imx482",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx482_of_match),
	},
	.probe = imx482_probe,
	.remove = imx482_remove,
	.id_table = imx482_id,
};

module_i2c_driver(imx482_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX482");
MODULE_AUTHOR("ScaleAQ");
MODULE_LICENSE("GPL v2");
