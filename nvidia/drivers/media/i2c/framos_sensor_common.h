/*
 * framos_sensor_common.h - common logic
 *
 * Copyright (c) 2019. FRAMOS.  All rights reserved.
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

#ifndef __framos_sensor_common__
#define __framos_sensor_common__

/**
 * Crosslink FPGA firmware data
 */

#define CROSSLINK_FW_NAME_BASE      0x0010
#define CROSSLINK_FW_VERSION_BASE   0x0020

#define CROSSLINK_FW_NAME_REG_LENGTH    16
#define CROSSLINK_FW_VERSION_REG_LENGTH 4

#define CROSSLINK_IS_WIDTH_LOW      0x00A0
#define CROSSLINK_IS_WIDTH_HIGH     0x00A1
#define CROSSLINK_IS_HEIGHT_LOW     0x00A2
#define CROSSLINK_IS_HEIGHT_HIGH    0x00A3
#define CROSSLINK_DATA_RATE_LOW     0x00A4
#define CROSSLINK_DATA_RATE_HIGH    0x00A5
#define CROSSLINK_PIX_FORMAT        0x00A6
#define CROSSLINK_LVDS_NUM_CH       0x00A7

#define CROSSLINK_MASTER_SLAVE      0x00E0 /*XMASTER pin*/

#define CROSSLINK_LINE_SKIP_COUNT_RW    0x00F0
#define CROSSLINK_V_ACTIVE_LOW_RW       0x00F1    
#define CROSSLINK_V_ACTIVE_HIGH_RW      0x00F2
#define CROSSLINK_LEFT_TRIM_UNIT_RW     0x00F3
#define CROSSLINK_LEFT_TRIM_LANE_RW     0x00F4
#define CROSSLINK_H_ACTIVE_UNIT_LOW_RW  0x00F5
#define CROSSLINK_H_ACTIVE_UNIT_HIGH_RW 0x00F6
#define CROSSLINK_WORD_COUNT_LOW_RW     0x00F7
#define CROSSLINK_WORD_COUNT_HIGH_RW    0x00F8
#define CROSSLINK_SW_RST_N_RW           0x00F9

#define CROSSLINK_TG_ENABLE				0x0100
#define CROSSLINK_TG_MODE               0x0101
#define CROSSLINK_SYNC_LOGIC			0x0102
#define CROSSLINK_OUT_LOGIC				0x0103

#define CROSSLINK_LINE_ACTIVE_WIDTH		0x0110
#define CROSSLINK_LINE_INACTIVE_WIDTH   0x0112

#define CROSSLINK_FRAME_DELAY			0x0120

#define CROSSLINK_FRAME_ACTIVE_WIDTH	0x0124
#define CROSSLINK_FRAME_INACTIVE_WIDTH	0x0128

#define MIN_TRIGGER_HIGH_WIDTH_TGES         11

extern const struct crosslink_readout_mode cl_def_readout_mode;

/**
 * Enum SOM
 */
enum {
    JETSON_TX2,
    JETSON_XAVIER,
    JETSON_NANO_DEV_KIT,
    JETSON_XAVIER_NX_DEV_KIT,
};

/**
 * Enum describing available operation modes
 */
typedef enum {
    MASTER_MODE,
    SLAVE_MODE,
} operation_mode;

/**
 * Enum describing available operation modes
 */
typedef enum {
    TG_DISABLED,
    TG_MASTER_MODE,
    TG_SLAVE_MODE,
    TG_TRIGGER_MODE,   
} tg_mode;

/**
 * Enum describing available shutter modes
 */
typedef enum {
    NORMAL_EXPO,
    SEQ_TRIGGER,
} shutter_mode;

/**
 * Enum describing available streaming modes
 */
typedef enum {
    STANDALONE_STREAM,
    SYNC_STREAM,
    EXTERNAL_HW_SYNC_STREAM,
} streaming_mode;

/**
 * Enum describing available sync modes
 */
typedef enum {
    INTERNAL_SYNC,
    EXTERNAL_SYNC,
} sync_mode;

/**
 * Enum describing available communication modes
 */
typedef enum {
    UNICAST,
    BROADCAST,
} i2c_broadcast_ctrl;

typedef struct reg_8 crosslink_reg;

struct crosslink_readout_mode {
    u8  line_skip;
    u8  v_active_low;
    u8  v_active_high;
    u8  left_trim_unit;
    u8  left_trim_lane;
    u8  h_active_unit_low;
    u8  h_active_unit_high;
    u8  word_count_low;
    u8  word_count_high;
};

struct time_generator_params {
    tg_mode tg_mode;
    u8 sync_logic;
    u8 out_logic;
    u32 xhs_min_active_width;
    u32 xhs_clk_offset;
    u32 frame_delay;
    u32 frame_active_width;
    u32 frame_inactive_width;
    u32 t_tgpd;
    u32 frame_length;
    u32 expanded_exposure;
	u32 *line_time;
    operation_mode *is_operation_mode;
    shutter_mode *is_shutter_mode;
};

struct crosslink_private {
	int	sensor_numch;
    struct time_generator_params tg_params;
    struct crosslink_readout_mode cl_readout_mode;
};

void fm_get_board_model(struct tegracam_device *tc_dev, u8 *som_board_model);

void fm_calc_lshift(u64 *val, u8 *lshift, s64 max);

struct v4l2_ctrl *fm_find_v4l2_ctrl(struct tegracam_device *tc_dev, 
                                                int ctrl_id);
void fm_update_ctrl_range(struct tegracam_device *tc_dev,
                                              int ctrl_id, u64 min, u64 max);

void fm_gpio_set(struct camera_common_data *s_data,
			    unsigned int gpio, int val);

void fm_get_gpio_ctrl(struct camera_common_pdata *board_priv_pdata);

int cam_gpio_register(struct device *dev,
			unsigned pin_num);

void cam_gpio_deregister(struct device *dev,
			unsigned pin_num);

int cam_gpio_ctrl(struct device *dev,
			unsigned pin_num, int ref_inc, bool active_high);

int common_get_broadcast_client(struct tegracam_device *tc_dev,
                                struct v4l2_ctrl *ctrl, 
                                const struct regmap_config *sensor_regmap_config);

int common_get_lvds2mipi_client(struct tegracam_device *tc_dev);
int common_verify_crosslink_fw_compatibility(struct tegracam_device *tc_dev, s32 *val);

int crosslink_set_readout_mode(struct tegracam_device *tc_dev);
int crosslink_set_is_operation_mode(struct tegracam_device *tc_dev);

int tg_set_operation_mode(struct tegracam_device *tc_dev, 
                                    struct v4l2_ctrl *ctrl);

int tg_set_trigger_exposure(struct tegracam_device *tc_dev, u32 val);
int tg_expand_trigger_exposure(struct tegracam_device *tc_dev, u32 val);
int tg_delay_frame(struct tegracam_device *tc_dev, u32 val);
int tg_set_line_width(struct tegracam_device *tc_dev);
int tg_set_frame_width(struct tegracam_device *tc_dev, u32 val);
int tg_start(struct tegracam_device *tc_dev);
int tg_stop(struct tegracam_device *tc_dev);
int crosslink_start(struct tegracam_device *tc_dev);
int crosslink_stop(struct tegracam_device *tc_dev);

#endif /* __framos_sensor_common__ */
