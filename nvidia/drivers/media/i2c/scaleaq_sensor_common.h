/*
 * scaleaq_sensor_common.h - common code that can be used for scaleaq image sensor
 *
 * Copyright (c) 2021. ScaleAQ.com.  All rights reserved.
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

#ifndef __scaleaq_sensor_common__
#define __scaleaq_sensor_common__
#include <media/camera_common.h>
#include <media/tegra_v4l2_camera.h>

#define TEGRA_CAMERA_CID_LCG                          (TEGRA_CAMERA_CID_BASE+160)
#define TEGRA_CAMERA_CID_SENSOR_TEMPERATURE           (TEGRA_CAMERA_CID_BASE+161)
#define TEGRA_CAMERA_CID_CLEAR_HDR_EN                 (TEGRA_CAMERA_CID_BASE+162)
#define TEGRA_CAMERA_CID_EXP_TH_H                     (TEGRA_CAMERA_CID_BASE+163)
#define TEGRA_CAMERA_CID_EXP_TH_L                     (TEGRA_CAMERA_CID_BASE+164)
#define TEGRA_CAMERA_CID_EXP_BK                       (TEGRA_CAMERA_CID_BASE+165)
#define TEGRA_CAMERA_CID_CCMP2_EXP                    (TEGRA_CAMERA_CID_BASE+166)
#define TEGRA_CAMERA_CID_CCMP1_EXP                    (TEGRA_CAMERA_CID_BASE+167)
#define TEGRA_CAMERA_CID_ACMP2_EXP                    (TEGRA_CAMERA_CID_BASE+168)
#define TEGRA_CAMERA_CID_ACMP1_EXP                    (TEGRA_CAMERA_CID_BASE+169)
#define TEGRA_CAMERA_CID_EXP_GAIN                     (TEGRA_CAMERA_CID_BASE+170)

int temperature_sensor_init(struct i2c_client *cami2c, struct camera_common_eeprom_data *eeprom, u32 addr);
int temperature_sensor_release(struct camera_common_eeprom_data *eeprom);
// Read temperature sensor, real value is `temperature`/1000000
int temperature_sensor_read(struct camera_common_eeprom_data *eeprom, s64 *temperature);

#endif /* __scaleaq_sensor_common__ */
