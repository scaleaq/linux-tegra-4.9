/*
 * scaleaq_sensor_common.c - common code that can be used for scaleaq image sensor
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
#include <linux/i2c.h>
#include "scaleaq_sensor_common.h"

int temperature_sensor_init(struct i2c_client *cami2c, struct camera_common_eeprom_data *eeprom, u32 addr)
{
    char *dev_name = "temperature_imx";
    s64 temp = 0;

    eeprom->adap = i2c_get_adapter(cami2c->adapter->nr);
    memset(&eeprom->brd, 0, sizeof(eeprom->brd));
    strncpy(eeprom->brd.type, dev_name, sizeof(eeprom->brd.type));
    eeprom->brd.addr = (u16)addr;
    eeprom->i2c_client = i2c_new_device(eeprom->adap, &eeprom->brd);

    // dev_info(&cami2c->dev, "%s: temperature sensor addr 0x%02x initialized", __func__, addr);

    // test reading
    if (temperature_sensor_read(eeprom, &temp)) {
        dev_err(&eeprom->i2c_client->dev, "%s: error reading temperature", __func__);
        return -1;
    }

    return 0;
}
EXPORT_SYMBOL(temperature_sensor_init);

int temperature_sensor_release(struct camera_common_eeprom_data *eeprom)
{
    if (eeprom->i2c_client != NULL) {
        i2c_unregister_device(eeprom->i2c_client);
        eeprom->i2c_client = NULL;
    }

    return 0;
}
EXPORT_SYMBOL(temperature_sensor_release);

int temperature_sensor_read(struct camera_common_eeprom_data *eeprom, s64 *temperature)
{
    struct i2c_client *client = eeprom->i2c_client;
    int err;
    u8 buf[6] = { 0 };
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = buf,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = 2,
            .buf = buf + 1,
        },
    };

    err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    if (err != ARRAY_SIZE(msgs)) {
        dev_err(&client->dev, "%s: temperature reading failed", __func__);
        return -1;
    }
    // dev_info(&client->dev, "%s: sensor temperature 0x%02x%02x", __func__, buf[1], buf[2]);
    *temperature = ((buf[1] << 8) | buf[2]) * 7812; // micro oC

    return 0;
}
EXPORT_SYMBOL(temperature_sensor_read);

MODULE_DESCRIPTION("ScaleAQ image sensor common stuffs");
MODULE_AUTHOR("ScaleAQ.com");
MODULE_LICENSE("GPL v2");
