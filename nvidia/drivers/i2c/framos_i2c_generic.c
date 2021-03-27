/*
 * framos_i2c_generic.c - i2c generic driver
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

//#define DEBUG 1
#include <linux/module.h>

#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/string.h>

#include <media/camera_common.h>

#include "i2c/framos_i2c_generic.h"

enum generic_i2c_dev_type {
	broadcast_mux_ch,
	lvds2mpip_crosslink,
};

struct broadcast_channel {
    struct i2c_client * device;    
};

struct broadcast_channel *mux_broadcast = NULL;
 
struct lvds2mpip {
    struct i2c_client * device;  
    struct list_head entry;  
};

/**
 * list HEAD of all probed crosslink devices on platform
 */
LIST_HEAD(lvds2mipi_dev_list);

/**
 * Get i2c-mux broadcast channel 
 */
struct i2c_client *get_broadcast_client(struct i2c_client *image_sensor)
{
    if (mux_broadcast == NULL) {
       dev_err(&image_sensor->dev,"Couldn't get handle of mux i2c_client\n");
        return NULL;
    }

    return mux_broadcast->device;
}
EXPORT_SYMBOL(get_broadcast_client);

/**
 * Get crosslink i2c_client on the same bus
 */
struct i2c_client *get_lvds2mipi_client(struct i2c_client *image_sensor)
{
    struct lvds2mpip *current_crosslink;
    int sensor_adapter_id = i2c_adapter_id(image_sensor->adapter);

    list_for_each_entry(current_crosslink, &lvds2mipi_dev_list, entry) {
        if (sensor_adapter_id == i2c_adapter_id(current_crosslink->device->adapter))
            return current_crosslink->device;    
    }

    dev_err(&image_sensor->dev,"Couldn't get handle of crosslink i2c_client\n");
    return NULL;
}
EXPORT_SYMBOL(get_lvds2mipi_client);

static struct of_device_id generic_sd_of_match[] = {
    { .compatible = "framos, mux_broadcast_ch"},
    { .compatible = "framos, lvds2mipi"},
    {},
};

MODULE_DEVICE_TABLE(of, generic_sd_of_match);

static int generic_sd_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    const struct of_device_id *match;
    struct device_node *np = client->dev.of_node;
    const char *name;
    int err;
    struct lvds2mpip *crosslink;

    dev_info(&client->dev, "%s\n", __func__);

    match = of_match_device(generic_sd_of_match, &client->dev);
    if (!match) {
        dev_info(&client->dev, "Failed to find matching dt id\n");
        return -1;
    }

    err = of_property_read_string(np, "device", &name);
    if (err) {
        dev_err(&client->dev, "device not in Device Tree\n");
    }

    if (!strcmp(name, "i2c-mux")){
        mux_broadcast = devm_kzalloc(&client->dev, sizeof(struct broadcast_channel), GFP_KERNEL);
        if (!mux_broadcast) {
            dev_err(&client->dev, "%s: unable to allocate memory\n", __func__);
            return -ENOMEM;
        }
        mux_broadcast->device = client;
        dev_info(&client->dev,"%s broadcast channel registered.\n",name);
    }
    else if(!strcmp(name, "crosslink")){

        crosslink = devm_kzalloc(&client->dev, sizeof(struct lvds2mpip), GFP_KERNEL);
        if (!crosslink) {
            dev_err(&client->dev, "%s: unable to allocate memory\n", __func__);
            return -ENOMEM;
        }

        crosslink->device = client; 
        INIT_LIST_HEAD(&crosslink->entry);
        list_add_tail(&crosslink->entry, &lvds2mipi_dev_list); 

        dev_info(&client->dev,"%s registered.\n", name);
    }
    else
        dev_info(&client->dev,"device name [%s] not compatible.\n",name);

    return 0;
}

static int generic_sd_remove(struct i2c_client *client)
{
  return 0;
}

static const struct i2c_device_id generic_sd_id[] = {
    { "i2c-mux", broadcast_mux_ch },
    { "crosslink", lvds2mpip_crosslink },
    {},
};

MODULE_DEVICE_TABLE(i2c, generic_sd_id);

static struct i2c_driver generic_sd_i2c_driver = {
  .driver = {
    .name = "framos_i2c_generic_driver",
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(generic_sd_of_match),
  },
  .probe = generic_sd_probe,
  .remove = generic_sd_remove,
  .id_table = generic_sd_id,
};

module_i2c_driver(generic_sd_i2c_driver);

MODULE_DESCRIPTION ("framos i2c generic driver");
MODULE_AUTHOR("FRAMOS GmbH");
MODULE_LICENSE ("GPL v2");
