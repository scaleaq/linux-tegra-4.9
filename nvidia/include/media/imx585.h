/*
 * imx585_.h - imx585 sensor header
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

#ifndef __IMX585_H__
#define __IMX585_H__

/* imx585 - sensor parameters */
#define IMX585_MIN_GAIN		                (0)
#define IMX585_MAX_GAIN_DEC                     (240)
#define IMX585_MAX_GAIN_DB                      (72)
#define IMX585_ANALOG_GAIN_C0                   (1024)
#define IMX585_SHIFT_8_BITS			(8)
#define IMX585_MIN_FRAME_LENGTH		        (256)
#define IMX585_MAX_FRAME_LENGTH		        (2097151)
#define IMX585_MIN_COARSE_EXPOSURE	        (1)
#define IMX585_MAX_COARSE_DIFF		        (10)
#define IMX585_MASK_LSB_2_BITS			0x0003
#define IMX585_MASK_LSB_8_BITS			0x00ff

#endif /* __IMX585_H__ */
