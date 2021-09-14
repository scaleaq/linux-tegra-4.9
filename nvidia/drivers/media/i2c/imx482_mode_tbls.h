/*
 * imx482_mode_tbls.h - imx482 sensor mode tables
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

#ifndef __IMX482_TABLES__
#define __IMX482_TABLES__

/**
 * Image sensor registers as described in the IMX482 register map
 */

#define STANDBY              0x3000
#define REGHOLD              0x3001
#define XMSTA                0x3002

#define BCWAIT_TIME_LOW      0x3008
#define BCWAIT_TIME_HIGH     0x3009
#define CPWAIT_TIME_LOW      0x300A
#define CPWAIT_TIME_HIGH_EFWAIT_TIME     0x300B

#define WINMODE              0x301C
#define CFMODE               0x301D

#define HADD                 0x3020
#define VADD                 0x3021
#define ADDMODE              0x3022

#define VMAX_LOW             0x3024
#define VMAX_MID             0x3025
#define VMAX_HIGH            0x3026
#define HMAX_LOW             0x3028
#define HMAX_HIGH            0x3029

#define HVREVERSE            0x3030
#define ADBIT                0x3031
#define MDBIT                0x3032

#define FDG_SEL0             0x3034
#define PIX_HST_LOW          0x303C
#define PIX_HST_HIGH         0x303D
#define PIX_HWIDTH_LOW       0x303E
#define PIX_HWIDTH_HIGH      0x303F

#define PIX_XSIZE_OVRLAP_LOW    0x3040
#define PIX_XSIZE_OVRLAP_HIGH   0x3041
#define PIX_VST_LOW          0x3044
#define PIX_VST_HIGH         0x3045
#define PIX_VWIDTH_LOW       0x3046
#define PIX_VWIDTH_HIGH      0x3047

#define SHR0_LOW             0x3050
#define SHR0_MID             0x3051
#define SHR0_HIGH            0x3052

#define GAIN_LOW             0x3084
#define GAIN_HIGH            0x3085

#define XHSOUTSEL_XVSOUTSEL  0x30A4
#define XVS_XHS_DRV          0x30A5
#define XVSLNG               0x30CC
#define XHSLNG               0x30CD

#define DIG_CLP_VSTART       0x30D5
#define BLKLEVEL_LOW         0x30DC
#define BLKLEVEL_HIGH        0x30DD

#define INCKSEL1             0x3114
#define INCKSEL2             0x3119
#define INCKSEL3_LOW         0x311C
#define INCKSEL3_HIGH        0x311D

#define INCKSEL4             0x3804
#define INCKSEL5             0x3807

#define LANEMODE             0x3D01
#define TXCLKESC_FREQ_LOW    0x3D04
#define TXCLKESC_FREQ_HIGH   0x3D05
#define INCKSEL6             0x3D0C



#define TCLKPOST_LOW         0x3D18
#define TCLKPOST_HIGH        0x3D19
#define TCLKPREPARE_LOW      0x3D1A
#define TCLKPREPARE_HIGH     0x3D1B
#define TCLKTRAIL_LOW        0x3D1C
#define TCLKTRAIL_HIGH       0x3D1D
#define TCLKZERO_LOW         0x3D1E
#define TCLKZERO_HIGH        0x3D1F

#define THSPREPARE_LOW       0x3D20
#define THSPREPARE_HIGH      0x3D21
#define THSZERO_LOW          0x3D22
#define THSZERO_HIGH         0x3D23
#define THSTRAIL_LOW         0x3D24
#define THSTRAIL_HIGH        0x3D25
#define THSEXIT_LOW          0x3D26
#define THSEXIT_HIGH         0x3D27
#define TLPX_LOW             0x3D28
#define TLPX_HIGH            0x3D29

#define TPG_EN_DUOUT        0x30E0
#define TPG_PATSEL_DUOUT    0x30E2
#define TPG_COLORWIDTH_STLINE_SEL 0x30E4
#define TESTCLKEN_MIPI      0x3110
#define DIG_CLP_MODE        0x3258
#define WRJ_OPEN            0x3390

#define EXTMODE             0x30CE
#define SECOND_SLAVE_ADD    0x300C

/**
 * Special values for the write table function
 */
#define IMX482_TABLE_WAIT_MS 0
#define IMX482_TABLE_END     1
#define IMX482_WAIT_MS       10

#define IMX482_DEFAULT_WIDTH            1932
#define IMX482_DEFAULT_HEIGHT           1090
#define IMX482_CROP_1920x1080_WIDTH     1920
#define IMX482_CROP_1920x1080_HEIGHT    1080

/**
 * Minimal value of frame length is resolution height + 30
 * Minimal value for scaling modes is full pixel mode height + 30
 *
 * Determined from the default value of FRM_LENGTH_LINES register
 * and empirically confirmed
 */
#define IMX482_MIN_FRAME_LENGTH_DELTA 70

#define IMX482_TO_LOW_BYTE(x) (x & 0xFF)
#define IMX482_TO_MID_BYTE(x) (x >> 8)

typedef struct reg_8 imx482_reg;

/**
 * Tables for the write table function
 */

static const imx482_reg imx482_start[] = {

    {STANDBY,              0x00},
    {IMX482_TABLE_WAIT_MS, 30},
    {XMSTA,                0x00},
    {ADBIT,                0x00},
    {MDBIT,                0x01},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x00}
};

static const imx482_reg imx482_stop[] = {

    {XMSTA,                0x01},
    {IMX482_TABLE_WAIT_MS, 30},
    {STANDBY,              0x01},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x00}
};

static const imx482_reg imx482_1782_mbps[] = {

    {TCLKPOST_HIGH,        0x00},
    {TCLKPOST_LOW,         0xB7},
    {TCLKPREPARE_HIGH,     0x00},
    {TCLKPREPARE_LOW,      0x67},
    {TCLKTRAIL_HIGH,       0x00},
    {TCLKTRAIL_LOW,        0x6F},
    {TCLKZERO_HIGH,        0x01},
    {TCLKZERO_LOW,         0xDF},
    {THSPREPARE_HIGH,      0x00},
    {THSPREPARE_LOW,       0x6F},
    {THSZERO_HIGH,         0x00},
    {THSZERO_LOW,          0xCF},
    {THSTRAIL_HIGH,        0x00},
    {THSTRAIL_LOW,         0x6F},
    {THSEXIT_HIGH,         0x00},
    {THSEXIT_LOW,          0xB7},
    {TLPX_HIGH,            0x00},
    {TLPX_LOW,             0x5F},

    {BCWAIT_TIME_HIGH,     0x00},
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_HIGH_EFWAIT_TIME,          0x50},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x02},
    {INCKSEL2,             0x00},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0xC0},
    {INCKSEL4,             0x02},
    {INCKSEL5,             0x60},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x01},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x0000}
};

static const imx482_reg imx482_1440_mbps[] = {

    {TCLKPOST_HIGH,        0x00},
    {TCLKPOST_LOW,         0x9F},
    {TCLKPREPARE_HIGH,     0x00},
    {TCLKPREPARE_LOW,      0x57},
    {TCLKTRAIL_HIGH,       0x00},
    {TCLKTRAIL_LOW,        0x57},
    {TCLKZERO_HIGH,        0x01},
    {TCLKZERO_LOW,         0x87},
    {THSPREPARE_HIGH,      0x00},
    {THSPREPARE_LOW,       0x5F},
    {THSZERO_HIGH,         0x00},
    {THSZERO_LOW,          0xA7},
    {THSTRAIL_HIGH,        0x00},
    {THSTRAIL_LOW,         0x5F},
    {THSEXIT_HIGH,         0x00},
    {THSEXIT_LOW,          0x97},
    {TLPX_HIGH,            0x00},
    {TLPX_LOW,             0x4F},

    {BCWAIT_TIME_HIGH,     0x00},
    {BCWAIT_TIME_LOW,      0x7F},
    {CPWAIT_TIME_HIGH_EFWAIT_TIME,          0x50},
    {CPWAIT_TIME_LOW,      0x5B},
    {INCKSEL1,             0x02},
    {INCKSEL2,             0x00},
    {INCKSEL3_HIGH,        0x00},
    {INCKSEL3_LOW,         0x9B},
    {INCKSEL4,             0x02},
    {INCKSEL5,             0x60},
    {TXCLKESC_FREQ_HIGH,   0x09},
    {TXCLKESC_FREQ_LOW,    0x48},
    {INCKSEL6,             0x01},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x0000}
};

static const imx482_reg imx482_init_settings[] = {

    {WINMODE,              0x00},
    {CFMODE,               0x05},
    {ADDMODE,              0x02},
    {VMAX_MID,             0x08},
    {VMAX_LOW,             0xCA},
    {HMAX_HIGH,            0x02},
    {HMAX_LOW,             0x26},
    {HVREVERSE,            0x00},
    {ADBIT,                0x00},
    {MDBIT,                0x01},

    {LANEMODE,             0x03},

    {0x3152,    0x1E},
    {0x3154,    0xC2},
    {0x3156,    0x1C},
    {0x3160,    0xC4},
    {0x3168,    0x3A},
    {0x3169,    0x00},
    {0x317A,    0x3B},
    {0x317B,    0x00},

    {0x3260,	0x22},
    {0x3262,	0x02},
    {0x3278,	0xA2},

    {0x3324,	0x00},
    {0x3366,	0x31},

    {0x340C,	0x4D},
    {0x3416,	0x10},
    {0x3417,	0x13},
    {0x3432,	0x93},
    {0x34CE,	0x1E},
    {0x34CF,	0x1E},
    {0x34DC,	0x80},

    {0x351C,	0x03},
    {0x359E,	0x70},
    {0x35A2,	0x9C},
    {0x35AC,	0x08},
    {0x35C0,	0xFA},
    {0x35C2,	0x4E},
    {0x35DC,    0x05},
    {0x35DE,    0x05},

    {0x3608,	0x41},
    {0x360A,	0x47},
    {0x361E,	0x4A},
    {0x3630,	0x43},
    {0x3632,	0x47},
    {0x363C,	0x41},
    {0x363E,	0x4A},
    {0x3648,	0x41},
    {0x364A,	0x47},
    {0x3660,	0x04},
    {0x3676,	0x3F},
    {0x367A,	0x3F},
    {0x36A4,	0x41},

    {0x3798,	0x8C},
    {0x379A,	0x8C},
    {0x379C,	0x8C},
    {0x379E,	0x8C},

    {0x3888,	0xA8},
    {0x388C,	0xA6},

    {0x3914,	0x15},
    {0x3915,	0x15},
    {0x3916,	0x15},
    {0x3917,	0x14},
    {0x3918,	0x14},
    {0x3919,	0x14},
    {0x391A,	0x13},
    {0x391B,	0x13},
    {0x391C,	0x13},
    {0x391E,	0x00},
    {0x391F,	0xA5},
    {0x3920,	0xDE},
    {0x3921,	0x0E},
    {0x39A2,	0x0C},
    {0x39A4,	0x16},
    {0x39A6,	0x2B},
    {0x39A7,	0x01},
    {0x39D2,	0x2D},
    {0x39D3,	0x00},
    {0x39D8,	0x37},
    {0x39D9,	0x00},
    {0x39DA,	0x9B},
    {0x39DB,	0x01},
    {0x39E0,	0x28},
    {0x39E1,	0x00},
    {0x39E2,	0x2C},
    {0x39E3,	0x00},
    {0x39E8,	0x96},
    {0x39EA,	0x9A},
    {0x39EB,	0x01},
    {0x39F2,	0x27},
    {0x39F3,	0x00},

    {0x3A00,	0x38},
    {0x3A01,	0x00},
    {0x3A02,	0x95},
    {0x3A03,	0x01},
    {0x3A18,	0x9B},
    {0x3A2A,	0x0C},
    {0x3A30,	0x15},
    {0x3A32,	0x31},
    {0x3A33,	0x01},
    {0x3A36,	0x4D},
    {0x3A3E,	0x11},
    {0x3A40,	0x31},
    {0x3A42,	0x4C},
    {0x3A43,	0x01},
    {0x3A44,	0x47},
    {0x3A46,	0x4B},
    {0x3A4E,	0x11},
    {0x3A50,	0x32},
    {0x3A52,	0x46},
    {0x3A53,	0x01},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x0000}
};

static const imx482_reg mode_1932x1090[] = {

    {WINMODE,              0x00},
    {HADD,                 0x01},
    {VADD,                 0x01},
    {ADDMODE,              0x02},
    {DIG_CLP_VSTART,       0x02},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x0000}
};

static const imx482_reg mode_crop_1920x1080[] = {

    {WINMODE,              0x04},
    {HADD,                 0x01},
    {VADD,                 0x01},
    {ADDMODE,              0x02},
    {DIG_CLP_VSTART,       0x02},

    {PIX_HST_HIGH,          IMX482_TO_MID_BYTE(6)},
    {PIX_HST_LOW,           IMX482_TO_LOW_BYTE(6)},
    {PIX_HWIDTH_HIGH,       IMX482_TO_MID_BYTE(IMX482_CROP_1920x1080_WIDTH)},
    {PIX_HWIDTH_LOW,        IMX482_TO_LOW_BYTE(IMX482_CROP_1920x1080_WIDTH)},

    {PIX_VST_HIGH,          IMX482_TO_MID_BYTE(4)},
    {PIX_VST_LOW,           IMX482_TO_LOW_BYTE(4)},
    {PIX_VWIDTH_HIGH,       IMX482_TO_MID_BYTE(IMX482_CROP_1920x1080_HEIGHT)},
    {PIX_VWIDTH_LOW,        IMX482_TO_LOW_BYTE(IMX482_CROP_1920x1080_HEIGHT)},

    {IMX482_TABLE_WAIT_MS,  IMX482_WAIT_MS},
    {IMX482_TABLE_END,      0x0000}
};

static const imx482_reg mode_enable_pattern_generator[] = {
    {CFMODE,               0x00},
    {ADDMODE,              0x01},
    {BLKLEVEL_LOW,         0x00},
    {TPG_EN_DUOUT,         0x01},
    {TPG_COLORWIDTH_STLINE_SEL,         0x00},
    {TESTCLKEN_MIPI,       0x20},
    {DIG_CLP_MODE,         0x00},
    {WRJ_OPEN,             0x00},
    {TPG_PATSEL_DUOUT,     0x0A},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x0000}
};

static const imx482_reg mode_disable_pattern_generator[] = {
    {CFMODE,               0x05},
    {BLKLEVEL_LOW,         0x32},
    {TPG_EN_DUOUT,         0x00},
    {TPG_COLORWIDTH_STLINE_SEL,         0x10},
    {TESTCLKEN_MIPI,       0x00},
    {DIG_CLP_MODE,         0x01},
    {WRJ_OPEN,             0x01},

    {IMX482_TABLE_WAIT_MS, IMX482_WAIT_MS},
    {IMX482_TABLE_END,     0x0000}
};

/**
 * Enum of available frame modes
 */

enum {
    IMX482_MODE_1932x1090,
    IMX482_MODE_CROP_1920x1080,

    IMX482_EN_PATTERN_GEN,
    IMX482_DIS_PATTERN_GEN,

    IMX482_INIT_SETTINGS,
    IMX482_MODE_START_STREAM,
    IMX482_MODE_STOP_STREAM,
};

typedef enum {
    IMX482_1782_MBPS,
    IMX482_1440_MBPS,
} data_rate_mode;

static const imx482_reg *data_rate_table[] = {
    [IMX482_1782_MBPS] = imx482_1782_mbps,
    [IMX482_1440_MBPS] = imx482_1440_mbps,
};

/**
 * Connecting frame modes to mode tables
 */

static const imx482_reg *mode_table[] = {

    [IMX482_MODE_1932x1090]      = mode_1932x1090,
    [IMX482_MODE_CROP_1920x1080] = mode_crop_1920x1080,

    [IMX482_EN_PATTERN_GEN]      = mode_enable_pattern_generator,
    [IMX482_DIS_PATTERN_GEN]     = mode_disable_pattern_generator,

    [IMX482_INIT_SETTINGS]       = imx482_init_settings,

    [IMX482_MODE_START_STREAM]   = imx482_start,
    [IMX482_MODE_STOP_STREAM]    = imx482_stop,
};

/**
 * Framerates of available frame modes
 */

static const int imx482_72fps[] = {
    72,
};
static const int imx482_90fps[] = {
    90,
};
static const int imx482_142fps[] = {
    142,
};

/**
 * Connecting resolutions, framerates and mode tables
 */
static const struct camera_common_frmfmt imx482_frmfmt[] = {
    {
        .size = {IMX482_DEFAULT_WIDTH, IMX482_DEFAULT_HEIGHT},
        .framerates = imx482_72fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX482_MODE_1932x1090
    },
    {
        .size = {IMX482_CROP_1920x1080_WIDTH, IMX482_CROP_1920x1080_HEIGHT},
        .framerates = imx482_142fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX482_MODE_CROP_1920x1080
    }
};

#endif /* __IMX482_TABLES__ */
