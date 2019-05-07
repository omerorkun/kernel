/*
 * Driver for AR0330 CMOS Image Sensor from Aptina
 *
 * Copyright (C) 2014, Julien Beraud <julien.beraud@parrot.com>
 * Copyright (C) 2011, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2011, Javier Martin <javier.martin@vista-silicon.com>
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on the MT9V032 driver and Bastian Hecht's code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/gcd.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

struct clk;
struct v4l2_subdev;
enum {
	AR0330_HW_BUS_PARALLEL,
	AR0330_HW_BUS_MIPI,
	AR0330_HW_BUS_HISPI,
};
struct ar0330_platform_data {
	struct clk     *clock;
	u32		clk_rate;
	u32		max_vco_rate;
	u32		max_op_clk_rate;
	int (*set_power)(int on);
	unsigned int	reset;
	u32		hw_bus;
};

#define AR0330_POWER_ON 1
#define ARO330_POWER_OFF 0
/* clk limits */
#define AR0330_MIN_EXT_CLK_HZ				6000000
#define AR0330_MAX_EXT_CLK_HZ				27000000
#define AR0330_MIN_VCO_CLK_HZ				384000000
#define AR0330_MAX_VCO_CLK_HZ				768000000
#define AR0330_MIN_PLL_MULT				32
#define AR0330_MAX_PLL_MULT				255
#define AR0330_MAX_PRE_PLL_CLK_DIV			64
/* pix array limits */
#define AR0330_PIXEL_ARRAY_WIDTH			2316
#define AR0330_PIXEL_ARRAY_HEIGHT			1555
#define AR0330_WINDOW_WIDTH_MIN				32
#define AR0330_WINDOW_WIDTH_DEF				2304
#define AR0330_WINDOW_WIDTH_MAX				2304
#define AR0330_WINDOW_HEIGHT_MIN			32
#define AR0330_WINDOW_HEIGHT_DEF			1544
#define AR0330_WINDOW_HEIGHT_MAX			1544
/* AR0330 Registers */
#define AR0330_CHIP_VERSION				0x3000
#define		AR0330_CHIP_VERSION_VALUE		0x2604
#define AR0330_Y_ADDR_START				0x3002
#define 	AR0330_Y_ADDR_START_MIN			6
#define AR0330_X_ADDR_START				0x3004
#define AR0330_X_ADDR_START_MIN				6
#define AR0330_Y_ADDR_END				0x3006
#define AR0330_Y_ADDR_END_MAX				1549
#define AR0330_X_ADDR_END				0x3008
#define AR0330_X_ADDR_END_MAX				2309
#define AR0330_FRAME_LENGTH_LINES			0x300a
#define AR0330_LINE_LENGTH_PCK				0x300c
#define AR0330_CHIP_REVISION				0x300e
#define		AR0330_CHIP_REVISION_1x			0x10
#define		AR0330_CHIP_REVISION_2x			0x20
#define AR0330_LOCK_CONTROL				0x3010
#define		AR0330_LOCK_CONTROL_UNLOCK		0xbeef
#define AR0330_COARSE_INTEGRATION_TIME			0x3012
#define AR0330_RESET					0x301a
#define		AR0330_RESET_SMIA_DIS			(1 << 12)
#define		AR0330_RESET_FORCE_PLL_ON		(1 << 11)
#define		AR0330_RESET_RESTART_BAD		(1 << 10)
#define		AR0330_RESET_MASK_BAD			(1 << 9)
#define		AR0330_RESET_GPI_EN			(1 << 8)
#define		AR0330_RESET_PARALLEL_EN		(1 << 7)
#define		AR0330_RESET_DRIVE_PINS			(1 << 6)
#define		AR0330_RESET_LOCK_REG			(1 << 3)
#define		AR0330_RESET_STREAM			(1 << 2)
#define		AR0330_RESET_RESTART			(1 << 1)
#define		AR0330_RESET_RESET			(1 << 0)
/* AR03303_MODE_SELECT is an alias for AR0330_RESET_STREAM */
#define AR0330_MODE_SELECT				0x301c
#define		AR0330_MODE_SELECT_STREAM		(1 << 0)
#define AR0330_VT_PIX_CLK_DIV				0x302a
#define AR0330_VT_SYS_CLK_DIV				0x302c
#define AR0330_PRE_PLL_CLK_DIV				0x302e
#define AR0330_PLL_MULTIPLIER				0x3030
#define AR0330_OP_PIX_CLK_DIV				0x3036
#define AR0330_OP_SYS_CLK_DIV				0x3038
#define AR0330_FRAME_COUNT				0x303a
#define AR0330_READ_MODE				0x3040
#define		AR0330_READ_MODE_VERT_FLIP		(1 << 15)
#define		AR0330_READ_MODE_HORIZ_MIRROR		(1 << 14)
#define		AR0330_READ_MODE_COL_BIN		(1 << 13)
#define		AR0330_READ_MODE_ROW_BIN		(1 << 12)
#define		AR0330_READ_MODE_COL_SF_BIN		(1 << 9)
#define		AR0330_READ_MODE_COL_SUM		(1 << 5)
#define	AR0330_EXTRA_DELAY				0x3042
#define AR0330_GREEN1_GAIN				0x3056
#define AR0330_BLUE_GAIN				0x3058
#define AR0330_RED_GAIN					0x305a
#define AR0330_GREEN2_GAIN				0x305c
#define AR0330_GLOBAL_GAIN				0x305e
#define		AR0330_GLOBAL_GAIN_MIN			0
#define		AR0330_GLOBAL_GAIN_DEF			1000
#define		AR0330_GLOBAL_GAIN_MAX			15992
#define AR0330_ANALOG_GAIN				0x3060
#define AR0330_SMIA_TEST				0x3064
#define         AR0330_EMBEDDED_DATA			(1 << 8)
#define AR0330_DATAPATH_SELECT				0x306e
#define		AR0330_DATAPATH_SLEW_DOUT_MASK		(7 << 13)
#define		AR0330_DATAPATH_SLEW_DOUT_SHIFT		13
#define		AR0330_DATAPATH_SLEW_PCLK_MASK		(7 << 10)
#define		AR0330_DATAPATH_SLEW_PCLK_SHIFT		10
#define		AR0330_DATAPATH_HIGH_VCM		(1 << 9)
#define AR0330_TEST_PATTERN_MODE			0x3070
#define AR0330_TEST_DATA_RED				0x3072
#define AR0330_TEST_DATA_GREENR				0x3074
#define AR0330_TEST_DATA_BLUE				0x3076
#define AR0330_TEST_DATA_GREENB				0x3078
#define AR0330_SEQ_DATA_PORT				0x3086
#define AR0330_SEQ_CTRL_PORT				0x3088
#define AR0330_X_ODD_INC				0x30a2
#define AR0330_Y_ODD_INC				0x30a6
#define AR0330_X_ODD_INC				0x30a2
#define AR0330_DIGITAL_CTRL				0x30ba
#define		AR0330_DITHER_ENABLE			(1 << 5)
/* Note from Developer Guide Version E :
 * The original AR0330 Rev2.0 samples did not have R0x300E
 * programmed correctly
 * Therefore reading register 0x30F0 is the only sure method
 * to get chip revision : 0x1200 indicates Rev1 while 0x1208
 * indicates Rev2.X
 */
#define AR0330_RESERVED_CHIPREV				0x30F0
#define AR0330_DATA_FORMAT_BITS				0x31ac
#define AR0330_SERIAL_FORMAT				0x31ae
#define AR0330_FRAME_PREAMBLE				0x31b0
#define AR0330_LINE_PREAMBLE				0x31b2
#define AR0330_MIPI_TIMING_0				0x31b4
#define AR0330_MIPI_TIMING_1				0x31b6
#define AR0330_MIPI_TIMING_2				0x31b8
#define AR0330_MIPI_TIMING_3				0x31ba
#define AR0330_MIPI_TIMING_4				0x31bc
#define AR0330_MIPI_CONFIG_STATUS			0x31be
#define AR0330_COMPRESSION				0x31d0
#define		AR0330_COMPRESSION_ENABLE		(1 << 0)
#define AR0330_POLY_SC					0x3780
#define		AR0330_POLY_SC_ENABLE			(1 << 15)
struct ar0330_pll {
	u16 pre_pll_clk_div;
	u16 pll_multiplier;
	u16 vt_sys_clk_div;
	u16 vt_pix_clk_div;
	u16 op_sys_clk_div;
	u16 op_pix_clk_div;
	u32 clk_pix;
};
struct ar0330 {
	struct ar0330_platform_data	*pdata;
	struct ar0330_pll 		pll;
	unsigned int 			version;
	struct v4l2_subdev 		subdev;
	struct media_pad 		pad;
	/* Sensor window */
	struct v4l2_rect 		crop;
	struct v4l2_rect 		video_timing;
	u32 				x_binning;
	u32 				y_binning;
	struct v4l2_mbus_framefmt 	format;
	struct v4l2_fract		frame_interval;
	bool				streaming;
	struct v4l2_ctrl_handler 	ctrls;
	struct v4l2_ctrl 		*flip[2];
	struct v4l2_ctrl 		*pixel_rate;
	struct v4l2_ctrl 		*exposure;
	struct v4l2_ctrl		*gains[4];
	/* lock to protect power_count */
	struct mutex 			power_lock;
	int 				power_count;
	/* Registers cache */
	u16 				read_mode;
};
//static struct ar0330 *to_ar0330(struct v4l2_subdev *sd)
//{
//	return container_of(sd, struct ar0330, subdev);
//}
///* -----------------------------------------------------------------------------
// * Register access
// */
//static int __ar0330_read(struct i2c_client *client, u16 reg, size_t size)
//{
//	u8 data[2];
//	struct i2c_msg msg[2] = {
//		{ client->addr, 0, 2, data },
//		{ client->addr, I2C_M_RD, size, data },
//	};
//	int ret;
//	data[0] = reg >> 8;
//	data[1] = reg & 0xff;
//	ret = i2c_transfer(client->adapter, msg, 2);
//	if (ret < 0) {
//		dev_err(&client->dev, "%s(0x%04x) failed (%d)\n", __func__,
//			reg, ret);
//		return ret;
//	}
//	if (size == 2)
//		return (data[0] << 8) | data[1];
//	else
//		return data[0];
//}
//static int __ar0330_write(struct i2c_client *client, u16 reg, u16 value,
//			  size_t size)
//{
//	u8 data[4];
//	struct i2c_msg msg = { client->addr, 0, 2 + size, data };
//	int ret;
//	v4l2_info(client, "writing 0x%04x to 0x%04x\n", value, reg);
//	data[0] = reg >> 8;
//	data[1] = reg & 0xff;
//	if (size == 2) {
//		data[2] = value >> 8;
//		data[3] = value & 0xff;
//	} else {
//		data[2] = value & 0xff;
//	}
//	ret = i2c_transfer(client->adapter, &msg, 1);
//	if (ret < 0) {
//		dev_err(&client->dev, "%s(0x%04x) failed (%d)\n", __func__,
//			reg, ret);
//		return ret;
//	}
//	return 0;
//}
//static inline int ar0330_read8(struct i2c_client *client, u16 reg)
//{
//	return __ar0330_read(client, reg, 1);
//}
//static inline int ar0330_write8(struct i2c_client *client, u16 reg, u8 value)
//{
//	return __ar0330_write(client, reg, value, 1);
//}
//static inline int ar0330_read16(struct i2c_client *client, u16 reg)
//{
//	return __ar0330_read(client, reg, 2);
//}
//static inline int ar0330_write16(struct i2c_client *client, u16 reg, u16 value)
//{
//	return __ar0330_write(client, reg, value, 2);
//}
//static inline int ar0330_set16(struct i2c_client *client, u16 reg, u16 value,
//			       u16 mask)
//{
//	int rval = ar0330_read16(client, reg);
//	if (rval < 0)
//		return rval;
//	else
//		return ar0330_write16(client, reg,
//				     (rval & ~mask) | (value & mask));
//}
static struct v4l2_subdev_video_ops ar0330_subdev_video_ops = {
	.s_stream       = ar0330_s_stream,
/*	.g_frame_interval = ar0330_g_frame_interval,
	.s_frame_interval = ar0330_s_frame_interval,*/
};
static struct v4l2_subdev_pad_ops ar0330_subdev_pad_ops = {
	.enum_mbus_code = ar0330_enum_mbus_code,
	.enum_frame_size = ar0330_enum_frame_size,
	.get_fmt = ar0330_get_format,
	.set_fmt = ar0330_set_format,
};
static struct v4l2_subdev_ops ar0330_subdev_ops = {
	.video  = &ar0330_subdev_video_ops,
	.pad    = &ar0330_subdev_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */
static int ar0330_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	//printk("Wellcome to ar0330 driver test");
	struct device *dev = &client->dev;
//	struct ar0330_platform_data *pdata = client->dev.platform_data;
	struct ar0330 *ar0330;
	struct v4l2_subdev *sd;
//	unsigned int i;
	int ret;
	printk("Wellcome to ar0330 driver test\n");
	if(dev == NULL)
	{
		printk("ar0330 dev is null\n");
	}
	ar0330 = devm_kzalloc(dev, sizeof(*ar0330), GFP_KERNEL);
	/*if (pdata == NULL) {
		dev_err(&client->dev, "No platform data\n");
		return -EINVAL;
	}*/
//	ar0330 = kzalloc(sizeof(*ar0330), GFP_KERNEL);
	if (ar0330 == NULL)
		return -ENOMEM;
	printk("ar0330 probe db-0\n");
	mutex_init(&ar0330->power_lock);

	sd = &ar0330->subdev;
	v4l2_i2c_subdev_init(sd, client, &ar0330_subdev_ops);
//	ar0330->pdata = pdata;
//	ar0330->read_mode = 0;
//	v4l2_ctrl_handler_init(&ar0330->ctrls,
//			ARRAY_SIZE(ar0330_gains) +
//			ARRAY_SIZE(ar0330_ctrls) +
//			5);
//	/* Exposure in us */
//	ar0330->exposure = v4l2_ctrl_new_std(&ar0330->ctrls,
//					       &ar0330_ctrl_ops,
//					       V4L2_CID_EXPOSURE_ABSOLUTE,
//					       0, 65535, 1, 30000);
//	v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
//			  V4L2_CID_GAIN, AR0330_GLOBAL_GAIN_MIN,
//			  AR0330_GLOBAL_GAIN_MAX, 1, AR0330_GLOBAL_GAIN_DEF);
//	ar0330->flip[0] = v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
//					    V4L2_CID_HFLIP, 0, 1, 1, 0);
//	ar0330->flip[1] = v4l2_ctrl_new_std(&ar0330->ctrls, &ar0330_ctrl_ops,
//					    V4L2_CID_VFLIP, 0, 1, 1, 0);
//	for (i = 0; i < ARRAY_SIZE(ar0330_gains); ++i)
//		ar0330->gains[i] = v4l2_ctrl_new_custom(&ar0330->ctrls,
//			&ar0330_gains[i], NULL);
//	v4l2_ctrl_new_std(&ar0330->ctrls,
//			&ar0330_ctrl_ops,
//			V4L2_CID_ANALOGUE_GAIN,
//			100, 800, 1, 100);
//	for (i = 0; i < ARRAY_SIZE(ar0330_ctrls); ++i)
//		v4l2_ctrl_new_custom(&ar0330->ctrls, &ar0330_ctrls[i], NULL);
//	v4l2_ctrl_cluster(ARRAY_SIZE(ar0330->flip), ar0330->flip);
//	if (ar0330->ctrls.error)
//		v4l2_err(client, "%s: control initialization error %d\n",
//		       __func__, ar0330->ctrls.error);
//	ar0330->subdev.ctrl_handler = &ar0330->ctrls;
//	mutex_init(&ar0330->power_lock);
//	v4l2_i2c_subdev_init(&ar0330->subdev, client, &ar0330_subdev_ops);
//	ar0330->subdev.internal_ops = &ar0330_subdev_internal_ops;
//	ar0330->pad.flags = MEDIA_PAD_FL_SOURCE;
//	ret = media_entity_init(&ar0330->subdev.entity, 1, &ar0330->pad, 0);
//	if (ret < 0)
//		goto done;
//	ar0330->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
//	ar0330->crop.width = AR0330_WINDOW_WIDTH_DEF;
//	ar0330->crop.height = AR0330_WINDOW_HEIGHT_DEF;
//	ar0330->crop.left = (AR0330_WINDOW_WIDTH_MAX - AR0330_WINDOW_WIDTH_DEF)
//			  / 2;
//	ar0330->crop.top = (AR0330_WINDOW_HEIGHT_MAX - AR0330_WINDOW_HEIGHT_DEF)
//			 / 2;
//	ar0330->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
//	ar0330->format.width = AR0330_WINDOW_WIDTH_DEF;
//	ar0330->format.height = AR0330_WINDOW_HEIGHT_DEF;
//	ar0330->format.field = V4L2_FIELD_NONE;
//	/* 30 FPS */
//	ar0330->frame_interval.numerator =  1;
//	ar0330->frame_interval.denominator = 30;
//	ar0330_calc_vt(&ar0330->subdev);
//	ret = ar0330_pll_init(ar0330);
//done:
//	if (ret < 0) {ge
//		v4l2_ctrl_handler_free(&ar0330->ctrls);
//		media_entity_cleanup(&ar0330->subdev.entity);
//		kfree(ar0330);
//	}
	return ret;
	return 0;
}
static int ar0330_remove(struct i2c_client *client)
{
/*	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ar0330 *ar0330 = to_ar0330(subdev);
	v4l2_ctrl_handler_free(&ar0330->ctrls);
	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	kfree(ar0330);*/
	return 0;
}
static const struct i2c_device_id ar0330_id[] = {
	{ "ar0330", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar0330_id);
static struct i2c_driver ar0330_i2c_driver = {
	.driver = {
		.name = "ar0330",
	},
	.probe          = ar0330_probe,
	.remove         = ar0330_remove,
	.id_table       = ar0330_id,
};
static int __init ar0330_mod_init(void)
{
	return i2c_add_driver(&ar0330_i2c_driver);
}
static void __exit ar0330_mod_exit(void)
{
	i2c_del_driver(&ar0330_i2c_driver);
}
module_init(ar0330_mod_init);
module_exit(ar0330_mod_exit);
MODULE_DESCRIPTION("Aptina AR0330 Camera driver");
MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_AUTHOR("Julien Beraud <julien.beraud@parrot.com>");
MODULE_AUTHOR("Omer Orkun Duztas <omerorkun@buyutech.com.tr>");
MODULE_LICENSE("GPL v2");
