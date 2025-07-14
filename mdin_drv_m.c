/*
 * Copyright (C) 2024 yoonseok seo
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

 
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/of_gpio.h>  
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/property.h>
#include <linux/module.h>


#ifndef SIGNED_CHAR_S8
#define SIGNED_CHAR_S8 1
#if defined(SIGNED_CHAR_S8)
	typedef 	char 			__signed_char8;
	typedef 	__signed_char8 	sc8;
#endif
#endif


#define APPL_USE_PWR  	1
#define ERROR_EXEC  	0  //  execute  error 


#define MDIN_NUM_MODES 	1
#define MDIN_MODE  		0

#if 0
enum mdin_mode_id {
	MDIN_MODE = 0,
 MDIN_NUM_MODES,
};
#endif

#define MDIN_NUM_FRAMERATES 	1


typedef enum  {
	MDIN_FPS=0,
} mdin_frame_rate;

#if 0
enum mdin_format_mux {
	MDIN_FMT_MUX_YUV422 = 0,
	MDIN_FMT_MUX_RGB,
	MDIN_FMT_MUX_DITHER,
	MDIN_FMT_MUX_RAW_DPC,
	MDIN_FMT_MUX_SNR_RAW,
	MDIN_FMT_MUX_RAW_CIP,
};
#endif

struct mdin_pixfmt {
	u32 code;
	u32 colorspace;
};


/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */
 #if 0
static unsigned int virtual_channel;
module_param(virtual_channel, uint, 0444);
MODULE_PARM_DESC(virtual_channel,
		 "MIPI CSI-2 virtual channel (0..3), default 0");
#endif

static const s32 mdin_framerates1280_960[1] = {
	[MDIN_FPS] = 60,
};


#if 0
#define MDIN_NUM_SUPPLIES ARRAY_SIZE(mdin_supply_name)
#endif

#if 0
enum mdin_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};
#endif

struct mdin_mode_info {
	s32 id;
#if 0	
	enum mdin_downsize_mode dn_mode;
#endif
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
};


struct mdin_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *light_freq;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

struct mdin_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to eocam */
	u32 xclk_freq;

#if 0
	struct regulator_bulk_data supplies[MDIN_NUM_SUPPLIES];
#endif
#if  APPL_USE_PWR 	
#if 0
	s32 pwr_gpio;
#endif	
#endif
	s32   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	s32 power_count;

	struct v4l2_mbus_framefmt fmt;
	// u32 pending_fmt_change; //

	const struct mdin_mode_info *current_mode;
	const struct mdin_mode_info *last_mode;
	mdin_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct mdin_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	s32 streaming;
};

#if APPL_USE_PWR
	static u32 mdin_g_pwr_gpio = 0;
	static struct mdin_dev *g_sensor = NULL;
#endif

static const struct mdin_mode_info mdin_mode_data1280_960[MDIN_NUM_MODES] = {
	{MDIN_MODE, 
#if 0		
		SUBSAMPLING,
#endif
	 1280, 1892, 960, 1080,
	},
};


static s32 mdin_framerates[1];
static struct mdin_mode_info mdin_mode_data[MDIN_NUM_MODES];

/////////////////////////////////////////////////////////////////////////////////

#if 0
extern void gpio_set_value(u32 gpio, s32 value);
extern s32 gpio_get_value(u32 gpio);
extern s32 gpio_is_valid(u32 number);
extern s32 gpio_request_one(u32 gpio, u64 flags, const s8* label);
extern void gpio_free(u32 gpio);
#endif


typedef s32 (*func_type_subdev)(struct v4l2_subdev *sd, s32 on);
typedef s32 (*func_type_getfmt)(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format);
typedef s32 (*func_type_setfmt)(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format);
typedef int (*func_type_ctrl)(struct v4l2_ctrl *ctrl);
typedef s32 (*func_type_framesize)(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_frame_size_enum *fse);
typedef s32 (*func_type_frameinterval)(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_frame_interval_enum *fie);
typedef s32 (*func_type_interval)(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi);
typedef s32 (*func_type_linksetup)(struct media_entity *entity, const struct media_pad *local, const struct media_pad *remote, u32 flags);
typedef ssize_t (*func_type_showmdin)(struct device *dev, struct device_attribute *attr, sc8* buf);
typedef ssize_t (*func_type_storemdin)(struct device *dev, struct device_attribute *attr, const sc8* buf, size_t size);


/////////////////////////////////////////////////////////////////////////////////


static inline struct mdin_dev *to_mdin_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mdin_dev, sd);      /* MISRA_C_2012_18_04 false_alarm */
                                                     /* MISRA_C_2012_01_02 false_alarm */ 
                                                     /* MISRA_C_2012_11_05 false_alarm */ 
                                                     /* MISRA_C_2012_11_09 false_alarm */ 
}
#if 0
static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct mdin_dev,
			     ctrls.handler)->sd;
}
#endif

/*
 * After trying the various combinations, reading various
 * documentations spread around the net, and from the various
 * feedback, the clock tree is probably as follows:
 *
 *   +--------------+
 *   |  Ext. Clock  |
 *   +-+------------+
 *     |  +----------+
 *     +->|   PLL1   | - reg 0x3036, for the multiplier
 *        +-+--------+ - reg 0x3037, bits 0-3 for the pre-divider
 *          |  +--------------+
 *          +->| System Clock |  - reg 0x3035, bits 4-7
 *             +-+------------+
 *               |  +--------------+
 *               +->| MIPI Divider | - reg 0x3035, bits 0-3
 *               |  +-+------------+
 *               |    +----------------> MIPI SCLK
 *               |    +  +-----+
 *               |    +->| / 2 |-------> MIPI BIT CLK
 *               |       +-----+
 *               |  +--------------+
 *               +->| PLL Root Div | - reg 0x3037, bit 4
 *                  +-+------------+
 *                    |  +---------+
 *                    +->| Bit Div | - reg 0x3035, bits 0-3
 *                       +-+-------+
 *                         |  +-------------+
 *                         +->| SCLK Div    | - reg 0x3108, bits 0-1
 *                         |  +-+-----------+
 *                         |    +---------------> SCLK
 *                         |  +-------------+
 *                         +->| SCLK 2X Div | - reg 0x3108, bits 2-3
 *                         |  +-+-----------+
 *                         |    +---------------> SCLK 2X
 *                         |  +-------------+
 *                         +->| PCLK Div    | - reg 0x3108, bits 4-5
 *                            ++------------+
 *                             +  +-----------+
 *                             +->|   P_DIV   | - reg 0x3035, bits 0-3
 *                                +-----+-----+
 *                                       +------------> PCLK
 *
 * This is deviating from the datasheet at least for the register
 * 0x3108, since it's said here that the PCLK would be clocked from
 * the PLL.
 *
 * There seems to be also (unverified) constraints:
 *  - the PLL pre-divider output rate should be in the 4-27MHz range
 *  - the PLL multiplier output rate should be in the 500-1000MHz range
 *  - PCLK >= SCLK * 2 in YUV, >= SCLK in Raw or JPEG
 *
 * In the two latter cases, these constraints are met since our
 * factors are hardcoded. If we were to change that, we would need to
 * take this into account. The only varying parts are the PLL
 * multiplier and the system clock divider, which are shared between
 * all these clocks so won't cause any issue.
 */

/*
 * This is supposed to be ranging from 1 to 8, but the value is always
 * set to 3 in the vendor kernels.
 */

#if 0
static s32 mdin_check_valid_mode(struct mdin_dev *sensor,
				   const struct mdin_mode_info *mode,
				   enum mdin_frame_rate rate)
{
	return 0;
}

static s32 mdin_set_stream_mipi(struct mdin_dev *sensor, bool on)
{
	return 0;
}

#endif

#if 0
static const struct mdin_mode_info *
mdin_find_mode(struct mdin_dev *sensor, enum mdin_frame_rate fr,
		 s32 width, s32 height, bool nearest)
#endif
static const struct mdin_mode_info *mdin_find_mode(s32 width, s32 height)
{
	const struct mdin_mode_info *mode;
#if 1
	mode = v4l2_find_nearest_size(mdin_mode_data, 1, hact, vact, width, height);  /* MISRA_C_2012_11_09 false_alarm */
#else
	mode = v4l2_find_nearest_size(mdin_mode_data, 1, ARRAY_SIZE(mdin_mode_data), hact, vact, width, height);
#endif
					
#if ERROR_EXEC
	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height)))
		return NULL;
#endif
	return mode;
}


/* --------------- Subdev Operations --------------- */

static s32 mdin_s_power(const struct v4l2_subdev *sd, s32 on)
{
	(void)(sd);
   pr_err("%s ==> %d\n", __func__, on);

   return 0;
}


static s32 mdin_get_fmt(struct v4l2_subdev *sd,
			  const struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	(void)(cfg);
	struct mdin_dev *sensor = to_mdin_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

#if ERROR_EXEC
	if (format->pad != 0)
		return -EINVAL;
#endif
	mutex_lock(&sensor->lock);

	fmt = &sensor->fmt;

	fmt->reserved[1] = 120U; //(sensor->current_fr == MDIN_FPS) ? 120U : 120U; //
	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

#if 0
static s32 mdin_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum mdin_frame_rate fr,
				   const struct mdin_mode_info **new_mode)
#endif

static s32 mdin_try_fmt_internal(const struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   const struct mdin_mode_info **new_mode)
{
#if 0
	struct mdin_dev *sensor = to_mdin_dev(sd);
#endif
	(void)(sd);
	const struct mdin_mode_info *mode;
  s32 i;

static const struct mdin_pixfmt mdin_formats[] = {
	{ MEDIA_BUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG, },
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_RGB565_2X8_BE, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_COLORSPACE_SRGB, },
};

	mode = mdin_find_mode((s32)fmt->width, (s32)fmt->height);
#if 0
	mode = mdin_find_mode(sensor, fr, fmt->width, fmt->height, true);
#endif
	
#if 	ERROR_EXEC
	if (!mode)
		return -EINVAL;
#endif		
	fmt->width = mode->hact;
	fmt->height = mode->vact;
	memset(fmt->reserved, 0, sizeof(fmt->reserved)); /* MISRA_C_2012_08_04 false_alarm */
	                                                 /* MISRA_C_2012_17_03 false_alarm */

#if 0
	if (new_mode)
#endif	
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(mdin_formats); i++)
	{
		if (mdin_formats[i].code == fmt->code)
			{
			break;
		 }
	}	 
#if ERROR_EXEC			
	if (i >= ARRAY_SIZE(mdin_formats))
		i = 0;
#endif

	fmt->code = mdin_formats[i].code;
	fmt->colorspace = mdin_formats[i].colorspace;
	fmt->ycbcr_enc = (u16)V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = (u16)V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = (u16)V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static s32 mdin_set_fmt(struct v4l2_subdev *sd,
			  const struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	(void)(cfg);
	struct mdin_dev *sensor = to_mdin_dev(sd);
	const struct mdin_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	s32 ret;
#if ERROR_EXEC
	if (format->pad != 0)
		return -EINVAL;
#endif
	mutex_lock(&sensor->lock);
#if ERROR_EXEC
	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}
#endif
	ret = mdin_try_fmt_internal((const struct v4l2_subdev*)sd, mbus_fmt, &new_mode);
#if 0
	ret = mdin_try_fmt_internal(sd, mbus_fmt, sensor->current_fr, &new_mode);
#endif	
#if ERROR_EXEC
	if (ret)
		goto out;
#endif

#if 0
	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
	else
#endif		
	fmt = &sensor->fmt;
	*fmt = *mbus_fmt;
#if ERROR_EXEC		
out:
#endif	
	mutex_unlock(&sensor->lock);
	return ret;
}

static s32 mdin_g_volatile_ctrl(const struct v4l2_ctrl *ctrl)
{
	(void)(ctrl);
	return 0;
}

static s32 mdin_s_ctrl(const struct v4l2_ctrl *ctrl)
{
	(void)(ctrl);
return 1;	
}

static const struct v4l2_ctrl_ops mdin_ctrl_ops = {
	.g_volatile_ctrl = (func_type_ctrl)(&mdin_g_volatile_ctrl),
	.s_ctrl = (func_type_ctrl)(&mdin_s_ctrl),
};

static s32 mdin_init_controls(struct mdin_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &mdin_ctrl_ops;
	struct mdin_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	u8 uval1, uval2;
  
	
	v4l2_ctrl_handler_init(hdl, 32);  /* MISRA_C_2012_11_05 false_alarm */

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Auto/manual white balance */
	ctrls->auto_wb = v4l2_ctrl_new_std(hdl, ops,
					   V4L2_CID_AUTO_WHITE_BALANCE,
					   0, 1, 1, 1);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4095, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4095, 1, 0);
	/* Auto/manual exposure */
	uval1 = (u8)V4L2_EXPOSURE_MANUAL;
	uval2 = (u8)V4L2_EXPOSURE_AUTO;

	ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
						 V4L2_CID_EXPOSURE_AUTO,
						 (u8)uval1, (u64)0,
						 (u8)uval2);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 65535, 1, 0);
	/* Auto/manual gain */
	ctrls->auto_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN,
					     0, 1, 1, 1);
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN,
					0, 1023, 1, 0);

	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 255, 1, 64);
	ctrls->hue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE,
				       0, 359, 1, 0);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 255, 1, 0);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP,
					 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP,
					 0, 1, 1, 0);
	uval1 = (u8)V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
	uval2 = (u8)V4L2_CID_POWER_LINE_FREQUENCY_50HZ;

	ctrls->light_freq =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_POWER_LINE_FREQUENCY,
				       (u8)uval1, (u64)0,
				       (u8)uval2);
#if ERROR_EXEC
	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}
#endif

	ctrls->gain->flags |= (u64)V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= (u64)V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_wb, 0, 0);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, 1);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, 1);

	sensor->sd.ctrl_handler = hdl;
	return 0;
#if ERROR_EXEC
free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
#endif	
} 

static s32 mdin_enum_frame_size(const struct v4l2_subdev *sd,
				  const struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	(void)(sd);
	(void)(cfg);
	s32 ret = -EINVAL;
	
#if ERROR_EXEC
	if (fse->pad != 0)
		{
			ret = -EINVAL;
		}
		else
#endif		
	if (fse->index < (u32)MDIN_NUM_MODES) {
		fse->min_width  = mdin_mode_data[fse->index].hact;
		fse->max_width  = fse->min_width;
		fse->min_height = mdin_mode_data[fse->index].vact;
		fse->max_height = fse->min_height;
    ret = 0;
 }
	return ret;
}

static s32 mdin_enum_frame_interval(
	const struct v4l2_subdev *sd,
	const struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
#if 0	
	struct mdin_dev *sensor = to_mdin_dev(sd);
#endif 
	(void)(sd);
	(void)(cfg);
  s32 i, j, count, ret;
	
	  ret = -EINVAL;
	
	fie->interval.numerator = 1;
    

	count = 0;
	for (i = 0; i < (s32)MDIN_NUM_FRAMERATES; i++) {
		for (j = 0; j < (s32)MDIN_NUM_MODES; j++) {
			if ((fie->width  == mdin_mode_data[j].hact) &&
			    (fie->height == mdin_mode_data[j].vact))
			    {
				     count++;
         }
			if ((s32)fie->index == (count - 1)) {
				fie->interval.denominator = (u32)mdin_framerates[i];
				ret = 0;
			}
		}
	}

	return ret;

}

static s32 mdin_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct mdin_dev *sensor = to_mdin_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static s32 mdin_s_frame_interval(struct v4l2_subdev *sd,
				   const struct v4l2_subdev_frame_interval *fi)
{
	(void)(fi);
	struct mdin_dev *sensor = to_mdin_dev(sd);
	const struct mdin_mode_info *mode;
#if ERROR_EXEC	
	s32 frame_rate
#endif
	s32 ret = 0;

#if ERROR_EXEC
	if (fi->pad != 0)
		return -EINVAL;
#endif
	mutex_lock(&sensor->lock);
#if ERROR_EXEC
	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}
#endif
	mode = sensor->current_mode;
	mode = mdin_find_mode((s32)mode->hact,	(s32)mode->vact);
#if 0
	mode = mdin_find_mode(sensor, frame_rate, mode->hact,	mode->vact, true);
#endif	
#if ERROR_EXEC				
	if (!mode) {
		ret = -EINVAL;
		goto out;
	}
#endif
#if ERROR_EXEC
out:
#endif	
	mutex_unlock(&sensor->lock);
	return ret;
}

static s32 mdin_s_stream(struct v4l2_subdev *sd, s32 enable)
{
	struct mdin_dev *sensor = to_mdin_dev(sd);
#if ERROR_EXEC	
	struct i2c_client *client = sensor->i2c_client;
#endif
	
	s32 ret = 0;

	mutex_lock(&sensor->lock);

	if ((s32)sensor->streaming != enable) {
     ret = 0;
#if 0
		ret = mdin_check_valid_mode(sensor,
					      sensor->current_mode,
					      sensor->current_fr);
#endif
#if ERROR_EXEC
		if (ret) {
			dev_err(&client->dev, "Not support WxH@fps=%dx%d@%d\n",
				sensor->current_mode->hact,
				sensor->current_mode->vact,
				mdin_framerates[sensor->current_fr]);
			goto out;
		}
#endif
		sensor->ep.bus_type = V4L2_MBUS_CSI2_DPHY;
#if 0
		mdin_set_stream_mipi(sensor, enable);
#endif		
		sensor->streaming = enable;
	}
#if ERROR_EXEC
out:
#endif
	
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops mdin_core_ops = {
	.s_power           = (func_type_subdev)(&mdin_s_power),
	.log_status        = &v4l2_ctrl_subdev_log_status,
	.subscribe_event   = &v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = &v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mdin_video_ops = {
	.g_frame_interval  = (func_type_interval)(&mdin_g_frame_interval),
	.s_frame_interval  = (func_type_interval)(&mdin_s_frame_interval),
	.s_stream          = &mdin_s_stream,
};

static const struct v4l2_subdev_pad_ops mdin_pad_ops = {
	.enum_mbus_code      = NULL, 
	.get_fmt             = (func_type_getfmt)(&mdin_get_fmt),
	.set_fmt             = (func_type_setfmt)(&mdin_set_fmt),
	.enum_frame_size     = (func_type_framesize)(&mdin_enum_frame_size),
	.enum_frame_interval = (func_type_frameinterval)(&mdin_enum_frame_interval),
};

static const struct v4l2_subdev_ops mdin_subdev_ops = {
	.core  = &mdin_core_ops,
	.video = &mdin_video_ops,
	.pad   = &mdin_pad_ops,
};

static s32 mdin_link_setup(const struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	(void)(entity);
	(void)(local);
	(void)(remote);
	(void)(flags);	
	return 0;
}

static const struct media_entity_operations mdin_sd_media_ops = {
	.link_setup = (func_type_linksetup)(&mdin_link_setup),
};


static ssize_t value_store_mdin(const struct device *dev,
		const struct device_attribute *attr, const sc8* buf, size_t size)
{
	(void)(dev);
	(void)(attr);
	(void)(size);

	static const s32 mdin_framerates640_480[1] = {
		[MDIN_FPS] = 120,
	};
	static const struct mdin_mode_info mdin_mode_data640_480[MDIN_NUM_MODES] = {
		{MDIN_MODE, 640, 1896, 480, 1080 },
	};

	ssize_t status = 1;

   	mutex_lock(&g_sensor->lock);

    if ((s32)(buf[0]) == 0) {
		gpio_set_value(mdin_g_pwr_gpio, 0);  /* MISRA_C_2012_08_04 false_alarm */
                                         /* MISRA_C_2012_17_03 false_alarm */
		
		pr_err(" %s Power off   \n", __func__);
	} else {
		gpio_set_value(mdin_g_pwr_gpio, 1); /* MISRA_C_2012_08_04 false_alarm */
                                         /* MISRA_C_2012_17_03 false_alarm */
		pr_err(" %s Power on  \n", __func__);
	}

	if ((s32)(buf[1]) == 0) {
		pr_err("  %s  640 480  \n", __func__);
		mdin_mode_data[0] = mdin_mode_data640_480[0];
		mdin_framerates[0] = mdin_framerates640_480[0];
    } else if((s32)(buf[1]) == 1) {
		pr_err(" %s 1280 960  \n", __func__);
		mdin_mode_data[0] = mdin_mode_data1280_960[0];
		mdin_framerates[0] = mdin_framerates1280_960[0];
	} 
	else {
		/* nothing to do */
	}

	mutex_unlock(&g_sensor->lock);

	return status;
} 


static s32 mdin_probe_sub1(struct i2c_client *client, struct mdin_dev *sensor)
{

	struct device *dev;
	struct fwnode_handle *endpoint;
	u32 rotation;
	s32 ret;

  dev = &client->dev;
	/* optional indication of physical rotation of sensor */
	ret = fwnode_property_read_u32((const struct fwnode_handle *)dev_fwnode(&client->dev), 
		(const s8*)"rotation", &rotation);  /* DAPA_CWE_628 false_alarm */
#if 0	
	ret = fwnode_property_read_u32(dev_fwnode(&client->dev), "rotation",
				       &rotation);
#endif				       
#if ERROR_EXEC				       
	if (!ret) {
		switch (rotation) {
		case 180:
			sensor->upside_down = true;
			/* fall through */
		case 0:
			break;
		default:
			dev_warn(dev, "%u degrees rotation is not supported, ignoring...\n",
				 rotation);
		}
	}
#endif
	endpoint = fwnode_graph_get_next_endpoint((const struct fwnode_handle *)dev_fwnode(&client->dev),(struct fwnode_handle *)NULL);
	/* DAPA_CWE_628 false_alarm */  /*MISRA_C_2012_11_09 false_alarm*/
#if 0
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
#endif
#if ERROR_EXEC						  
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}
#endif
	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep); /* DAPA_CWE_628 false_alarm */
	fwnode_handle_put(endpoint);
#if ERROR_EXEC	
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}
#endif
	/* get system clock (xclk) */

		sensor->xclk = devm_clk_get((struct device *)dev, (const s8 *)"xclk");

#if ERROR_EXEC	
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}
#endif
	sensor->xclk_freq = (u32)clk_get_rate(sensor->xclk);
	
	dev_err(dev, "  sensor->xclk_freq = 0x%08x \n",sensor->xclk_freq);
  return ret;
}


static s32 mdin_probe_sub2(struct i2c_client *client)
{

	struct device *dev; 
	s32 ret;
 	u32 pwr_enable_gpio;
	struct device_node *np;
	


   dev = &client->dev;
   np  = dev->of_node;
  	pwr_enable_gpio= (u32)of_get_named_gpio(np, "pwr-enable-gpio", 0);

	if (gpio_is_valid(pwr_enable_gpio) > 0) { /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
		
		ret = gpio_request_one(pwr_enable_gpio, (u64)(GPIOF_OUT_INIT_LOW), "camera-pwr-enable-gpio"); /* MISRA_C_2012_08_04 false_alarm */
		                                                                                              /* MISRA_C_2012_10_01 false_alarm */
                                                      		                                        /* MISRA_C_2012_17_03 false_alarm */
#if ERROR_EXEC
		if (ret) {
          /* nothing to do */
		} else {
			gpio_set_value(pwr_enable_gpio, 1);   /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
			dev_info(dev, "[CAM] %s: camera power enabled\n", __func__);
			gpio_free(pwr_enable_gpio);           /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
		}
#endif		
	}

#if  APPL_USE_PWR 	
  	mdin_g_pwr_gpio = pwr_enable_gpio;
#if 0  
  sensor->pwr_gpio = pwr_enable_gpio;
#endif  
#endif

  return ret;
}

static s32 mdin_probe_sub3(struct i2c_client *client, struct mdin_dev *sensor)
{
	static struct device_attribute dev_attr_value_mdin = \
		__ATTR(value, 0660, NULL, (func_type_storemdin)(&value_store_mdin));  /* MISRA_C_2012_11_01 false_alarm */
		                                                   		                /* MISRA_C_2012_07_01 false_alarm */
		

	static const struct attribute *gpio_attrs_mdin[2] = { /* MISRA_C_2012_08_13 false_alarm */
		&dev_attr_value_mdin.attr,
		NULL,                                               /* MISRA_C_2012_11_05 false_alarm */
	};

	static const struct attribute_group gpio_group_mdin = {
		.attrs = (struct attribute **)(&gpio_attrs_mdin[0]), /* MISRA_C_2012_11_08 false_alarm */
		                                                     /* MISRA_C_2012_11_03 false_alarm */   
	};

	const struct device *dev;
	s32 ret;

  dev = &client->dev;
	v4l2_i2c_subdev_init(&sensor->sd, client, &mdin_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &mdin_sd_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);

	mutex_init(&sensor->lock);

	ret = mdin_init_controls(sensor);
	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	dev_info(dev, "eocam : i1643 probe success !\n");

#if APPL_USE_PWR	
	ret = sysfs_create_group((struct kobject *)&client->dev.kobj, (const struct attribute_group *)&gpio_group_mdin);
                                                                                         	/* MISRA_C_2012_11_03 false_alarm */
#endif

  return ret;
}




static s32 mdin_probe(struct i2c_client *client)
{
	struct device *dev;
	struct mdin_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	s32 ret;

  dev = &client->dev;
  
	sensor = (struct mdin_dev *)devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);  /*DAPA_CWE_628 */
	sensor->i2c_client = client;
 	g_sensor = sensor;

	mdin_mode_data[0] = mdin_mode_data1280_960[0];
	mdin_framerates[0] = mdin_framerates1280_960[0];

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace = (u32)V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc =(u16) V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = (u16)V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = (u16)V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width =  mdin_mode_data[MDIN_MODE].hact;
	fmt->height = mdin_mode_data[MDIN_MODE].vact;

	fmt->field = (u32)V4L2_FIELD_NONE;
	
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = (u32)mdin_framerates[MDIN_FPS];
	sensor->current_fr = MDIN_FPS;

	sensor->current_mode = &mdin_mode_data[MDIN_MODE];
	sensor->ae_target = 52;


	ret = mdin_probe_sub1(client, sensor);
	ret = mdin_probe_sub2(client);

	ret = mdin_probe_sub3(client,sensor);

  pr_err("%s  OK \n", __func__);

	return ret;
}


static const struct i2c_device_id mdin_id[] = {
	{"mdin", 0},       /* MISRA_C_2012_08_09 false_alarm */
	{},
};
MODULE_DEVICE_TABLE(i2c, mdin_id); /* MISRA_C_2012_08_01 false_alarm */
                                     /* MISRA_C_2012_08_05 false_alarm */
                                     /* MISRA_C_2012_08_02 false_alarm */

static const struct of_device_id mdin_dt_ids[] = {
	{ .compatible = "mdin,i540" }, /* MISRA_C_2012_08_09 false_alarm */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdin_dt_ids); /* MISRA_C_2012_08_01 false_alarm */
                                     /* MISRA_C_2012_08_02 false_alarm */
                                     /* MISRA_C_2012_08_05 false_alarm */
 
 
#if 0
module_i2c_driver(mdin_i2c_driver);
#endif



static s32 __init module_begin_mdin(void){  /* MISRA_C_2012_02_08 false_alarm */

	static struct i2c_driver mdin_i2c_driver = {
		.driver = {
			.name  = "mdin",
			.of_match_table	= mdin_dt_ids,
		},  
		.id_table  = mdin_id,
		.probe_new = &mdin_probe,
		.remove    = NULL, 
	};

    return i2c_add_driver(&mdin_i2c_driver); /* MISRA_C_2012_08_04 false_alarm */
                                             /* MISRA_C_2012_17_03 false_alarm */
}
 
static void __exit module_end_mdin(void){
	return;
}
 
module_init(module_begin_mdin);   /* MISRA_C_2012_08_01 false_alarm */
                                  /* MISRA_C_2012_08_02 false_alarm */
                                  /* MISRA_C_2012_08_05 false_alarm */
                                  /* MISRA_C_2012_08_06 false_alarm */
module_exit(module_end_mdin);   /* MISRA_C_2012_08_01 false_alarm */
                                  /* MISRA_C_2012_08_02 false_alarm */
                                  /* MISRA_C_2012_08_05 false_alarm */
                                  /* MISRA_C_2012_08_06 false_alarm */

MODULE_DESCRIPTION("eocam MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
   
