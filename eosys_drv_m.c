
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


#define EOSYS_MODE_640_480 	0
#define  EOSYS_NUM_MODES  	1


#define EOSYS_NUM_FRAMERATES 1

/////////////////////////////////////////////////////////////////////////////////


typedef enum  {
	EOSYS_60_FPS =0,
}eocam_frame_rate;

struct eocam_pixfmt {
	u32 code;
	u32 colorspace;
};


/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */

static const s32 eocam_framerates[1] = {
	[EOSYS_60_FPS] = 60,
};

/*
 * Image size under 1280 * 960 are SUBSAMPLING
 * Image size upper 1280 * 960 are SCALING
 */
#if 0
enum eocam_downsize_mode {
	SUBSAMPLING,
/*	SCALING, */
};
#endif


struct eocam_mode_info {
	s32 id;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
};

struct eocam_ctrls {
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

struct eocam_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to eocam */
	u32 xclk_freq;

	s32   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;

	const struct eocam_mode_info *current_mode;
	const struct eocam_mode_info *last_mode;
	eocam_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct eocam_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	s32 streaming;
};

#if APPL_USE_PWR
static	u32 eosys_g_pwr_gpio;
#endif

static const struct eocam_mode_info
eocam_mode_data[EOSYS_NUM_MODES] = {
	{EOSYS_MODE_640_480,
	 640, 1896, 480, 1080,
	},
};

#if 0
eocam_mode_data[EOSYS_NUM_MODES] = {
	{EOSYS_MODE_640_480, SUBSAMPLING,
	 640, 1896, 480, 1080,
	},
};
#endif


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
typedef ssize_t (*func_type_showeosys)(struct device *dev, struct device_attribute *attr, sc8* buf);
typedef ssize_t (*func_type_storeeosys)(struct device *dev, struct device_attribute *attr, const sc8* buf, size_t size);

/////////////////////////////////////////////////////////////////////////////////


static inline struct eocam_dev *to_eocam_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct eocam_dev, sd);      
	
	                                                  /* MISRA_C_2012_18_04 false_alarm */
                                                     /* MISRA_C_2012_01_02 false_alarm */ 
                                                     /* MISRA_C_2012_11_05 false_alarm */ 
                                                     /* MISRA_C_2012_11_09 false_alarm */ 

	
};	


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

#if 0
static s32 eocam_check_valid_mode(struct eocam_dev *sensor,
				   const struct eocam_mode_info *mode,
				   enum eocam_frame_rate rate)
{
	return 0;
}
#endif

#if 0
static s32 eocam_set_stream_mipi(struct eocam_dev *sensor, s32 on)
{
	return 0;
}
#endif

#if 0
static const struct eocam_mode_info *
eocam_find_mode(struct eocam_dev *sensor, enum eocam_frame_rate fr, s32 width, s32 height, bool nearest)
#endif

static const struct eocam_mode_info * eocam_find_mode( s32 width, s32 height)
{
	const struct eocam_mode_info *mode;
	
		mode = v4l2_find_nearest_size(eocam_mode_data,       /* MISRA_C_2012_11_09 false_alarm */
				      1, /*ARRAY_SIZE(eocam_mode_data), */
				      hact, vact,
				      width, height);
#if 0
	mode = v4l2_find_nearest_size(eocam_mode_data,
				      ARRAY_SIZE(eocam_mode_data),
				      hact, vact,
				      width, height);
#endif

#if ERROR_EXEC
	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height)))
		return NULL;
#endif

	return mode;
}

/* --------------- Subdev Operations --------------- */
#if 1
static s32 eocam_s_power(const struct v4l2_subdev *sd, s32 on)
{
	(void)(sd);
	pr_err("%s ==> %d\n", __func__, on);

	return 0;
}
#endif

#if ERROR_EXEC
#if 0
static s32 eocam_try_frame_interval(struct eocam_dev *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
#endif
static s32 eocam_try_frame_interval(struct eocam_dev *sensor,
				     u32 width, u32 height)
{
	const struct eocam_mode_info *mode;
	eocam_frame_rate rate = EOSYS_60_FPS;

	mode = eocam_find_mode((s32)width, (s32)height);
#if 0
	mode = eocam_find_mode(sensor, rate, (s32)width, (s32)height, false);
#endif	
	return mode ? (s32)rate : -EINVAL;
}
#endif


static s32 eocam_get_fmt(struct v4l2_subdev *sd,
			  const struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	(void)(cfg);
	struct eocam_dev *sensor = to_eocam_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

#if ERROR_EXEC
	if (format->pad != 0)
		return -EINVAL;
#endif

	mutex_lock(&sensor->lock);

	fmt = &sensor->fmt;
	fmt->reserved[1] = (u16)((sensor->current_fr == EOSYS_60_FPS) ? 60 : 30);
	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

#if 0
static s32 eocam_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum eocam_frame_rate fr,
				   const struct eocam_mode_info **new_mode)
#else
static s32 eocam_try_fmt_internal(const struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   const struct eocam_mode_info **new_mode)
#endif
{
#if 0
	struct eocam_dev *sensor = to_eocam_dev(sd);
#endif

	(void)(sd);
	const struct eocam_mode_info *mode;
	s32 i;

  const struct eocam_pixfmt eocam_formats[] = {
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

#if 1
	mode = eocam_find_mode((s32)fmt->width, (s32)fmt->height);
#else 
	mode = eocam_find_mode(sensor, fr, fmt->width, fmt->height, true);
#endif

#if 	ERROR_EXEC
	if (!mode)
		return -EINVAL;
#endif

	fmt->width = mode->hact;
	fmt->height = mode->vact;
	memset(fmt->reserved, 0, sizeof(fmt->reserved)); /* MISRA_C_2012_08_04 false_alarm */
	                                                 /* MISRA_C_2012_17_03 false_alarm */
	                                                   
	*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(eocam_formats); i++) {
		if (eocam_formats[i].code == fmt->code) {
			break;
		}
	}

#if 	ERROR_EXEC			
	if (i >= ARRAY_SIZE(eocam_formats))
		i = 0;
#endif

	fmt->code = eocam_formats[i].code;
	fmt->colorspace = eocam_formats[i].colorspace;
	fmt->ycbcr_enc = (u16)V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = (u16)V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = (u16)V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}


static s32 eocam_set_fmt(struct v4l2_subdev *sd,
			  const struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	(void)(cfg);
	struct eocam_dev *sensor = to_eocam_dev(sd);
	const struct eocam_mode_info *new_mode;
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

#if 1
	ret = eocam_try_fmt_internal((const struct v4l2_subdev*)sd, mbus_fmt, &new_mode);
#else
	ret = eocam_try_fmt_internal(sd, mbus_fmt, sensor->current_fr, &new_mode);
#endif

#if ERROR_EXEC
	if (ret != 0)
		goto out;
#endif

	fmt = &sensor->fmt;
	*fmt = *mbus_fmt;

#if ERROR_EXEC
out:
#endif

	mutex_unlock(&sensor->lock);
	return ret;
}


static s32 eocam_g_volatile_ctrl(const struct v4l2_ctrl *ctrl)
{
	(void)(ctrl);
	return 0;
}

static s32 eocam_s_ctrl(const struct v4l2_ctrl *ctrl)
{
	(void)(ctrl);
	return 1;
}


static const struct v4l2_ctrl_ops eocam_ctrl_ops = {
	.g_volatile_ctrl = (func_type_ctrl)(&eocam_g_volatile_ctrl),
	.s_ctrl = (func_type_ctrl)(&eocam_s_ctrl),
};

static s32 eocam_init_controls(struct eocam_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &eocam_ctrl_ops;
	struct eocam_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	
	u8 uval1, uval2;
	
	v4l2_ctrl_handler_init(hdl, 32); /* MISRA_C_2012_11_05 false_alarm */

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Auto/manual white balance */
	ctrls->auto_wb 		= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE, 0, 4095, 1, 0);
	ctrls->red_balance 	= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE, 0, 4095, 1, 0);
	/* Auto/manual exposure */
	uval1 = (u8)V4L2_EXPOSURE_MANUAL;
	uval2 = (u8)V4L2_EXPOSURE_AUTO;
	ctrls->auto_exp 	= v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO, (u8)uval1, (u64)0, (u8)uval2);
	ctrls->exposure 	= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0, 65535, 1, 0);
	/* Auto/manual gain */
	ctrls->auto_gain 	= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	ctrls->gain 		= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN, 0, 1023, 1, 0);
	ctrls->saturation 	= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION, 0, 255, 1, 64);
	ctrls->hue 			= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE, 0, 359, 1, 0);
	ctrls->contrast 	= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST, 0, 255, 1, 0);
	ctrls->hflip 		= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip 		= v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	uval1 = (u8)V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
	uval2 = (u8)V4L2_CID_POWER_LINE_FREQUENCY_50HZ;

	ctrls->gain->flags |= (u64)V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= (u64)V4L2_CTRL_FLAG_VOLATILE;
	ctrls->light_freq = v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_POWER_LINE_FREQUENCY, (u8)uval1, (u64)0, (u8)uval2);

#if ERROR_EXEC
	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}
#endif

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_wb, 0, 0);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, 1); 
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, 1);

	sensor->sd.ctrl_handler = hdl;

#if ERROR_EXEC
free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
#endif

	return 0;
} 


static s32 eocam_enum_frame_size(const struct v4l2_subdev *sd,
				  const struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	(void)(sd);
	(void)(cfg);
	s32 ret = -EINVAL;
			
#if ERROR_EXEC
	if (fse->pad != 0) {	
		ret =  -EINVAL;
	}
	else
#else
	if (fse->index < (u32)EOSYS_NUM_MODES) {
#endif
		fse->min_width  = eocam_mode_data[fse->index].hact;
		fse->max_width  = fse->min_width;
		fse->min_height = eocam_mode_data[fse->index].vact;
		fse->max_height = fse->min_height;
  		ret = 0;
  	}

	return ret;
}


static s32 eocam_enum_frame_interval(
	const struct v4l2_subdev *sd,
	const struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	(void)(sd);
	(void)(cfg);
	s32 i, j;
	s32 count = 0;
	s32 ret = -EINVAL;

	fie->interval.numerator = 1;
	for (i = 0; i < (s32)EOSYS_NUM_FRAMERATES; i++) {
		for (j = 0; j < (s32)EOSYS_NUM_MODES; j++) {
			if ((fie->width  == eocam_mode_data[j].hact) && (fie->height == eocam_mode_data[j].vact)) {    
				count++;
      		}
			if ((s32)fie->index == (count - 1)) {
				fie->interval.denominator = (u32)eocam_framerates[i];
				ret = 0;
			}
		}
	}

	return ret;
}


static s32 eocam_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct eocam_dev *sensor = to_eocam_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}


static s32 eocam_s_frame_interval(struct v4l2_subdev *sd,
				   const struct v4l2_subdev_frame_interval *fi)
{
	(void)(fi);
	struct eocam_dev *sensor; 
	const struct eocam_mode_info *mode;
#if ERROR_EXEC
	s32 frame_rate;
#endif	
	s32 ret = 0;

  	sensor = to_eocam_dev(sd);

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

#if ERROR_EXEC
	frame_rate = eocam_try_frame_interval(sensor,  mode->hact, mode->vact);
#if 0
	frame_rate = eocam_try_frame_interval(sensor, &fi->interval, mode->hact, mode->vact);
#endif

	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}
#endif

	mode = eocam_find_mode( (s32)mode->hact,	(s32)mode->vact);

#if 0
	mode = eocam_find_mode(sensor, frame_rate, mode->hact,	mode->vact, true);
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

static s32 eocam_s_stream(struct v4l2_subdev *sd, s32 enable)
{
	struct eocam_dev *sensor; 

#if ERROR_EXEC
	struct i2c_client *client;
#endif
	s32 ret = 0;
	
	sensor = to_eocam_dev(sd);

#if ERROR_EXEC
	client = sensor->i2c_client;
#endif

	mutex_lock(&sensor->lock);

	if ((s32)sensor->streaming != enable) {
		ret = 0;
#if ERROR_EXEC
		if (ret != 0) {
			dev_err(&client->dev, "Not support WxH@fps=%dx%d@%d\n",
				sensor->current_mode->hact,
				sensor->current_mode->vact,
				eocam_framerates[sensor->current_fr]);
			goto out;
		}
#endif

		sensor->ep.bus_type = V4L2_MBUS_CSI2_DPHY;
#if 0
		eocam_set_stream_mipi(sensor, enable);
#endif		
		sensor->streaming = enable;
	}

#if ERROR_EXEC	
out:
#endif

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops eocam_core_ops = {
	.s_power           = (func_type_subdev)(&eocam_s_power),
	.log_status        = &v4l2_ctrl_subdev_log_status,
	.subscribe_event   = &v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = &v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops eocam_video_ops = {
	.g_frame_interval  = (func_type_interval)(&eocam_g_frame_interval),
	.s_frame_interval  = (func_type_interval)(&eocam_s_frame_interval),
	.s_stream          = &eocam_s_stream,
};

static const struct v4l2_subdev_pad_ops eocam_pad_ops = {
#if 0
	.enum_mbus_code      = seocam_enum_mbus_code,
#else	
	.enum_mbus_code      = NULL,
#endif
	
	.get_fmt             = (func_type_getfmt)(&eocam_get_fmt),
	.set_fmt             = (func_type_setfmt)(&eocam_set_fmt),
	.enum_frame_size     = (func_type_framesize)(&eocam_enum_frame_size),
	.enum_frame_interval = (func_type_frameinterval)(&eocam_enum_frame_interval),
};

static const struct v4l2_subdev_ops eocam_subdev_ops = {
	.core  = &eocam_core_ops,
	.video = &eocam_video_ops,
	.pad   = &eocam_pad_ops,
};



static s32 eocam_link_setup(const struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	(void)(entity);
	(void)(local);
	(void)(remote);
	(void)(flags);

	return 0;
}

static const struct media_entity_operations eocam_sd_media_ops = {
	.link_setup = (func_type_linksetup)(&eocam_link_setup),
};


static ssize_t value_show_eosys(const struct device *dev,
		const struct device_attribute *attr, const sc8* buf)
{
	(void)(dev);
	(void)(attr);
	(void)(buf);
	ssize_t status = 1;
	return status;
}

static ssize_t value_store_eosys(const struct device *dev,
		const struct device_attribute *attr, const sc8* buf, size_t size)
{
	(void)(dev);
	(void)(attr);
	(void)(size);
	ssize_t status;

#if  APPL_USE_PWR
	u8 zero = (u8)('0');
    if((u8)(buf[0]) == zero) {
		gpio_set_value(eosys_g_pwr_gpio, 0);    /* MISRA_C_2012_08_04 false_alarm */
                                            /* MISRA_C_2012_17_03 false_alarm */

		dev_err(dev, " %s Power off   %c ==>\n", __func__, *buf);
	} else {
		gpio_set_value(eosys_g_pwr_gpio, 1);    /* MISRA_C_2012_08_04 false_alarm */
                                            /* MISRA_C_2012_17_03 false_alarm */
		dev_err(dev, " %s Power on   %c ==>\n", __func__, *buf);
	}
#endif

   	status = 1;
	return status;
}

static s32 eocam_probe_sub1(struct i2c_client *client, struct eocam_dev *sensor)
{
	
 struct device *dev;
	struct fwnode_handle *endpoint;
	u32 rotation;
	s32 ret;
	
	dev = &client->dev;
		/* optional indication of physical rotation of sensor */
	ret = fwnode_property_read_u32((const struct fwnode_handle *)dev_fwnode(&client->dev), 
		(const s8*)"rotation", &rotation);   /* DAPA_CWE_628 false_alarm */

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

#if 1
	endpoint = fwnode_graph_get_next_endpoint((const struct fwnode_handle *)dev_fwnode(&client->dev), (struct fwnode_handle *)NULL);
	   /* DAPA_CWE_628 false_alarm */ /*MISRA_C_2012_11_09 false_alarm*/
#else
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
	if (ret != 0) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}
#endif
	
	sensor->xclk = devm_clk_get((struct device *)dev, (const s8 *)"xclk");

#if ERROR_EXEC	
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}
#endif

	sensor->xclk_freq = (u32)clk_get_rate(sensor->xclk);
  return ret;

}

static s32 eocam_probe_sub2(struct i2c_client *client)
{

	const struct device *dev;
	s32 ret;
	u32 pwr_enable_gpio;
	struct device_node *np;
	


  dev = &client->dev; 
 np =  dev->of_node;
	pwr_enable_gpio = (u32)of_get_named_gpio(np, "pwr-enable-gpio", 0);

	if (gpio_is_valid(pwr_enable_gpio) > 0) { /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
		ret = gpio_request_one(pwr_enable_gpio, (u64)(GPIOF_OUT_INIT_LOW), "camera-pwr-enable-gpio"); /* MISRA_C_2012_08_04 false_alarm */
		                                                                                              /* MISRA_C_2012_10_01 false_alarm */
                                                      		                                        /* MISRA_C_2012_17_03 false_alarm */
		if (ret == 0) {
			gpio_set_value(pwr_enable_gpio, 1);   /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
			dev_info(dev, "[CAM] %s: camera power enabled\n", __func__);
			gpio_free(pwr_enable_gpio);           /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
		}
	}

#if  APPL_USE_PWR 	
  	eosys_g_pwr_gpio = pwr_enable_gpio;
#endif
   return ret;
}

static s32 eocam_probe_sub3(struct i2c_client *client, struct eocam_dev *sensor)
{
#if 0
	static struct device_attribute dev_attr_value_eosys = {
		.attr = {.name = "value_eosys", .mode = 432 },
		.show	= (func_type_showeosys)(&value_show_eosys),
		.store	= (func_type_storeeosys)(&value_store_eosys),
	};
#else
	static struct device_attribute dev_attr_value_eosys = \
		__ATTR(value, 0660, (func_type_showeosys)(&value_show_eosys), (func_type_storeeosys)(&value_store_eosys)); /* MISRA_C_2012_11_01 false_alarm */
		                                                   		                                                     /* MISRA_C_2012_07_01 false_alarm */

#endif
	static const struct attribute *gpio_attrs_eosys[2] = {   /* MISRA_C_2012_08_13 false_alarm */
		&dev_attr_value_eosys.attr,
		NULL,                                                  /* MISRA_C_2012_11_05 false_alarm */
	};
 	
	static struct attribute_group gpio_group_eosys = {
		.attrs = (struct attribute **)(&gpio_attrs_eosys[0]), /* MISRA_C_2012_11_08 false_alarm */
		                                                     /* MISRA_C_2012_11_03 false_alarm */   
	};

const	struct device *dev;
	s32 ret;


  dev = &client->dev; 

	v4l2_i2c_subdev_init(&sensor->sd, client, &eocam_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

 #if 1
	sensor->sd.entity.ops = &eocam_sd_media_ops;
#endif
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);

#if ERROR_EXEC		
	if (ret != 0)
		return ret;
#endif

	mutex_init(&sensor->lock);

	ret = eocam_init_controls(sensor);

#if ERROR_EXEC	
	if (ret != 0)
		goto entity_cleanup;
#endif

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);

#if ERROR_EXEC	
	if (ret != 0)
		goto free_ctrls;
#endif
	dev_info(dev, "eocam : i1643 probe success !\n");

#if APPL_USE_PWR	

	ret = sysfs_create_group((struct kobject *)(&client->dev.kobj), (const struct attribute_group *)&gpio_group_eosys);
                                                                         	/* MISRA_C_2012_11_03 false_alarm */
#if ERROR_EXEC	
	if(ret < 0) {
		dev_err(dev, "failed to create sysfs files\n");
	}
#endif
#endif
  return ret;
} 




	
static s32 eocam_probe(struct i2c_client *client)
{

	struct device *dev; 
	struct eocam_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	s32 ret;
   
   dev = &client->dev;  
   
	sensor = (struct eocam_dev *)devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL); /*DAPA_CWE_628 */

#if ERROR_EXEC	
	if (!sensor)
		return -ENOMEM;
#endif
	
	sensor->i2c_client = client;

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace = (u32)V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = (u16)V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = (u16)V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = (u16)V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 640;
	fmt->height = 480;
	fmt->field = (u32)V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = (u32)eocam_framerates[EOSYS_60_FPS];
	sensor->current_fr = EOSYS_60_FPS;
	sensor->current_mode = &eocam_mode_data[EOSYS_MODE_640_480];
	sensor->ae_target = 52;







   ret = eocam_probe_sub1(client, sensor);
   ret = eocam_probe_sub2(client);
   ret = eocam_probe_sub3(client, sensor);



  

 	pr_err("%s  OK  %d \n", __func__, ret);

	return 0;

#if ERROR_EXEC	
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
#endif	
}

static const struct i2c_device_id eocam_id[] = {
	{"eocam", 0},                      	/* MISRA_C_2012_08_09 false_alarm */

	{},
};
MODULE_DEVICE_TABLE(i2c, eocam_id);  /* MISRA_C_2012_08_01 false_alarm */
                                     /* MISRA_C_2012_08_06 false_alarm */
                                     /* MISRA_C_2012_08_02 false_alarm */

static const struct of_device_id eocam_dt_ids[] = {
	{ .compatible = "eocam,i1643" },  /* MISRA_C_2012_08_09 false_alarm */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, eocam_dt_ids); /* MISRA_C_2012_08_01 false_alarm */
                                     /* MISRA_C_2012_08_02 false_alarm */


static s32 __init module_begin_eosys(void)  /* MISRA_C_2012_02_08 false_alarm */
{
	static struct i2c_driver eocam_i2c_driver = {
		.driver = {
			.name  = "eocam",
			.of_match_table	= eocam_dt_ids,
		},
		.id_table  = eocam_id,
		.probe_new = &eocam_probe,
		.remove    = NULL, 
	};
	
  	return i2c_add_driver(&eocam_i2c_driver); /* MISRA_C_2012_08_04 false_alarm */
                                              /* MISRA_C_2012_17_03 false_alarm */
  	
}
 
static void __exit module_end_eosys(void){
	return;
}

module_init(module_begin_eosys);  
                                   /* MISRA_C_2012_08_01 false_alarm */
                                  /* MISRA_C_2012_08_02 false_alarm */
                                  /* MISRA_C_2012_08_06 false_alarm */
module_exit(module_end_eosys);  
                                  /* MISRA_C_2012_08_01 false_alarm */
                                  /* MISRA_C_2012_08_02 false_alarm */
                                  /* MISRA_C_2012_08_06 false_alarm */

MODULE_DESCRIPTION("eocam MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
