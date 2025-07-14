// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
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


#define m_1920 0


#define APPL_USE_PWR  1
#define ERROR_EXEC  0  


#define TP_MODE 0
#define TP_NUM_MODES 1


#if 0 
enum tp_mode_id {
	TP_MODE = 0,
	TP_NUM_MODES,
};
#endif
#define TP_NUM_FRAMERATES 1

/////////////////////////////////////////////////////////////////////////////////

typedef enum {
	TP_FPS=0,

} tp_frame_rate;

struct tp_pixfmt {
	u32 code;
	u32 colorspace;
};


/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */
 #if 0
static unsigned s32 virtual_channel;
module_param(virtual_channel, uint, 0444);
MODULE_PARM_DESC(virtual_channel,
		 "MIPI CSI-2 virtual channel (0..3), default 0");
#endif


static const s32 tp_framerates[1] = {
	[TP_FPS] = 60,
};

#if 0
/* regulator supplies */
static const char * const tp_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.5V) supply */
};




#define TP_NUM_SUPPLIES ARRAY_SIZE(tp_supply_name)
#endif

/*
 * Image size under 1280 * 960 are SUBSAMPLING
 * Image size upper 1280 * 960 are SCALING
 */
 #if 0
enum tp_downsize_mode {
	SUBSAMPLING,
	SCALING,
};
#endif


struct tp_mode_info {
	s32 id;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
};

struct tp_ctrls {
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

struct tp_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to tpcam */
	u32 xclk_freq;

#if 0
	s32 pwr_gpio;
#endif	
  s32 reset_gpio;

	s32   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;

	const struct tp_mode_info *current_mode;
	const struct tp_mode_info *last_mode;
	tp_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct tp_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	s32 streaming;
};
#if 0
s32 tp_g_pwr_gpio;
s32 tp_g_reset_gpio;
#endif

static const struct tp_dev *tp_g_sensor;

static const struct tp_mode_info tp_mode_data[TP_NUM_MODES] = {
#if m_1920
	{TP_MODE, 
	 1920, 2500, 1080, 1120,
	},
#else 	
	{TP_MODE, 
//	 720, 1892, 240, 740,
	 1280, 1892, 720, 740,
	},
#endif 	
};



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
typedef ssize_t (*func_type_storetp)(struct device *dev, struct device_attribute *attr, const sc8* buf, size_t size);

/////////////////////////////////////////////////////////////////////////////////


#if 1
static inline __attribute__((__gnu_inline__)) __attribute__((__unused__)) __attribute__((__no_instrument_function__)) struct tp_dev *to_tp_dev(struct v4l2_subdev *sd)
{
  return ({ 
  	                     /* MISRA_C_2012_01_02 false_alarm */ 
  void *__mptr = (void *)(sd); do { extern void __compiletime_assert_388(void) __attribute__((__error__("pointer type mismatch in container_of()"))); 
  	                                                         /* MISRA_C_2012_08_06 false_alarm */      
#if 0 
    if (!(!(!__builtin_types_compatible_p(typeof(*(sd)), typeof(((struct tp_dev *)0)->sd)) && !__builtin_types_compatible_p(typeof(*(sd)), typeof(void))))) 
        __compiletime_assert_388(); 
#endif      
      }
      
   while (0);
   
  ((struct tp_dev *)(__mptr - __builtin_offsetof(struct tp_dev, sd))); }); /* MISRA_C_2012_11_05 false_alarm */ 
                                                                           /* MISRA_C_2012_18_04 false_alarm */         
}
 #endif
#if 0
static inline struct tp_dev *to_tp_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tp_dev, sd);
}
#endif
#if 0
static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct tp_dev,
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





#define tp2860_DEFAULT_SLAVE_ID ((u16)68)	// (0x88U >> 1U)  // 0x8a //



enum
{		
	VIN1=0,
    VIN2=1,
    VIN3=2,
    VIN4=3,
};
#if 0
enum
{    STD_TVI,
    STD_HDA, //AHD
};
enum
{    PAL,
    NTSC,
    HD25,
    HD30,
    FHD25,
    FHD30,
    HD50,
    HD60,
    QHD25,  //only support with 2lane mode
    QHD30,	//only support with 2lane mode
    FHD50,	//only support with 2lane mode
    FHD60,	//only support with 2lane mode
};
enum
{    MIPI_2LANE,
    MIPI_1LANE,
};
#endif
static s32 tp2860_write_reg(u8 reg, u8 val)
{
	struct i2c_client *client = tp_g_sensor->i2c_client; 
	struct i2c_msg msg;
	u8 buf[2];
	s32 ret;

	buf[0] = reg;
	buf[1] = val;

	msg.addr = tp2860_DEFAULT_SLAVE_ID;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = (u16)sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1); /* MISRA_C_2012_17_03 false_alarm */
	                                              /* MISRA_C_2012_08_04 false_alarm */ 
	                                              /* DAPA_CWE_628 false_alarm */ 
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x\n", __func__, reg, val);
	}
	else {
		ret = 0;
	}

	return ret;
}



static u8 tp2860_read_reg(u8 reg)
{
	struct i2c_client *client = tp_g_sensor->i2c_client; 
	struct i2c_msg msg[2];
	u8 buf[2];
	u8 ret;

	buf[0] = reg;

	msg[0].addr = tp2860_DEFAULT_SLAVE_ID;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | (u16)I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = (u16)1;

	if (i2c_transfer(client->adapter, msg, 2) < 0) {  /* MISRA_C_2012_17_03 false_alarm */
	                                              /* MISRA_C_2012_08_04 false_alarm */ 
	                                              /* DAPA_CWE_628 false_alarm */ 
		dev_err(&client->dev, "%s: error: reg=%x\n",__func__, reg);
		ret = 0;
	}
	else {
		ret = buf[0];
	}

	return ret;
}

#if 0
void tp2860_mipi_out(unsigned char fmt, unsigned char std, unsigned char lane)
#endif

static void tp2860_mipi_out(void)
{	
	u8 tmp;
	
	(void)tp2860_write_reg(0x40, 0x08); 
	
	(void)tp2860_write_reg(0x02, 0x7d);
	(void)tp2860_write_reg(0x03, 0x75);
	(void)tp2860_write_reg(0x04, 0x75);
	(void)tp2860_write_reg(0x13, 0xef);	
	(void)tp2860_write_reg(0x20, 0x00);	
	(void)tp2860_write_reg(0x23, 0x9e);	

	(void)tp2860_write_reg(0x21, 0x12);
	(void)tp2860_write_reg(0x14, 0x41);
	(void)tp2860_write_reg(0x15, 0x02);			
				
	(void)tp2860_write_reg(0x2a, 0x04);
	(void)tp2860_write_reg(0x2b, 0x03);
	(void)tp2860_write_reg(0x2c, 0x09);
	(void)tp2860_write_reg(0x2e, 0x02);				
 
	tmp = tp2860_read_reg(0x14);
	(void)tp2860_write_reg(0x14, (u8)((u8)0x80 | tmp));
	(void)tp2860_write_reg(0x14, tmp);
	
	/* Enable MIPI CSI2 output */
	/* stream off */
	(void)tp2860_write_reg(0x28, 0x02);	 
#if 0	
	/*stream on */
	//tp2860_write_reg(0x28, 0x00);	//
#endif
  /*	back to decoder page */
	(void)tp2860_write_reg(0x40, 0x00); 

	return;
}
	
#if 0
//ch: video channel
//fmt: PAL/NTSC/HD25/HD30
//std: STD_TVI/STD_HDA
//lane: MIPI_2LANE/MIPI_1LANE
//sample: tp2860_sensor_init(VIN1,HD30,STD_TVI,MIPI_2LANE); //video is TVI 720p30 from Vin1
////////////////////////////////
#endif
#if 0
void tp2860_sensor_init(unsigned char ch,unsigned char fmt,unsigned char std, unsigned char lane)
#endif

static void tp2860_sensor_init(u8 ch)
{
	(void)tp2860_write_reg(0x40, 0x00); 
	(void)tp2860_write_reg(0x06, 0x12); 
	(void)tp2860_write_reg(0x42, 0x00);	
	(void)tp2860_write_reg(0x4e, 0x00); 
	(void)tp2860_write_reg(0x54, 0x00); 
	(void)tp2860_write_reg(0x41, ch);		

	(void)tp2860_write_reg(0x02, 0x42);

	(void)tp2860_write_reg(0x07, 0xc0); 
	(void)tp2860_write_reg(0x0b, 0xc0);  		
	(void)tp2860_write_reg(0x0c, 0x03); 
	(void)tp2860_write_reg(0x0d, 0x50);  

	(void)tp2860_write_reg(0x15, 0x13);
	(void)tp2860_write_reg(0x16, 0x15); 
	(void)tp2860_write_reg(0x17, 0x00); 
	(void)tp2860_write_reg(0x18, 0x19);
	(void)tp2860_write_reg(0x19, 0xd0);
	(void)tp2860_write_reg(0x1a, 0x25);			
	(void)tp2860_write_reg(0x1c, 0x06);  
	(void)tp2860_write_reg(0x1d, 0x72);  

	(void)tp2860_write_reg(0x20, 0x30);  
	(void)tp2860_write_reg(0x21, 0x84); 
	(void)tp2860_write_reg(0x22, 0x36);
	(void)tp2860_write_reg(0x23, 0x3c);

	(void)tp2860_write_reg(0x2b, 0x60);  
	(void)tp2860_write_reg(0x2c, 0x1a); 
	(void)tp2860_write_reg(0x2d, 0x30);
	(void)tp2860_write_reg(0x2e, 0x70);

	(void)tp2860_write_reg(0x30, 0x48);  
	(void)tp2860_write_reg(0x31, 0xbb); 
	(void)tp2860_write_reg(0x32, 0x2e);
	(void)tp2860_write_reg(0x33, 0x90);
	
	(void)tp2860_write_reg(0x35, 0x05); 
	(void)tp2860_write_reg(0x38, 0x00);
	(void)tp2860_write_reg(0x39, 0x1C); 

	tp2860_mipi_out();
#if 0
	tp2860_mipi_out(fmt, std, lane);
#endif
	return;
}

#if 0
static s32 tp_set_stream_mipi(struct tp_dev *sensor, bool on)
{
	return 0;
}
#endif
#if 0
static const struct tp_mode_info *
tp_find_mode(struct tp_dev *sensor, tp_frame_rate fr,
		 s32 width, s32 height, bool nearest)
#endif		 
static const struct tp_mode_info* tp_find_mode( s32 width, s32 height)
{
	const struct tp_mode_info *mode;

	mode = v4l2_find_nearest_size(tp_mode_data,   /* MISRA_C_2012_11_09 false_alarm */
				      1, /*ARRAY_SIZE(tp_mode_data), */
				      hact, vact,
				      width, height);

#if ERROR_EXEC
	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height)))
		return NULL;
#endif
	return mode;
}


static s32 tp_s_power(const struct v4l2_subdev *sd, s32 on)
{
	(void)(sd);
	pr_err("%s ==> %d\n", __func__, on);
   	return 0;
}

#if 0
static s32 tp_try_frame_interval(struct tp_dev *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct tp_mode_info *mode;
	tp_frame_rate rate = TP_FPS;
	

	mode = tp_find_mode((s32)width, (s32)height);
#if 0	
	mode = tp_find_mode(sensor, rate, width, height, false);
#endif
	return mode ? rate : -EINVAL;
}
#endif


static s32 tp_get_fmt(struct v4l2_subdev *sd,
			  const struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	(void)(cfg);
	struct tp_dev *sensor = to_tp_dev(sd);
	struct v4l2_mbus_framefmt *fmt;
#if ERROR_EXEC
	if (format->pad != 0)
		return -EINVAL;
#endif
	mutex_lock(&sensor->lock);

	fmt = &sensor->fmt;
	fmt->reserved[1] = 120U; // (u16)(sensor->current_fr == TP_FPS) ? 120 : 120; //
	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}
#if 0
static s32 tp_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum tp_frame_rate fr,
				   const struct tp_mode_info **new_mode)
#endif
static s32 tp_try_fmt_internal(
				   struct v4l2_mbus_framefmt *fmt,
				   const struct tp_mode_info **new_mode)
{
#if 0
	struct tp_dev *sensor = to_tp_dev(sd);
#endif	
	const struct tp_mode_info *mode;
	s32 i;

static const struct tp_pixfmt tp_formats[] = {
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


	mode = tp_find_mode((s32)fmt->width, (s32)fmt->height);
#if 0	
	mode = tp_find_mode(sensor, fr, fmt->width, fmt->height, true);
#endif
	
#if ERROR_EXEC
	if (!mode)
		return -EINVAL;
#endif

	fmt->width = mode->hact;
	fmt->height = mode->vact;
	memset(fmt->reserved, 0, sizeof(fmt->reserved)); /*MISRA_C_2012_08_04 false_alarm*/
	                                                 /* MISRA_C_2012_17_03 false_alarm */


/*	if (new_mode) */
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(tp_formats); i++)
	{
		if (tp_formats[i].code == fmt->code)
		{	
			break;
		}
	}
		
#if ERROR_EXEC
	if (i >= ARRAY_SIZE(tp_formats))
		i = 0;
#endif

	fmt->code = tp_formats[i].code;
	fmt->colorspace = tp_formats[i].colorspace;
	fmt->ycbcr_enc = (u16)V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = (u16)V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = (u16)V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	/* The top field is always transferred first by the chip */
	fmt->field = (u32)V4L2_FIELD_INTERLACED_TB; 

	return 0;
}

static s32 tp_set_fmt(struct v4l2_subdev *sd,
			  const struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	(void)(cfg);
	struct tp_dev *sensor = to_tp_dev(sd);
	const struct tp_mode_info *new_mode;
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

	ret = tp_try_fmt_internal(mbus_fmt, &new_mode);
#if 0
	ret = tp_try_fmt_internal(sd, mbus_fmt,
				      sensor->current_fr, &new_mode);
#endif				      
#if ERROR_EXEC
	if (ret)
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

static s32 tp_g_volatile_ctrl(const struct v4l2_ctrl *ctrl)
{
	(void)(ctrl);
	return 0;
}
static s32 tp_s_ctrl(const struct v4l2_ctrl *ctrl)
{
	(void)(ctrl);
	return 1;
}
static const struct v4l2_ctrl_ops tp_ctrl_ops = {
	.g_volatile_ctrl = (func_type_ctrl)(&tp_g_volatile_ctrl),
	.s_ctrl = (func_type_ctrl)(&tp_s_ctrl),
};

static s32 tp_init_controls(struct tp_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &tp_ctrl_ops;
	struct tp_ctrls *ctrls = &sensor->ctrls;
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

static s32 tp_enum_frame_size(const struct v4l2_subdev *sd,
				  const struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	(void)(sd);
	(void)(cfg);	
	s32 ret = -EINVAL;
	
	#if ERROR_EXEC
	if (fse->pad != 0)
	{	
		ret =  -EINVAL;
	}
	else
	#endif
	if (fse->index < (u32)TP_NUM_MODES) {
		fse->min_width  = tp_mode_data[fse->index].hact;
		fse->max_width  = fse->min_width;
		fse->min_height = tp_mode_data[fse->index].vact;
		fse->max_height = fse->min_height;
		ret = 0;
	}

	return ret;
}

static s32 tp_enum_frame_interval(
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
	for (i = 0; i < (s32)TP_NUM_FRAMERATES; i++) {
		for (j = 0; j < (s32)TP_NUM_MODES; j++) {
			if ((fie->width  == tp_mode_data[j].hact) &&
			    (fie->height == tp_mode_data[j].vact) )
			    {
					count++;
				}

			if ((s32)fie->index == (count - 1)) {
				fie->interval.denominator = (u32)tp_framerates[i];
				ret = 0;
			}
		}
	}
	return ret;
}

static s32 tp_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct tp_dev *sensor = to_tp_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static s32 tp_s_frame_interval(struct v4l2_subdev *sd,
				   const struct v4l2_subdev_frame_interval *fi)
{
	(void)(fi);
	struct tp_dev *sensor;
	const struct tp_mode_info *mode;
#if ERROR_EXEC	
	s32 frame_rate;
#endif	
	s32 ret = 0;
	
	sensor = to_tp_dev(sd);
	ret = 0;
	
	mutex_lock(&sensor->lock);

	mode = sensor->current_mode;

#if 0
	frame_rate = tp_try_frame_interval(sensor, &fi->interval,
					       mode->hact, mode->vact);
#endif
#if ERROR_EXEC	
	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}
#endif

	mode = tp_find_mode((s32)mode->hact,(s32)mode->vact);
#if 0
	mode = tp_find_mode(sensor, frame_rate, mode->hact,
				mode->vact, true);
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

static s32 tp_s_stream(struct v4l2_subdev *sd, s32 enable)
{
	struct tp_dev *sensor = to_tp_dev(sd);

	s32 ret = 0;

	mutex_lock(&sensor->lock);

	if ((s32)sensor->streaming != enable) {
		ret = 0;

#if ERROR_EXEC
		if (ret) {
			dev_err(&client->dev, "Not support WxH@fps=%dx%d@%d\n",
				sensor->current_mode->hact,
				sensor->current_mode->vact,
				tp_framerates[sensor->current_fr]);
			goto out;
		}
#endif		
		sensor->ep.bus_type = V4L2_MBUS_CSI2_DPHY;
#if 0		
		tp_set_stream_mipi(sensor, enable);
#endif		
		sensor->streaming = enable;
	}
#if ERROR_EXEC
out:
#endif	
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops tp_core_ops = {
	.s_power           = (func_type_subdev)(&tp_s_power),
	.log_status        = &v4l2_ctrl_subdev_log_status,
	.subscribe_event   = &v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = &v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops tp_video_ops = {
	.g_frame_interval  = (func_type_interval)(&tp_g_frame_interval),
	.s_frame_interval  = (func_type_interval)(&tp_s_frame_interval),
	.s_stream          = &tp_s_stream,
};

static const struct v4l2_subdev_pad_ops tp_pad_ops = {
	.enum_mbus_code      = NULL,
	.get_fmt             = (func_type_getfmt)(&tp_get_fmt),
	.set_fmt             = (func_type_setfmt)(&tp_set_fmt),
	.enum_frame_size     = (func_type_framesize)(&tp_enum_frame_size),
	.enum_frame_interval = (func_type_frameinterval)(&tp_enum_frame_interval),
};

static const struct v4l2_subdev_ops tp_subdev_ops = {
	.core  = &tp_core_ops,
	.video = &tp_video_ops,
	.pad   = &tp_pad_ops,
};

static s32 tp_link_setup(const struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	(void)(entity);
	(void)(local);
	(void)(remote);
	(void)(flags);	
	return 0;
}

static const struct media_entity_operations tp_sd_media_ops = {
	.link_setup = (func_type_linksetup)(&tp_link_setup),
};


static ssize_t value_store_tp(const struct device *dev,
		const struct device_attribute *attr, const sc8* buf, size_t size)
{
	(void)(dev);
	(void)(attr);
	(void)(size);

	ssize_t status;

 #if  APPL_USE_PWR 	

    if ((s32)(buf[0]) == 0) {
		pr_err(" %s MIPI off   \n", __func__);
			
		(void)tp2860_write_reg(0x40, 0x08); //select MIPI page
		/* Disable MIPI CSI2 output */
		(void)tp2860_write_reg(0x28, 0x02);	 //stream off 
		(void)tp2860_write_reg(0x40, 0x00); //back to decoder page
	}
    else if ((s32)(buf[0]) == 1) {
		pr_err(" %s MIPI on   \n", __func__);
		/* Enable MIPI CSI2 output */
		(void)tp2860_write_reg(0x40, 0x08); //select MIPI page
		(void)tp2860_write_reg(0x28, 0x00);	 //stream on
		(void)tp2860_write_reg(0x40, 0x00); //back to decoder page
	}
#if 0
		 else if(buf[0] == 2)
	   {


// 		         tp2860_dum_reg(); //
//  #if 1          
         pr_err(" %s reset    \n", __func__);

 		        	gpio_set_value(tp_g_reset_gpio,  1); /* MISRA_C_2012_08_04 false_alarm */
                                                   /* MISRA_C_2012_17_03 false_alarm */
	            usleep_range(1000, 2000);

	            gpio_set_value(tp_g_reset_gpio,  0); /* MISRA_C_2012_08_04 false_alarm */
                                                   /* MISRA_C_2012_17_03 false_alarm */
	            usleep_range(20000, 25000);

	            gpio_set_value(tp_g_reset_gpio,  1); /* MISRA_C_2012_08_04 false_alarm */
                                                   /* MISRA_C_2012_17_03 false_alarm */
	            usleep_range(1000, 2000);
//  #endif 
 
	   }	 
	    else if(buf[0] == 3) // on power // pattern display
	   {
         u8 tmp;
	 
         pr_err(" %s power on    \n", __func__);
 	       gpio_set_value(tp_g_pwr_gpio, 1);  	  /* MISRA_C_2012_08_04 false_alarm */
                                                   /* MISRA_C_2012_17_03 false_alarm */
 	       
// #if 0
           pr_err(" %s pattern on   \n", __func__);
	         
	    		 ret = tp2860_write_reg(0x40, 0x08); //select MIPI page
	         ret = tp2860_write_reg(0x28, 0x02);	 //stream off 
	   	   	 tmp = tp2860_read_reg(0x22);
	         ret = tp2860_write_reg(0x22, tmp|0x80);
	         ret = tp2860_write_reg(0x28, 0x00);	 //stream on
	         ret = tp2860_write_reg(0x40, 0x00); //back to decoder page
//#endif
	   }
	    else if(buf[0] == 4) // off power // pattern display
	   {
	    		 u8 tmp;

         pr_err(" %s power off    \n", __func__);

	       gpio_set_value(tp_g_pwr_gpio, 0);  	

//#if 0	    		 
				 pr_err(" %s pattern off   \n", __func__);
	    		 ret = tp2860_write_reg(0x40, 0x08); //select MIPI page
	         ret = tp2860_write_reg(0x28, 0x02);	 //stream off 
	   	   	 tmp = tp2860_read_reg(0x22);
	   	   	 tmp &= 0x7f;
	         ret = tp2860_write_reg(0x22, tmp);
	         ret = tp2860_write_reg(0x28, 0x00);	 //stream on
	         ret = tp2860_write_reg(0x40, 0x00); //back to decoder page
//#endif
	   }
//#if 0	   
	    else if(buf[0] == 5)
	   	{
	   		  mipi_stream_off();
	   		  if( buf[1] == 0)
	        {    		   
	   		    ret = tp2860_write_reg(0x40, 0x00); //back to decoder page
	   		    ret = tp2860_write_reg(buf[2], buf[3]);
		        pr_err(" decoder reg 0x%02x   =>   value 0x%02x\n", buf[2], tp2860_read_reg(buf[2]));
         	 }
         	 else 
         	 {
 	    		  ret = tp2860_write_reg(0x40, 0x08); //select MIPI page
	   		    ret = tp2860_write_reg(buf[2], buf[3]);
		        pr_err("mipi reg 0x%02x   =>   value 0x%02x\n",buf[2], tp2860_read_reg(buf[2]));
	   		    ret = tp2860_write_reg(0x40, 0x00); //back to decoder page
         	 	
         	 	
         	 }  		    
	   		
	   	}
	    else if(buf[0] == 6)
	   	{
	   		  mipi_stream_off();
	   		  if( buf[1] == 0)
	        {    		   
	   		    ret = tp2860_write_reg(0x40, 0x00); //back to decoder page
		        pr_err(" decoder reg 0x%02x   =>   value 0x%02x\n",buf[2], tp2860_read_reg(buf[2]));
	   		  }  
	   		  else 
	   		  {
 	    		  ret = tp2860_write_reg(0x40, 0x08); //select MIPI page
		        pr_err(" mipi reg 0x%02x   =>   value 0x%02x\n",buf[2], tp2860_read_reg(buf[2]));
	   		    ret = tp2860_write_reg(0x40, 0x00); //back to decoder page
	   		  	
	   		  }
	   		
	   	}
#endif	
     else {
		/* do nothing */
	 }
	   
#endif

	status = 1;
	return status;
} 



static s32 tp_probe_sub1(struct i2c_client *client, struct tp_dev *sensor)
{
	struct device *dev ;
	struct fwnode_handle *endpoint;
	u32 rotation;
	s32 ret = 0;

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
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
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

  
static s32 tp_probe_sub2(struct i2c_client *client ,struct tp_dev *sensor)
{
	
	const struct device *dev; 
	s32 ret;
	u32 pwr_enable_gpio;
	s32 reset_gpio;
	struct device_node *np;
	
	
	
	dev = &client->dev;  
	np = dev->of_node;
	
		pwr_enable_gpio= (u32)of_get_named_gpio(np, "pwr-enable-gpio", 0);

	if (gpio_is_valid(pwr_enable_gpio) > 0) { /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
		
		ret = gpio_request_one(pwr_enable_gpio, GPIOF_OUT_INIT_LOW, "camera-pwr-enable-gpio"); /* MISRA_C_2012_08_04 false_alarm */
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

#if 0	
	 else {
		dev_err(dev, "[CAM] %s: failed to get pwr-enable-gpio\n", __func__);
		return -EINVAL;
	}
#endif

#if 0
  tp_g_pwr_gpio = pwr_enable_gpio;
#endif
 #if 0 
  sensor->pwr_gpio = pwr_enable_gpio;
#endif 


  reset_gpio= of_get_named_gpio(np, "reset-gpio", 0);/* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */

	if (gpio_is_valid(reset_gpio) > 0) {
		ret = gpio_request_one(reset_gpio, GPIOF_OUT_INIT_LOW, "camera-reset-gpio"); /* MISRA_C_2012_08_04 false_alarm */
                                                                                              /* MISRA_C_2012_10_01 false_alarm */
                                               		                                        /* MISRA_C_2012_17_03 false_alarm */
		if (ret == 0) {
			gpio_set_value(reset_gpio, 1); /* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
			dev_info(dev, "[CAM] %s: camera reset enabled\n", __func__);
			gpio_free(reset_gpio);/* MISRA_C_2012_08_04 false_alarm */
		                                        /* MISRA_C_2012_17_03 false_alarm */
		}
	}
#if 0
	else {
		dev_err(dev, "[CAM] %s: failed to get reset-gpio\n", __func__);
		return -EINVAL;
	}
#endif

#if 0
  tp_g_reset_gpio = reset_gpio;
#endif   
  	sensor->reset_gpio = reset_gpio;

   return ret;
}


   

static s32 tp_probe_sub3(struct i2c_client *client, struct tp_dev *sensor)	
{
	
	

	static struct device_attribute dev_attr_value_tp = \
		__ATTR(value, 0660, NULL, (func_type_storetp)(&value_store_tp));  /* MISRA_C_2012_11_01 false_alarm */
               		                                                     /* MISRA_C_2012_07_01 false_alarm */
 
	
	static const struct attribute* gpio_attrs_tp[2] = {  /* MISRA_C_2012_08_13 false_alarm */
		&dev_attr_value_tp.attr,
		NULL,                                               /* MISRA_C_2012_11_05 false_alarm */
	};
	 
	static const struct attribute_group gpio_group_tp = {
		.attrs = (struct attribute **)(&gpio_attrs_tp[0]), /* MISRA_C_2012_11_08 false_alarm */
		                                                     /* MISRA_C_2012_11_03 false_alarm */   
	};

	const struct device *dev;
	s32 ret;


  dev = &client->dev;
	v4l2_i2c_subdev_init(&sensor->sd, client, &tp_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &tp_sd_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
#if 0
	if (ret)
		return ret;
#endif
#if 0
	ret = tp_get_regulators(sensor);
	if (ret)
		return ret;
#endif

	mutex_init(&sensor->lock);

	ret = tp_init_controls(sensor);
#if 0
	if (ret)
		goto entity_cleanup;
#endif

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
#if 0
	if (ret)
		goto free_ctrls;
#endif
	dev_info(dev, "tpcam : i8452 probe success !\n");

#if APPL_USE_PWR	

	ret = sysfs_create_group(&client->dev.kobj, &gpio_group_tp);
	                           /* DAPA_CWE_628 false_alarm */
	                           /* MISRA_C_2012_11_03 false_alarm */

#if ERROR_EXEC	
	if(ret < 0) {
		dev_err(dev, "failed to create sysfs files\n");
	}
#endif	
#endif


return ret;
}







static s32 tp_probe(struct i2c_client *client)
{
	struct device *dev;
	struct tp_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	s32 ret = 0;

	dev = &client->dev; 

 
	sensor = (struct tp_dev *)devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);  /*DAPA_CWE_628 */
#if ERROR_EXEC
	if (!sensor)
		return -ENOMEM;
#endif

	sensor->i2c_client = client;
  	tp_g_sensor = sensor;

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

#if m_1920
	fmt->width = 1920;
	fmt->height = 1080;
#else
	fmt->width = 1280;
	fmt->height = 720;
#endif
  
	fmt->field = (u32)V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = (u32)tp_framerates[TP_FPS];
	sensor->current_fr = TP_FPS;
	sensor->current_mode = &tp_mode_data[TP_MODE];
	sensor->ae_target = 52;



  ret = tp_probe_sub1(client,sensor);	
  ret = tp_probe_sub2(client,sensor);	
  ret = tp_probe_sub3(client,sensor);	


	gpio_set_value(sensor->reset_gpio,  1);
	usleep_range(1000, 2000);

	gpio_set_value(sensor->reset_gpio,  0);
	usleep_range(20000, 25000);

	gpio_set_value(sensor->reset_gpio,  1);
	usleep_range(1000, 2000);

 	tp2860_sensor_init(VIN1);
#if 0
 tp2860_sensor_init(VIN1,HD60,STD_TVI,MIPI_2LANE);
#endif  
 	pr_err("%s  OK \n", __func__);

	return ret;
#if ERROR_EXEC
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
#endif	
}
#if 0
static s32 tp_remove(struct i2c_client *client)
{
#if 0	
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tp_dev *sensor = to_tp_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);
#endif
	return 0;
}
#endif

#if 1
static const struct i2c_device_id tp_id[] = {
	{"tpcam", 0},   /* MISRA_C_2012_08_09 false_alarm */
	{},
};
MODULE_DEVICE_TABLE(i2c, tp_id);  /* MISRA_C_2012_08_01 false_alarm */
                                     /* MISRA_C_2012_08_06 false_alarm */
                                     /* MISRA_C_2012_08_02 false_alarm */
#endif

static const struct of_device_id tp_dt_ids[] = {
	{ .compatible = "tpcam,i8452" }, /* MISRA_C_2012_08_09 false_alarm */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tp_dt_ids); /* MISRA_C_2012_08_01 false_alarm */
                                     /* MISRA_C_2012_08_02 false_alarm */
 
static struct i2c_driver tp_i2c_driver = {
	.driver = {
		.name  = "tpcam",
		.of_match_table	= tp_dt_ids,
	},  
	.id_table  = tp_id,
	.probe_new = tp_probe,
	.remove    = NULL,
};
#if 0
module_i2c_driver(tp_i2c_driver);
#endif


static s32 __init module_begin_tp(void){  /* MISRA_C_2012_02_08 false_alarm */
  return i2c_add_driver(&tp_i2c_driver);      /* MISRA_C_2012_08_04 false_alarm */
                                              /* MISRA_C_2012_17_03 false_alarm */
}
 
static void __exit module_end_tp(void){
  i2c_del_driver(&tp_i2c_driver);
  return ;
	
}
 
module_init(module_begin_tp);     /* MISRA_C_2012_08_01 false_alarm */
                                  /* MISRA_C_2012_08_02 false_alarm */
                                  /* MISRA_C_2012_08_06 false_alarm */
module_exit(module_end_tp);     /* MISRA_C_2012_08_01 false_alarm */
                                  /* MISRA_C_2012_08_02 false_alarm */
                                  /* MISRA_C_2012_08_06 false_alarm */




MODULE_DESCRIPTION("tpcam MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
   
