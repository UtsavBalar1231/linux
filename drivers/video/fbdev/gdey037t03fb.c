#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/rmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include "gdey037t03fb.h"

/* GDEY037T03 Display Commands */
#define GDE_PANEL_SETTING             0x00
#define GDE_POWER_SETTING             0x01
#define GDE_POWER_OFF                 0x02
#define GDE_POWER_ON                  0x04
#define GDE_BOOSTER_SOFT_START        0x06
#define GDE_DEEP_SLEEP                0x07
#define GDE_DTM1                      0x10    /* Data Start Transmission 1 */
#define GDE_DISPLAY_REFRESH           0x12
#define GDE_DTM2                      0x13    /* Data Start Transmission 2 */
#define GDE_VCOM_LUT                  0x20    /* VCOM LUT */
#define GDE_W2W_LUT                   0x21    /* White to White LUT */
#define GDE_B2W_LUT                   0x22    /* Black to White LUT */
#define GDE_W2B_LUT                   0x23    /* White to Black LUT */
#define GDE_B2B_LUT                   0x24    /* Black to Black LUT */
#define GDE_PLL_CONTROL               0x30    /* PLL Control (frequency) */
#define GDE_TEMP_SENSOR_CALIBRATION   0x40
#define GDE_TEMP_SENSOR_ENABLE        0x41
#define GDE_TEMP_SENSOR_WRITE         0x42
#define GDE_TEMP_SENSOR_READ          0x43
#define GDE_VCOM_AND_DATA_INTERVAL    0x50
#define GDE_RESOLUTION_SETTING        0x61
#define GDE_GSST_SETTING              0x65
#define GDE_REVISION                  0x70
#define GDE_GET_STATUS                0x71
#define GDE_AUTO_MEASUREMENT_VCOM     0x80
#define GDE_READ_VCOM_VALUE           0x81
#define GDE_VCM_DC_SETTING            0x82
#define GDE_PARTIAL_WINDOW            0x90
#define GDE_PARTIAL_IN                0x91
#define GDE_PARTIAL_OUT               0x92
#define GDE_PROGRAM_MODE              0xA0
#define GDE_ACTIVE_PROGRAM            0xA1
#define GDE_READ_OTP_DATA             0xA2
#define GDE_POWER_SAVING              0xE3
#define GDE_LUT_OPTION                0xE0
#define GDE_PARTIAL_LUT_OPTION        0xE5

#define GDE_FB_SET_SCREEN_DEEP_SLEEP  _IOW('M', 1, int8_t)
#define GDE_FB_SET_SCREEN_WAKEUP      _IOW('M', 2, int8_t)
#define GDE_FB_SET_SCREEN_PARTIAL_UPDATE _IOW('M', 3, int8_t)
#define GDE_FB_SET_SCREEN_FAST_UPDATE _IOW('M', 4, int8_t)

#define GDEY037T03_WIDTH  240
#define GDEY037T03_HEIGHT 416

struct gde_eink_device_properties {
	unsigned int width;
	unsigned int height;
	unsigned int bpp;
};

struct gde_eink_fb_par {
	struct spi_device *spi;
	struct fb_info *info;
	int rst;
	int dc;
	int busy;
	const struct gde_eink_device_properties *props;
	u8 *old_image; /* Buffer to store old image for partial updates */
};

/* LUT for GDEY037T03 - standard */
static const u8 lut_vcom[] = {
    0x01, 0x0a, 0x0a, 0x0a, 0x0a, 0x01, 0x01,
    0x02, 0x0f, 0x01, 0x0f, 0x01, 0x01, 0x01,
    0x01, 0x0a, 0x00, 0x0a, 0x00, 0x01, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const u8 lut_ww[] = {
    0x01, 0x4a, 0x4a, 0x0a, 0x0a, 0x01, 0x01,
    0x02, 0x8f, 0x01, 0x4f, 0x01, 0x01, 0x01,
    0x01, 0x8a, 0x00, 0x8a, 0x00, 0x01, 0x01,
    0x01, 0x80, 0x00, 0x80, 0x00, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const u8 lut_bw[] = {
    0x01, 0x4a, 0x4a, 0x0a, 0x0a, 0x01, 0x01,
    0x02, 0x8f, 0x01, 0x4f, 0x01, 0x01, 0x01,
    0x01, 0x8a, 0x00, 0x8a, 0x00, 0x01, 0x01,
    0x01, 0x80, 0x00, 0x80, 0x00, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const u8 lut_wb[] = {
    0x01, 0x0a, 0x0a, 0x8a, 0x8a, 0x01, 0x01,
    0x02, 0x8f, 0x01, 0x4f, 0x01, 0x01, 0x01,
    0x01, 0x4a, 0x00, 0x4a, 0x00, 0x01, 0x01,
    0x01, 0x40, 0x00, 0x40, 0x00, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const u8 lut_bb[] = {
    0x01, 0x0a, 0x0a, 0x8a, 0x8a, 0x01, 0x01,
    0x02, 0x8f, 0x01, 0x4f, 0x01, 0x01, 0x01,
    0x01, 0x4a, 0x00, 0x4a, 0x00, 0x01, 0x01,
    0x01, 0x40, 0x00, 0x40, 0x00, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static int gde_eink_send_cmd(struct gde_eink_fb_par *par, u8 cmd, const u8 *data,
				size_t data_size)
{
	int ret;
	struct spi_device *spi = par->spi;
	struct device *dev = &spi->dev;

	gpio_set_value_cansleep(par->dc, 0);
	ret = spi_write(spi, &cmd, 1);
	if (ret) {
		dev_err(dev,
			"SPI write failure (%d) while sending command (%d)",
			ret, cmd);
		return ret;
	}

	if (data_size > 0) {
		gpio_set_value_cansleep(par->dc, 1);
		ret = spi_write(spi, data, data_size);
		if (ret)
			dev_err(dev,
				"SPI write failure (%d) while sending data for command (%d)",
				ret, cmd);
	}

	return ret;
}

static void wait_until_idle(struct gde_eink_fb_par *par)
{
	int retry_count = 0;
	const int max_retries = 100;
	
	while (gpio_get_value_cansleep(par->busy) == 0 && retry_count < max_retries) {
		mdelay(10);
		retry_count++;
	}
	
	if (retry_count >= max_retries) {
		dev_warn(&par->spi->dev, "Busy wait timeout after %d retries\n", retry_count);
	}
}

static void gde_eink_reset(struct gde_eink_fb_par *par)
{
	gpio_set_value_cansleep(par->rst, 0);
	mdelay(10);
	gpio_set_value_cansleep(par->rst, 1);
	mdelay(10);
}

static int set_lut(struct gde_eink_fb_par *par)
{
	int ret;
	
	ret = gde_eink_send_cmd(par, GDE_VCOM_LUT, lut_vcom, sizeof(lut_vcom));
	if (ret)
		return ret;
		
	ret = gde_eink_send_cmd(par, GDE_W2W_LUT, lut_ww, sizeof(lut_ww));
	if (ret)
		return ret;
		
	ret = gde_eink_send_cmd(par, GDE_B2W_LUT, lut_bw, sizeof(lut_bw));
	if (ret)
		return ret;
		
	ret = gde_eink_send_cmd(par, GDE_W2B_LUT, lut_wb, sizeof(lut_wb));
	if (ret)
		return ret;
		
	ret = gde_eink_send_cmd(par, GDE_B2B_LUT, lut_bb, sizeof(lut_bb));
	if (ret)
		return ret;
		
	return 0;
}

static int gde_eink_sleep(struct gde_eink_fb_par *par)
{
	int ret;
	const u8 data = 0xA5; /* Required parameter for sleep command */
	
	ret = gde_eink_send_cmd(par, GDE_POWER_OFF, NULL, 0);
	if (ret)
		return ret;
		
	wait_until_idle(par);
	
	ret = gde_eink_send_cmd(par, GDE_DEEP_SLEEP, &data, 1);
	if (ret)
		return ret;

	return 0;
}

static int gde_eink_wakeup(struct gde_eink_fb_par *par)
{
	gde_eink_reset(par);
	return 0;
}

/* Initialize display for full update mode */
static int gde_eink_init_display(struct gde_eink_fb_par *par)
{
	int ret;
	struct device *dev = &par->spi->dev;
	
	dev_dbg(dev, "Initializing display for full update\n");
	
	gde_eink_reset(par);
	
	/* Power on sequence */
	ret = gde_eink_send_cmd(par, GDE_POWER_ON, NULL, 0);
	if (ret)
		return ret;
	wait_until_idle(par);

	/* Panel setting */
	{
		const u8 data[] = { 0xF7 }; /* Panel setting */
		ret = gde_eink_send_cmd(par, GDE_PANEL_SETTING, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* Power setting */
	{
		const u8 data[] = { 0x03, 0x10, 0x3F, 0x3F, 0x3F };
		ret = gde_eink_send_cmd(par, GDE_POWER_SETTING, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* Booster soft start */
	{
		const u8 data[] = { 0xD7, 0xD7, 0x33 };
		ret = gde_eink_send_cmd(par, GDE_BOOSTER_SOFT_START, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* PLL control */
	{
		const u8 data[] = { 0x09 }; /* 150Hz */
		ret = gde_eink_send_cmd(par, GDE_PLL_CONTROL, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* VCOM and data interval setting */
	{
		const u8 data[] = { 0xD7 };
		ret = gde_eink_send_cmd(par, GDE_VCOM_AND_DATA_INTERVAL, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* Resolution setting */
	{
		const u8 data[] = { 0xF0, 0x01, 0xA0 }; /* 240x416 */
		ret = gde_eink_send_cmd(par, GDE_RESOLUTION_SETTING, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* VCM DC setting */
	{
		const u8 data[] = { 0x0F };
		ret = gde_eink_send_cmd(par, GDE_VCM_DC_SETTING, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* Flash start/end positions */
	{
		const u8 data[] = { 0x80, 0x00, 0x00, 0xFF, 0x00 };
		ret = gde_eink_send_cmd(par, 0x2A, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* Set LUT */
	ret = set_lut(par);
	if (ret)
		return ret;
		
	/* Allocate buffer for old image if not already allocated */
	if (!par->old_image) {
		par->old_image = devm_kzalloc(dev, par->props->width * par->props->height / 8, GFP_KERNEL);
		if (!par->old_image)
			return -ENOMEM;
	}
	
	return 0;
}

/* Initialize display for fast update mode */
static int gde_eink_init_display_fast(struct gde_eink_fb_par *par)
{
	int ret;
	struct device *dev = &par->spi->dev;
	
	dev_dbg(dev, "Initializing display for fast update\n");
	
	gde_eink_reset(par);
	
	/* Power on sequence */
	ret = gde_eink_send_cmd(par, GDE_POWER_ON, NULL, 0);
	if (ret)
		return ret;
	wait_until_idle(par);

	/* LUT option */
	{
		const u8 data[] = { 0x02 };
		ret = gde_eink_send_cmd(par, GDE_LUT_OPTION, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* Partial LUT option */
	{
		const u8 data[] = { 0x5A }; /* 1.5s */
		ret = gde_eink_send_cmd(par, GDE_PARTIAL_LUT_OPTION, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	return 0;
}

/* Initialize display for partial update mode */
static int gde_eink_init_display_partial(struct gde_eink_fb_par *par)
{
	int ret;
	struct device *dev = &par->spi->dev;
	
	dev_dbg(dev, "Initializing display for partial update\n");
	
	gde_eink_reset(par);
	
	/* Power on sequence */
	ret = gde_eink_send_cmd(par, GDE_POWER_ON, NULL, 0);
	if (ret)
		return ret;
	wait_until_idle(par);

	/* LUT option */
	{
		const u8 data[] = { 0x02 };
		ret = gde_eink_send_cmd(par, GDE_LUT_OPTION, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* Partial LUT option */
	{
		const u8 data[] = { 0x6E };
		ret = gde_eink_send_cmd(par, GDE_PARTIAL_LUT_OPTION, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	/* VCOM and data interval setting */
	{
		const u8 data[] = { 0xD7 };
		ret = gde_eink_send_cmd(par, GDE_VCOM_AND_DATA_INTERVAL, data, sizeof(data));
		if (ret)
			return ret;
	}
	
	return 0;
}

static int clear_frame_memory(struct gde_eink_fb_par *par, u8 color)
{
	int ret;
	u32 buffer_size = par->props->width * par->props->height / 8;
	u8 *buffer;
	
	buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	
	memset(buffer, color, buffer_size);
	
	/* Send old data - all zeroes */
	ret = gde_eink_send_cmd(par, GDE_DTM1, par->old_image, buffer_size);
	if (ret) {
		kfree(buffer);
		return ret;
	}
	
	/* Send new data - all color */
	ret = gde_eink_send_cmd(par, GDE_DTM2, buffer, buffer_size);
	if (ret) {
		kfree(buffer);
		return ret;
	}
	
	/* Update the old image buffer */
	memcpy(par->old_image, buffer, buffer_size);
	kfree(buffer);
	
	/* Refresh display */
	ret = gde_eink_send_cmd(par, GDE_DISPLAY_REFRESH, NULL, 0);
	if (ret)
		return ret;
		
	wait_until_idle(par);
	
	return 0;
}

static int gde_eink_update_display(struct gde_eink_fb_par *par)
{
	int ret;
	struct fb_info *info = par->info;
	u32 buffer_size = par->props->width * par->props->height / 8;
	
	/* Send old data */
	ret = gde_eink_send_cmd(par, GDE_DTM1, par->old_image, buffer_size);
	if (ret)
		return ret;
	
	/* Send new data */
	ret = gde_eink_send_cmd(par, GDE_DTM2, (u8 *)info->screen_buffer, buffer_size);
	if (ret)
		return ret;
	
	/* Save new image as old image for next update */
	memcpy(par->old_image, info->screen_buffer, buffer_size);
	
	/* Refresh display */
	ret = gde_eink_send_cmd(par, GDE_DISPLAY_REFRESH, NULL, 0);
	if (ret)
		return ret;
		
	wait_until_idle(par);
	
	return 0;
}

static void gde_eink_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct gde_eink_fb_par *par = info->par;
	sys_fillrect(info, rect);
	gde_eink_update_display(par);
}

static void gde_eink_fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct gde_eink_fb_par *par = info->par;
	sys_copyarea(info, area);
	gde_eink_update_display(par);
}

static void gde_eink_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct gde_eink_fb_par *par = info->par;
	sys_imageblit(info, image);
	gde_eink_update_display(par);
}

static ssize_t gde_eink_fb_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;
	struct gde_eink_fb_par *par = info->par;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->screen_size;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void __force *)(info->screen_buffer + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if (!err)
		*ppos += count;

	gde_eink_update_display(par);

	return (err) ? err : count;
}

static int gde_eink_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct gde_eink_fb_par *par = info->par;
	
	switch (cmd) {
	case GDE_FB_SET_SCREEN_DEEP_SLEEP:
		ret = gde_eink_sleep(par);
		break;
	case GDE_FB_SET_SCREEN_WAKEUP:
		ret = gde_eink_wakeup(par);
		ret = gde_eink_init_display(par);
		break;
	case GDE_FB_SET_SCREEN_PARTIAL_UPDATE:
		ret = gde_eink_init_display_partial(par);
		break;
	case GDE_FB_SET_SCREEN_FAST_UPDATE:
		ret = gde_eink_init_display_fast(par);
		break;
	default:
		ret = -EINVAL;
	}
	
	return ret;
}

static struct fb_ops gde_eink_ops = {
	.owner = THIS_MODULE,
	.fb_read = fb_sys_read,
	.fb_write = gde_eink_fb_write,
	.fb_fillrect = gde_eink_fb_fillrect,
	.fb_copyarea = gde_eink_fb_copyarea,
	.fb_imageblit = gde_eink_fb_imageblit,
	.fb_ioctl = gde_eink_fb_ioctl,
};

static struct gde_eink_device_properties gdey037t03_props = {
	.width = GDEY037T03_WIDTH,
	.height = GDEY037T03_HEIGHT,
	.bpp = 1,
};

static struct spi_device_id gde_eink_tbl[] = {
	{ "gdey037t03", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, gde_eink_tbl);

static void gde_eink_deferred_io(struct fb_info *info,
				  struct list_head *pagelist)
{
	struct gde_eink_fb_par *par = info->par;
	gde_eink_update_display(par);
}

static struct fb_deferred_io gde_eink_defio = {
	.delay = HZ,
	.deferred_io = gde_eink_deferred_io,
};

static int gde_eink_spi_probe(struct spi_device *spi)
{
	struct fb_info *info;
	int retval = 0;
	struct gde_eink_platform_data *pdata;
	struct gde_eink_fb_par *par;
	u8 *vmem;
	
	dev_dbg(&spi->dev, "Probing for GDEY037T03 device\n");

	pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&spi->dev, "Failed to allocate memory for platform data\n");
		return -ENOMEM;
	}
	
	/* Get GPIO information from device tree */
	pdata->rst_gpio = of_get_named_gpio(spi->dev.of_node, "reset-gpios", 0);
	pdata->dc_gpio = of_get_named_gpio(spi->dev.of_node, "dc-gpios", 0);
	pdata->busy_gpio = of_get_named_gpio(spi->dev.of_node, "busy-gpios", 0);
	
	if (!gpio_is_valid(pdata->rst_gpio) || !gpio_is_valid(pdata->dc_gpio) || 
	    !gpio_is_valid(pdata->busy_gpio)) {
		dev_err(&spi->dev, "Invalid GPIO pins\n");
		return -EINVAL;
	}
	
	info = framebuffer_alloc(sizeof(struct gde_eink_fb_par), &spi->dev);
	if (!info) {
		dev_err(&spi->dev, "Failed to allocate framebuffer memory\n");
		return -ENOMEM;
	}

	par = info->par;
	par->spi = spi;
	par->info = info;
	par->rst = pdata->rst_gpio;
	par->dc = pdata->dc_gpio;
	par->busy = pdata->busy_gpio;
	par->props = &gdey037t03_props;
	
	vmem = vzalloc(gdey037t03_props.width * gdey037t03_props.height / 8);
	if (!vmem) {
		dev_err(&spi->dev, "Failed to allocate video memory\n");
		retval = -ENOMEM;
		goto fballoc_fail;
	}

	info->screen_buffer = vmem;
	info->fbops = &gde_eink_ops;
	info->fix = (struct fb_fix_screeninfo) {
		.id = "GDEY037T03",
		.type = FB_TYPE_PACKED_PIXELS,
		.visual = FB_VISUAL_MONO01,
		.xpanstep = 0,
		.ypanstep = 0,
		.ywrapstep = 0,
		.line_length = gdey037t03_props.width / 8,
		.accel = FB_ACCEL_NONE,
	};
	
	info->var = (struct fb_var_screeninfo) {
		.xres = gdey037t03_props.width,
		.yres = gdey037t03_props.height,
		.xres_virtual = gdey037t03_props.width,
		.yres_virtual = gdey037t03_props.height,
		.bits_per_pixel = 1,
		.grayscale = 1,
		.red.offset = 0,
		.red.length = 1,
		.red.msb_right = 0,
		.green.offset = 0,
		.green.length = 1,
		.green.msb_right = 0,
		.blue.offset = 0,
		.blue.length = 1,
		.blue.msb_right = 0,
		.transp.offset = 0,
		.transp.length = 0,
		.transp.msb_right = 0,
	};
	
	info->flags = FBINFO_VIRTFB;
	info->fbdefio = &gde_eink_defio;
	fb_deferred_io_init(info);
	
	retval = devm_gpio_request_one(&spi->dev, par->rst,
				      GPIOF_OUT_INIT_HIGH, "gdey037t03-reset");
	if (retval) {
		dev_err(&spi->dev, "Failed to request reset GPIO %d\n", par->rst);
		goto vidalloc_fail;
	}
	
	retval = devm_gpio_request_one(&spi->dev, par->dc,
				      GPIOF_OUT_INIT_LOW, "gdey037t03-dc");
	if (retval) {
		dev_err(&spi->dev, "Failed to request DC GPIO %d\n", par->dc);
		goto vidalloc_fail;
	}
	
	retval = devm_gpio_request_one(&spi->dev, par->busy,
				      GPIOF_IN, "gdey037t03-busy");
	if (retval) {
		dev_err(&spi->dev, "Failed to request busy GPIO %d\n", par->busy);
		goto vidalloc_fail;
	}
	
	/* Allocate buffer for old image */
	par->old_image = devm_kzalloc(&spi->dev, gdey037t03_props.width * gdey037t03_props.height / 8, GFP_KERNEL);
	if (!par->old_image) {
		dev_err(&spi->dev, "Failed to allocate memory for old image buffer\n");
		retval = -ENOMEM;
		goto vidalloc_fail;
	}
	
	retval = gde_eink_init_display(par);
	if (retval) {
		dev_err(&spi->dev, "Failed to initialize display\n");
		goto vidalloc_fail;
	}
	
	retval = clear_frame_memory(par, 0xff); /* Clear to white */
	if (retval) {
		dev_err(&spi->dev, "Failed to clear display\n");
		goto vidalloc_fail;
	}
	
	retval = register_framebuffer(info);
	if (retval) {
		dev_err(&spi->dev, "Failed to register framebuffer\n");
		goto vidalloc_fail;
	}
	
	spi_set_drvdata(spi, info);
	
	dev_info(&spi->dev, "GDEY037T03 e-ink framebuffer device registered, %dx%d, %d KiB video memory\n", 
		 gdey037t03_props.width, gdey037t03_props.height, 
		 gdey037t03_props.width * gdey037t03_props.height / 8 / 1024);
	
	return 0;
	
vidalloc_fail:
	vfree(vmem);
fballoc_fail:
	framebuffer_release(info);
	return retval;
}

static void gde_eink_spi_remove(struct spi_device *spi)
{
	struct fb_info *p = spi_get_drvdata(spi);
	struct gde_eink_fb_par *par = p->par;
	
	/* Put display in sleep mode */
	gde_eink_sleep(par);
	
	unregister_framebuffer(p);
	fb_deferred_io_cleanup(p);
	vfree(p->screen_buffer);
	framebuffer_release(p);
}

#ifdef CONFIG_OF
static const struct of_device_id gde_eink_dt_ids[] = {
	{ .compatible = "gooddisplay,gdey037t03" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gde_eink_dt_ids);
#endif

static struct spi_driver gde_eink_driver = {
	.driver = {
		.name = "gdey037t03",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gde_eink_dt_ids),
	},
	.id_table = gde_eink_tbl,
	.probe = gde_eink_spi_probe,
	.remove = gde_eink_spi_remove,
};

module_spi_driver(gde_eink_driver);

MODULE_DESCRIPTION("Framebuffer driver for GDEY037T03 e-ink display");
MODULE_AUTHOR("Utsav Balar <utsavbalar1231@gmail.com>");
MODULE_LICENSE("GPL"); 
