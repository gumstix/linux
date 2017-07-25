/* OV7692: linux kernel driver for the OmniVision OV7692 Image Sensor 
*
*  Copyright (C) 2017, 
*  Adam YH Lee <adam@gumstix.com> 
*  Jason Liu <jason.liu@gumstix.com>
*  
*  This is a derived work from the following:
*  
*  		-  OV9650/OV9652 by Sylwester Nawrocki
*		-  MT9V032 by Laurent Pinchart
*
*  This is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  
*  You should have received a copy of the GNU General Public License
*  along with OV7692. If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/mutex.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/ov7692.h>

MODULE_DESCRIPTION("OmniVision OV7692 sensor experimental driver");
MODULE_LICENSE("GPL v2");

#define REG_OV7692_MODEL_ID_MSB	0x0A
#define REG_OV7692_MODEL_ID_LSB	0x0B

struct ov7692_ctrls {
	struct v4l2_ctrl_handler handler;

	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};

	u8 update;
};

struct ov7692 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;
	struct v4l2_mbus_framefmt format;
	struct ov7692_ctrls ctrls;
	struct {
		struct v4l2_ctrl *link_freq;
		struct v4l2_ctrl *pixel_rate;
	};
	struct ov7692_platform_data *pdata;
	int streaming;
	int power;

	unsigned int hratio;
	unsigned int vratio;

	struct clk *clk;
	u32 sysclk;
};

static const u8 reset_registers[] = {
       	0x12, 0x80, // Reset
	0xEE, 0x00,
};

static const u8 initial_registers[] = {
	0x0e, 0x08, // Sleep mode
	0x69, 0x52,
	0x1e, 0xb3,
	0x48, 0x42,
	0xff, 0x01,
	0xb5, 0x30,
	0xff, 0x00,
	0x16, 0x03,
	0x62, 0x10,
	0x12, 0x01,
	0x17, 0x65,
	0x18, 0xa4,
	0x19, 0x0c,
	0x1a, 0xf6,
	0x37, 0x04,
	0x3e, 0x20,
	0x81, 0x3f,
	0xcc, 0x02,
	0xcd, 0x80,
	0xce, 0x01,
	0xcf, 0xe0,
	0x82, 0x01,
	0xc8, 0x02,
	0xc9, 0x80,
	0xca, 0x01,
	0xcb, 0xe0,
	0xd0, 0x28,
	0x0e, 0x00,
	0x70, 0x00,
	0x71, 0x34,
	0x74, 0x28,
	0x75, 0x98,
	0x76, 0x00,
	0x77, 0x64,
	0x78, 0x01,
	0x79, 0xc2,
	0x7a, 0x4e,
	0x7b, 0x1f,
	0x7c, 0x00,
	0x11, 0x01,
	0x20, 0x00,
	0x21, 0x57,
	0x50, 0x4d,
	0x51, 0x40,
	0x4c, 0x7d,
	0x0e, 0x00,
	0x80, 0x7f,
	0x85, 0x00,
	0x86, 0x00,
	0x87, 0x00,
	0x88, 0x00,
	0x89, 0x2a,
	0x8a, 0x22,
	0x8b, 0x20,
	0xbb, 0xab,
	0xbc, 0x84,
	0xbd, 0x27,
	0xbe, 0x0e,
	0xbf, 0xb8,
	0xc0, 0xc5,
	0xc1, 0x1e,
	0xb7, 0x05,
	0xb8, 0x09,
	0xb9, 0x00,
	0xba, 0x18,
	0x5a, 0x1f,
	0x5b, 0x9f,
	0x5c, 0x69,
	0x5d, 0x42,
	0x24, 0x78,
	0x25, 0x68,
	0x26, 0xb3,
	0xa3, 0x0b,
	0xa4, 0x15,
	0xa5, 0x29,
	0xa6, 0x4a,
	0xa7, 0x58,
	0xa8, 0x65,
	0xa9, 0x70,
	0xaa, 0x7b,
	0xab, 0x85,
	0xac, 0x8e,
	0xad, 0xa0,
	0xae, 0xb0,
	0xaf, 0xcb,
	0xb0, 0xe1,
	0xb1, 0xf1,
	0xb2, 0x14,
	0x8e, 0x92,
	0x96, 0xff,
	0x97, 0x00,
	0x14, 0x3b,
	0x0e, 0x00,
	0xEE, 0x00,
};

static const u8 start_registers[] = {
	0x0e, 0x00, // Normal mode (no sleep)
	0xEE, 0x00,
};

static const u8 stop_registers[] = {
	0x0e, 0x08, // Sleep mode
	0xEE, 0x00, // 0xEE is a safe marker
};

static const u8 full_registers[] = {
	0xcc, 0x02,
	0xcd, 0x80,
	0xce, 0x01,
	0xcf, 0xe0,
	0xc8, 0x02,
	0xc9, 0x80,
	0xca, 0x01,
	0xcb, 0xe0,
	0x61, 0x60,
	0xEE, 0x00,
};

static inline struct ov7692 *to_ov7692(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7692, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov7692, ctrls.handler)->sd;
}

static int write_regs_i2c(struct i2c_client *client, const u8 *regs)
{
	int i;
	int ret;

	u8 buf[2];

	v4l_info(client, "Writing into i2c..!\n");
	for (i = 0; regs[i] != 0xEE; i += 2) {
		buf[0] = regs[i];
		buf[1] = regs[i+1];

		ret = i2c_master_send(client, buf, 2);
		if (ret < 0) {
			v4l_err(client, "i2c send failed!\n");
			return ret;
		}
	}
	v4l_info(client, "Done writing into i2c..!\n");

	return 0;	
}

static int read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
	*data = i2c_smbus_read_byte_data(client, reg);

	if (data < 0){
		v4l_err(client, "SMBus Read Failed!\n");
			return -1;
	}
	v4l_info(client, "SMBus read  - reg:0x%02x data 0x%02x\n", reg, *data);

	return 0;
}

static void __ov7692_set_power(struct ov7692 *ov7692, int on)
{
	ov7692->streaming = 0;
}

static int ov7692_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov7692 *ov7692 = to_ov7692(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_info(client, "Turning the power on/off: power: %d on: %d\n", ov7692->power, on);

	if (ov7692->power == !on) {

		__ov7692_set_power(ov7692, on);

		if (on) {
			v4l2_info(client, "Resetting...\n");
			write_regs_i2c(client, reset_registers);
			usleep_range(900000, 1200000);
			v4l2_info(client, "Initializing... \n");
			write_regs_i2c(client, initial_registers);
			ov7692->ctrls.update = 1;
		}
	}
		
	ov7692->power += on ? 1: -1;

	v4l2_info(client, "Exiting s_power - power on/off: power: %d on: %d\n", ov7692->power, on);

	return 0;
};

static int ov7692_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SBGGR8_1X8;
	return 0;
}

static int ov7692_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_info(client, "Setting frame sizes... \n");

	fse->code = V4L2_MBUS_FMT_SBGGR8_1X8;
	fse->min_width = 640;
	fse->min_height = 480;
	fse->max_width = 640;
	fse->max_height = 480;

	return 0;
}

static int ov7692_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7692 *ov7692 = to_ov7692(sd);
	struct ov7692_ctrls *ctrls = &ov7692->ctrls;
	int ret = 0;

	v4l_info(client, "ov7692->streaming: %d on: %d\n", ov7692->streaming, on);

	if(ov7692->streaming == !on){
		if(on) {
			usleep_range(900000, 1200000);
			write_regs_i2c(client, reset_registers);
			v4l2_info(client, "Initializing... \n");
			usleep_range(900000, 1200000);
			write_regs_i2c(client, initial_registers);
			usleep_range(900000, 1200000);

			v4l2_info(client, "starting stream\n");
			if (write_regs_i2c(client, start_registers) < 0) {
				v4l_err(client, "err starting a stream\n");
			}
			else
				v4l_info(client, "start_registers written \n");
			usleep_range(900000, 1200000);
		}
		if(ctrls->update) {
			v4l_info(client, "update controls\n");
			ret = v4l2_ctrl_handler_setup(&ctrls->handler);

			if(!ret){
				ctrls->update = 0;
				v4l_info(client, "control update succedded\n");
			}
		}
		
	}	

	usleep_range(100, 200);

	ov7692->streaming += on ? 1 : -1;
	v4l_info(client, "streaming is on/off %d\n", ov7692->streaming);
	WARN_ON(ov7692->streaming < 0);

	return ret;
}

static void ov7692_get_default_format(struct v4l2_mbus_framefmt *mf)
{
	mf->width = 640;
	mf->height = 480;
	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_SRGB;
	mf->code = V4L2_MBUS_FMT_SBGGR8_1X8;
}


static int ov7692_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov7692 *ov7692 = to_ov7692(sd);

	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_info(client, "Getting format... \n");

	fmt->format = ov7692->format;

	v4l2_info(client, "Done getting format... \n");

	return 0;
}

static int ov7692_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov7692 *ov7692 = to_ov7692(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	v4l2_info(client, "Setting format... \n");

	ov7692->format = fmt->format;

	return ret;
}

static const struct v4l2_subdev_core_ops ov7692_core_ops = {
	.s_power = ov7692_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_pad_ops ov7692_pad_ops = {
	.enum_mbus_code = ov7692_enum_mbus_code,
	.enum_frame_size = ov7692_enum_frame_sizes,
	.get_fmt = ov7692_get_fmt,
	.set_fmt = ov7692_set_fmt,
};

static int ov7692_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_info(client, "Setting g_frame rate\n");

	return 0;
}

static int ov7692_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_info(client, "Setting s_frame rate\n");
	
	return 0;	
}

static const struct v4l2_subdev_video_ops ov7692_video_ops = {
	.s_stream = ov7692_s_stream,
	.g_frame_interval = ov7692_g_frame_interval,
	.s_frame_interval = ov7692_s_frame_interval,
};

static const struct v4l2_subdev_ops ov7692_ops = {
	.core = &ov7692_core_ops,
	.pad = &ov7692_pad_ops,
	.video = &ov7692_video_ops,
};

static int ov7692_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *mf = v4l2_subdev_get_try_format(fh, 0);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_info(client, "Getting default format ... \n");
	ov7692_get_default_format(mf);

	v4l2_info(client, "Opening the device ... \n");

	return 0;
}

static int ov7692_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l2_info(client, "Closing the device ... \n");

	return 0;
}

static void ov7692_configure_pixel_rate(struct i2c_client *client, struct ov7692 *ov7692)
{
	int ret;

	ov7692->hratio = 1;
	ret = v4l2_ctrl_s_ctrl_int64(ov7692->pixel_rate,
				     ov7692->sysclk / ov7692->hratio);

	if (ret < 0)
		dev_warn(&client->dev, "failed to set pixel rate (%d)\n", ret);
	else
		dev_info(&client->dev, "Set pixel rate\n");
}

static int ov7692_registered(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7692 *ov7692 = to_ov7692(sd);

	v4l2_info(client, "Registered the device ... \n");

	ov7692_configure_pixel_rate(client, ov7692);

	return 0;
}

static int ov7692_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ov7692 *ov7692 = to_ov7692(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 freq;

	if (ov7692->power == 0) {
		v4l2_info(client, "Power is down, can't apply ctrls\n");
	}

	v4l2_info(client, "Setting controls... \n");

	switch (ctrl->id){
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		v4l2_info(client, "Setting pixel rate\n");

		if (ov7692->link_freq == NULL) {
			v4l2_err(client, "pixel rate / link freq. not available\n");
			break;
		}
		
		freq = ov7692->pdata->link_freqs[ov7692->link_freq->val];
		*ov7692->pixel_rate->p_new.p_s64 = freq;
		ov7692->sysclk = freq;
		v4l2_info(client, "Link frequency being set at %u\n", freq);

		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ov7692_ctrl_ops = {
	.s_ctrl = ov7692_s_ctrl,
};

static int ov7692_initialize_controls(struct ov7692 *ov7692)
{
	const struct v4l2_ctrl_ops *ops = &ov7692_ctrl_ops;
	struct ov7692_ctrls *ctrls = &ov7692->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct v4l2_subdev *sd = &ov7692->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	v4l2_info(client, "Initializing controls... \n");

	ret = v4l2_ctrl_handler_init(hdl, 16);
	if (ret < 0) {
		v4l2_info(client, "v4l2_ctrl_handler_init returns error %d\n", ret);
		return ret;
	}

	ov7692->pixel_rate = v4l2_ctrl_new_std(&ov7692->ctrls.handler, &ov7692_ctrl_ops,
			  V4L2_CID_PIXEL_RATE, 1, INT_MAX, 1, 1);
	
        if (hdl->error) {
		v4l2_info(client, "v4l2_ctrl_new_std_menu_itmes returns error %d\n", ret);
                ret = hdl->error;
                v4l2_ctrl_handler_free(hdl);
                return ret;
        }

        ov7692->sd.ctrl_handler = hdl;
	v4l2_info(client, "ov7692_initialize_controls exiting\n");

        return 0;
}

static const struct v4l2_subdev_internal_ops ov7692_sd_internal_ops = {
	.registered = ov7692_registered,
	.open = ov7692_open,
	.close = ov7692_close,
};

static const s64 ov7692_link_freqs[] = {
	19200000,
	0,
};

static struct ov7692_platform_data *
ov7692_get_pdata(struct i2c_client *client)
{
	struct ov7692_platform_data *pdata;
	struct device_node *np;

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
			return client->dev.platform_data;

	np = of_graph_get_next_endpoint(client->dev.of_node, NULL);
	if (!np)
		return NULL;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto done;

	pdata->clk_pol = 0;
	pdata->link_freqs = ov7692_link_freqs;
	pdata->link_def_freq = 19200000;

done:
	of_node_put(np);
	return pdata;
}

static int ov7692_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov7692_platform_data *pdata = ov7692_get_pdata(client);
	struct ov7692 *ov7692;
	struct i2c_adapter *adapter = client->adapter;
	struct v4l2_subdev *sd;
	struct clk *clk;
	int ret;
	unsigned int i;
	u8 model_id_msb;
	u8 model_id_lsb;
	u16 model_id;	

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		v4l_info(client, "i2c_check_functionality failed\n");
		return -ENODEV;
	}

	clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(clk))
		return -EPROBE_DEFER;

	v4l_info(client, "clock being set at %lld\n", pdata->link_def_freq);
	ret = clk_set_rate(clk, pdata->link_def_freq);
	if (ret < 0)
	{
		v4l_err(client, "clk_set_rate failed\n");
		return ret;
	}
	
	ret = clk_prepare_enable(clk);

	if (ret < 0)
	{
		v4l_err(client, "clk_enable failed\n");
		return ret;
	}

	udelay(1);

	ov7692 = devm_kzalloc(&client->dev, sizeof(*ov7692), GFP_KERNEL);
	if (!ov7692)
		return -ENOMEM;
	sd = &ov7692->sd;

	ov7692->pdata = pdata;

	if (sd == NULL){
		v4l_info(client, "sd == NULL\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(sd, client, &ov7692_ops);
	strlcpy(sd->name, "OV7692", sizeof(sd->name));

	sd->internal_ops = &ov7692_sd_internal_ops;

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	
	client->flags = I2C_CLIENT_SCCB;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr, client->adapter->name);

	/* read model ID */
	read_reg(client, REG_OV7692_MODEL_ID_MSB, &model_id_msb);
	read_reg(client, REG_OV7692_MODEL_ID_LSB, &model_id_lsb);

	model_id = (model_id_msb << 8) | ((model_id_lsb & 0x00FF)) ;

	v4l_info(client, "Model ID: 0x%x, 0x%x, 0x%x\n", model_id, model_id_msb, model_id_lsb);

	ov7692->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	/*	i2c device check	*/
	ret = write_regs_i2c(client, reset_registers);
	if (ret < 0)
	{
		v4l_err(client, "Probe failed\n");
		devm_kfree(&client->dev, (void *)ov7692);
		return ret;
	}

	ret = media_entity_init(&sd->entity, 1, &ov7692->pad, 0); 
	if (ret < 0){
		v4l_err(client, "media entity init failed\n");
		goto err1;
	}	

	ret = ov7692_initialize_controls(ov7692);
	if (ret < 0){
		v4l_err(client, "controls init failed\n");
		goto err1;
	}

	if (pdata && pdata->link_freqs) {
		unsigned int def = 0;

		for (i = 0; pdata->link_freqs[i]; ++i) {
			if (pdata->link_freqs[i] == pdata->link_def_freq)
				def = i;
		}
		ov7692->link_freq = 
			v4l2_ctrl_new_int_menu(&ov7692->ctrls.handler,
						&ov7692_ctrl_ops,
						V4L2_CID_LINK_FREQ, i - 1, def,
						pdata->link_freqs);
		v4l2_ctrl_cluster(2, &ov7692->link_freq);
	}

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0){
		v4l_err(client, "Sub-device registration failed\n");
		goto err2;
	}

	return 0;

err2:
	v4l2_ctrl_handler_free(&ov7692->ctrls.handler);
err1:
	media_entity_cleanup(&sd->entity);
	devm_kfree(&client->dev, (void *)ov7692);
	return ret;
}

static int ov7692_remove(struct i2c_client *client)
{
	if (client != NULL) {
		struct v4l2_subdev *sd = i2c_get_clientdata(client);
		if (sd != NULL) {
			struct ov7692 *ov7692 = to_ov7692(sd);
			if (ov7692 != NULL)
				v4l2_ctrl_handler_free(&ov7692->ctrls.handler);
			v4l2_device_unregister_subdev(sd);			
			media_entity_cleanup(&sd->entity);
		}	
	}
	return 0;
}

static const struct i2c_device_id ov7692_id[] = {
	{ "ov7692", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov7692_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov7692_of_match[] = {
	{ .compatible = "omnivision,ov7692", },
	{	},
};
#endif

static struct i2c_driver ov7692_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ov7692_of_match),
		.name  = "ov7692",
	},
	.probe = ov7692_probe,
	.remove = ov7692_remove,
	.id_table = ov7692_id,
};
module_i2c_driver(ov7692_driver);
