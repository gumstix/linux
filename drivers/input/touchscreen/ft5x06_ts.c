/*
 * Copyright (C) 2010 Focal Tech Ltd.
 * Copyright (C) 2013, Ash Charles <ash@gumstix.com>
 * Copyright (C) 2014, Adam Lee <adam@gumstix.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This is a driver for the FocalTech FT5x0x family of touch controllers
 * use in Newhaven capacitive touchscreens.  Note: this does not support with
 * EDT "Polytouch" controllers.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/input/ft5x06_ts.h>

struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

	int reset_gpio;
	int irq_gpio;
	int wake_gpio;
};

static int ft5x0x_ts_readwrite(struct i2c_client *client, u16 wr_len,
			       u8 *wr_buf, u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

	if (wr_len) {
		wrmsg[i].addr = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}

	if (rd_len) {
		wrmsg[i].addr = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);

	if (ret < 0)
		return ret;

	if (ret != i)
		return -EIO;

	return 0;
}

static int ft5x0x_register_write(struct ft5x0x_ts_data *tsdata,
				 u8 addr, u8 value)
{
	u8 wrbuf[2];

	wrbuf[0] = addr;
	wrbuf[1] = value;

	return ft5x0x_ts_readwrite(tsdata->client, 2, wrbuf, 0, NULL);
}

static int ft5x0x_register_read(struct ft5x0x_ts_data *tsdata, u8 addr)
{
	u8 rdbuf;
	int error;

	error = ft5x0x_ts_readwrite(tsdata->client, 1, &addr, 1, &rdbuf);

	if (error) {
		dev_err(&tsdata->client->dev,
				"Unable to fetch data, error: %d\n",
				error);

		return error;
	}

	return rdbuf;
}

static unsigned char ft5x0x_read_fw_ver(struct ft5x0x_ts_data *tsdata)
{
	return ft5x0x_register_read(tsdata, FT5X0X_REG_FIRMID);
}

static irqreturn_t ft5x0x_ts_isr(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *tsdata = dev_id;

	schedule_delayed_work(&tsdata->work, 0);
	return IRQ_HANDLED;
}
static void ft5x0x_ts_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct ft5x0x_ts_data *tsdata = container_of(dw,
						     struct ft5x0x_ts_data,
						     work);
	struct device *dev = &tsdata->client->dev;
	u8 cmd = 0x00;
	u8 rdbuf[32];
	int i, x, y, id;
	bool pressed;
	int error, type;

	memset(rdbuf, 0, sizeof(rdbuf));
	error = ft5x0x_ts_readwrite(tsdata->client, sizeof(cmd),
				    &cmd, sizeof(rdbuf), rdbuf);

	if (error) {
		dev_err(dev, "Unabled to fetch data, error: %d\n", error);
		return;
	}

	for (i = 0; i < 5; i++) {
		u8 *buf = &rdbuf[i * 6 + 3];

		type = buf[0] >> 6 & 0x3;

		if (type == 0x3)
			continue;

		x = ((buf[0] << 8) | buf[1]) & 0x0fff;
		y = ((buf[2] << 8) | buf[3]) & 0x0fff;
		id = (buf[2] >> 4) & 0x0f;
		pressed = (type != 0x01);
		input_mt_slot(tsdata->input, id);
		input_mt_report_slot_state(tsdata->input,
					   MT_TOOL_FINGER,
					   pressed);

		if (!pressed)
			continue;

		input_report_abs(tsdata->input, ABS_MT_POSITION_X, x);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y);
	}

	input_mt_report_pointer_emulation(tsdata->input, true);
	input_sync(tsdata->input);
}

static int ft5x0x_ts_reset(struct i2c_client *client,
			   struct ft5x0x_ts_data *tsdata)
{
	int error;

	if (gpio_is_valid(tsdata->wake_gpio)) {
		/* reset is active low */
		error = devm_gpio_request_one(&client->dev, tsdata->wake_gpio,
					      GPIOF_OUT_INIT_LOW,
					      "ft5x0x wake");

		if (error) {
			dev_err(&client->dev,
			"Failed to request GPIO %d as wake pin, error %d\n",
			tsdata->wake_gpio, error);

			return error;
		}

		mdelay(20);
		gpio_set_value(tsdata->wake_gpio, 1);
	}

	return 0;
}

static int ft5x0x_i2c_ts_probe_dt(struct device *dev,
				  struct ft5x0x_ts_data *tsdata)
{
	struct device_node *np = dev->of_node;

	tsdata->irq_gpio = -EINVAL;
	tsdata->reset_gpio = -EINVAL;
	tsdata->wake_gpio = of_get_named_gpio(np, "wake-gpios", 0);

	return 0;
}

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	const struct ft5x0x_platform_data *pdata;
	struct ft5x0x_ts_data *tsdata;
	struct input_dev *input;
	int error;

	pdata = dev_get_platdata(&client->dev);

	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);

	if (!tsdata) {
		dev_err(&client->dev, "Failed to allocate driver data.\n");
		return -ENOMEM;
	}

	if (!pdata) {
		error = ft5x0x_i2c_ts_probe_dt(&client->dev, tsdata);
		if (error) {
			dev_err(&client->dev, "DT probe failed and no platform data present\n");
			return error;
		}
	} else {
		tsdata->irq_gpio = pdata->irq_gpio;
		tsdata->wake_gpio = pdata->wake_gpio;
	}

	error = ft5x0x_ts_reset(client, tsdata);

	if (error)
		return error;

	if (gpio_is_valid(tsdata->irq_gpio)) {
		error = devm_gpio_request_one(&client->dev, tsdata->irq_gpio,
					      GPIOF_IN, "ft5x0x irq");

		if (error) {
			dev_err(&client->dev, "Failed to request GPIO %d, error %d\n",
					tsdata->irq_gpio, error);
			return error;
	}
	} else {
		dev_dbg(&client->dev, "irq_gpio: %d\n", tsdata->irq_gpio);
		dev_err(&client->dev, "irq_gpio is invalid\n");
	}

	input = devm_input_allocate_device(&client->dev);

	if (!tsdata || !input) {
		dev_err(&client->dev, "Failed to allocate driver data.\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&tsdata->work, ft5x0x_ts_work);

	tsdata->client = client;
	tsdata->input = input;

	dev_dbg(&client->dev, "FT5x0x Firmware version: 0x%x\n",
			ft5x0x_read_fw_ver(tsdata));

	input->name = FT5X0X_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);

	input_mt_init_slots(input, 5, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);

	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	error = devm_request_irq(&client->dev, client->irq, ft5x0x_ts_isr,
				 IRQF_TRIGGER_FALLING, dev_name(&client->dev),
				 tsdata);

	if (error != 0) {
		dev_err(&client->dev, "FT5x0x: Unable to request touchscreen IRQ.\n");
		return error;
	}

	error = input_register_device(input);

	if (error) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));

		return error;
	}

	device_init_wakeup(&client->dev, 1);

	dev_dbg(&client->dev, "FT5x0x initialized\n");
	return 0;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(client);

	input_unregister_device(tsdata->input);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ft5x0x_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int ft5x0x_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5x0x_ts_pm_ops, ft5x0x_ts_suspend, ft5x0x_ts_resume);

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x0x_dt_match[] = {
	{ .compatible = "focaltech,ft5x0x" },
	{},
};
#endif

static struct i2c_driver ft5x0x_ts_driver = {
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ft5x0x_dt_match,
	},
	.probe		= ft5x0x_ts_probe,
	.remove		= ft5x0x_ts_remove,
	.id_table	= ft5x0x_ts_id,
};

module_i2c_driver(ft5x0x_ts_driver);

MODULE_AUTHOR("Ash Charles <ash@gumstix.com>");
MODULE_AUTHOR("Adam Lee <adam@gumstix.com>");
MODULE_DESCRIPTION("FocalTech FT5x0x Touchscreen driver");
MODULE_LICENSE("GPL");
