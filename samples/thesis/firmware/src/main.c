/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*ALL*/
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <settings/settings.h>
/*BLUETOOTH*/
/*NOTE: ENABLING BLUETOOTH MAKES THE FIRMWARE HIGHLY UNSTABLE*/
//#include <bluetooth/bluetooth.h>
//#include <bluetooth/hci.h>
//#include <bluetooth/conn.h>
//#include <bluetooth/uuid.h>
//#include <bluetooth/gatt.h>
//#define ENABLE_BLUETOOTH
/*DISPLAY*/
#include <drivers/display.h>
#include <lvgl.h>
#include <drivers/gpio.h>
/*SENSOR & STEP COUNTER*/
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include "StepCountingAlgo.h"
static struct device *i2c;
static struct bma4_dev accel;

/*DISPLAY*/
#define LED_PORT DT_ALIAS_LED1_GPIOS_CONTROLLER
#define LED DT_ALIAS_LED1_GPIOS_PIN

static void backlight_init(void)
{
	struct device *dev;

	dev = device_get_binding(LED_PORT);
	/* If you have a backlight, set it up and turn it on here */
	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);
	gpio_pin_write(dev, LED, 0);
}

static struct sensor_value touch[1];
static struct device *devtouch;
static s32_t lastTouchPoint[2] = {-1, -1};
static bool stop = true;
static lv_obj_t *accel_btn_text;
static lv_obj_t *reset_btn_text;
static void handleTouch(void)
{
	sensor_sample_fetch(devtouch);
	sensor_channel_get(devtouch, SENSOR_CHAN_ACCEL_XYZ, touch);
	if (lastTouchPoint[0] != touch[0].val2 || lastTouchPoint[1] != touch[0].val1)
	{
		lastTouchPoint[0] = touch[0].val2;
		lastTouchPoint[1] = touch[0].val1;
		if (touch[0].val2 < 100 && touch[0].val1 < 100)
		{
			//Handle stop
			stop = (stop) ? false : true;
			if (stop)
			{
				lv_label_set_text(accel_btn_text, "#0000FF START");
			}
			else
			{
				lv_label_set_text(accel_btn_text, "#00FF00 STOP");
			}
		}
		else if (touch[0].val2 > 180 && touch[0].val1 < 100)
		{
			//handle reset
			resetAlgo();
			resetSteps();
		}
	}
}


/*BLUETOOTH*/
#ifdef ENABLE_BLUETOOTH
static struct bt_uuid_128 raw_data_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 step_svc_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x55, 0x33, 0x11);

#define MAX_DATA 20
static u8_t raw_data_long_value[MAX_DATA];

static u8_t step_value[6];

static ssize_t read_long_raw_data(struct bt_conn *conn,
							 const struct bt_gatt_attr *attr, void *buf,
							 u16_t len, u16_t offset)
{
	//const char *value = attr->user_data;
	ssize_t ret = bt_gatt_attr_read(conn, attr, buf, len, offset, raw_data_long_value,
									sizeof(raw_data_long_value));
	return ret;
}

static ssize_t read_step(struct bt_conn *conn,
						 const struct bt_gatt_attr *attr, void *buf,
						 u16_t len, u16_t offset)
{
	//const char *value = attr->user_data;
	ssize_t ret = bt_gatt_attr_read(conn, attr, buf, len, offset, step_value,
									sizeof(step_value));
	return ret;
}

static const struct bt_uuid_128 raw_data_long_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const struct bt_uuid_128 step_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x55, 0x33, 0x11);

static void raw_data_long_ccc_cfg_changed(const struct bt_gatt_attr *attr,
									 u16_t value) {}
static void step_ccc_cfg_changed(const struct bt_gatt_attr *attr,
								 u16_t value) {}

BT_GATT_SERVICE_DEFINE(raw_data_svc,
					   BT_GATT_PRIMARY_SERVICE(&raw_data_uuid),
					   BT_GATT_CHARACTERISTIC(&raw_data_long_uuid.uuid,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ, read_long_raw_data, NULL,
											  &raw_data_long_value),
					   BT_GATT_CCC(raw_data_long_ccc_cfg_changed,
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );
BT_GATT_SERVICE_DEFINE(step_svc,
					   BT_GATT_PRIMARY_SERVICE(&step_svc_uuid),
					   BT_GATT_CHARACTERISTIC(&step_uuid.uuid,
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ, read_step, NULL,
											  &step_value),
					   BT_GATT_CCC(step_ccc_cfg_changed,
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
				  0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
				  0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
				  0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err)
	{
		printk("Connection failed (err 0x%02x)\n", err);
	}
	else
	{
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static inline s16_t swap_int16(s16_t val)
{
	return (val << 8) | ((val >> 8) & 0xFF);
}
#endif
void main(void)
{
/*BLUETOOTH*/
#ifdef ENABLE_BLUETOOTH
	int err;
	for (u8_t i = 0; i < MAX_DATA; i++)
	{
		raw_data_long_value[i] = i;
	}
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	bt_ready();
	bt_conn_cb_register(&conn_callbacks);
	delay_ms(5);
#endif
	/*ACCELEROMETER*/
	i2c = device_get_binding("I2C_1");
	//enable accelerometer
	//set data_rate to 50Hz
	//sensitivity to 16G
	//continous mode
	u16_t count = 0;
	s16_t lastAccelSample[3];
	//create sens_data struct with X, Y, Z as int16_t
	//read data into struct
	lastAccelSample[0] = sens_data.x;
	lastAccelSample[1] = sens_data.y;
	lastAccelSample[2] = sens_data.z;
	s16_t accelVals[3];
	u32_t uptime;
	initAlgo();
	/*TOUCH*/
	devtouch = device_get_binding(DT_INST_0_HYNITRON_CST816S_LABEL);
	/*DISPLAY*/
	struct device *display_dev = device_get_binding(CONFIG_LVGL_DISPLAY_DEV_NAME);
	backlight_init();
	display_blanking_off(display_dev);
	static lv_style_t theme;
	lv_style_copy(&theme, &lv_style_plain_color);
	theme.body.main_color = LV_COLOR_BLACK;
	theme.body.grad_color = LV_COLOR_BLACK;
	theme.text.color = LV_COLOR_WHITE;
	lv_obj_set_style(lv_scr_act(), &theme);
	lv_obj_t *step_label = lv_label_create(lv_scr_act(), NULL);
	lv_obj_t *diff_label = lv_label_create(lv_scr_act(), NULL); //Jitter
	lv_obj_t *accel_label1 = lv_label_create(lv_scr_act(), NULL);
	lv_obj_t *accel_label2 = lv_label_create(lv_scr_act(), NULL);
	lv_obj_t *accel_label3 = lv_label_create(lv_scr_act(), NULL);
	lv_obj_t *uptime_label = lv_label_create(lv_scr_act(), NULL);
	accel_btn_text = lv_label_create(lv_scr_act(), NULL);
	reset_btn_text = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(accel_label1, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
	lv_obj_align(accel_label2, NULL, LV_ALIGN_IN_TOP_MID, 0, 25);
	lv_obj_align(accel_label3, NULL, LV_ALIGN_IN_TOP_MID, 0, 50);
	lv_obj_align(step_label, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_obj_align(diff_label, NULL, LV_ALIGN_CENTER, 0, 25); //Jitter
	lv_obj_align(uptime_label, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
	lv_obj_align(accel_btn_text, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
	lv_obj_align(reset_btn_text, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
	lv_label_set_style(accel_label1, LV_LABEL_STYLE_MAIN, &theme);
	lv_label_set_style(accel_label2, LV_LABEL_STYLE_MAIN, &theme);
	lv_label_set_style(accel_label3, LV_LABEL_STYLE_MAIN, &theme);
	lv_label_set_recolor(accel_btn_text, true);
	lv_label_set_recolor(reset_btn_text, true);
	lv_label_set_text(accel_btn_text, "#00FF00 STOP");
	lv_label_set_text(reset_btn_text, "#00FF00 RESET");
	lv_task_handler();
	u32_t lastUptime = 0;
	while (1)
	{
		k_sleep(K_MSEC(17)); //17 for 50Hz 7 for 100Hz (it takes 3-4ms to do a full loop)
		uptime = k_uptime_get_32();
		if (!stop)
		{
			//read data into sens_data
			//decrease accuarcy for better results from step counter
			sens_data.x = sens_data.x >> 2;
			sens_data.y = sens_data.y >> 2;
			sens_data.z = sens_data.z >> 2;
			accelVals[0] = (s16_t)(lastAccelSample[0] - sens_data.x);
			accelVals[1] = (s16_t)(lastAccelSample[1] - sens_data.y);
			accelVals[2] = (s16_t)(lastAccelSample[2] - sens_data.z);
			lastAccelSample[0] = sens_data.x;
			lastAccelSample[1] = sens_data.y;
			lastAccelSample[2] = sens_data.z;
		}
#ifdef ENABLE_BLUETOOTH
		//Swap bytes for bluetooth sending
		s16_t x = swap_int16(accelVals[0]);
		s16_t y = swap_int16(accelVals[1]);
		s16_t z = swap_int16(accelVals[2]);
		//copy values to bluetooth buffer
		memcpy(&(raw_data_long_value[(count * 10)]), &uptime, sizeof(uptime));
		memcpy(&(raw_data_long_value[(count * 10) + 4]), &x, sizeof(accelVals[0]));
		memcpy(&(raw_data_long_value[(count * 10) + 6]), &y, sizeof(accelVals[1]));
		memcpy(&(raw_data_long_value[(count * 10) + 8]), &z, sizeof(accelVals[2]));
		int32_t steps = getSteps();
		sprintf(step_value, "%d", steps);
#endif
		count++;
		handleTouch();
		if (!stop)
		{
			processSample(uptime, accelVals[0], accelVals[1], accelVals[2]);
		}
		if (count == 2)
		{
			count = 0;
			//send data over bluetooth
#ifdef ENABLE_BLUETOOTH
			if (!stop)
			{
				bt_gatt_notify(NULL, &raw_data_svc.attrs[1], &raw_data_long_value, sizeof(raw_data_long_value));
				bt_gatt_notify(NULL, &step_svc.attrs[1], &step_value, sizeof(step_value));
			}
#endif
		}
		if (count == 1)
		{
			/*DISPLAY*/
			lv_label_set_text_fmt(accel_label1, "X: %d", accelVals[0]);
			lv_label_set_text_fmt(accel_label2, "Y: %d", accelVals[1]);
			lv_label_set_text_fmt(accel_label3, "Z: %d", accelVals[2]);
			lv_label_set_text_fmt(step_label, "Step: %d", getSteps());
			lv_label_set_text_fmt(uptime_label, "%ld", uptime);
			lv_label_set_text_fmt(diff_label, "diff: %d", (uptime - lastUptime)); //Jitter
			lv_task_handler();
		}
		lastUptime = uptime; //Jitter
	}
}