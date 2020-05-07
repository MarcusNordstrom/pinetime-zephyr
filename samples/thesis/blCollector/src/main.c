/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>

#include <drivers/i2c.h>
#include <drivers/sensor.h>

static struct device *i2c;

/* Custom Service Variables */
static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

#define MAX_DATA 20
static u8_t vnd_long_value[MAX_DATA];

static ssize_t read_long_vnd(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr, void *buf,
			     u16_t len, u16_t offset)
{
	//const char *value = attr->user_data;
	ssize_t ret = bt_gatt_attr_read(conn, attr, buf, len, offset, vnd_long_value,
				 sizeof(vnd_long_value));
	return ret;
}

static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static void vnd_long_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				       u16_t value)
{
}

BT_GATT_SERVICE_DEFINE(vnd_svc,
	BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
	BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_long_vnd, NULL,
			       &vnd_long_value),
	BT_GATT_CCC(vnd_long_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);


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
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
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

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static inline s16_t swap_int16( s16_t val ) 
{
    return (val << 8) | ((val >> 8) & 0xFF);
}

void main(void)
{
	int err;
	for(u8_t i = 0; i < MAX_DATA; i++) {
		vnd_long_value[i] = i;
	}
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_cb_register(&conn_callbacks);
	delay_ms(5);
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
	while (1) {
		k_sleep(K_MSEC(20));
		uptime = k_uptime_get_32();
		//read data into sens_data
		sens_data.x = sens_data.x / 4;
		sens_data.y = sens_data.y / 4;
		sens_data.z = sens_data.z / 4;
		accelVals[0] = (s16_t)(lastAccelSample[0] - sens_data.x);
		accelVals[1] = (s16_t)(lastAccelSample[1] - sens_data.y);
		accelVals[2] = (s16_t)(lastAccelSample[2] - sens_data.z);
		lastAccelSample[0] = sens_data.x;
		lastAccelSample[2] = sens_data.y;
		lastAccelSample[1] = sens_data.z;
		s16_t x = swap_int16(accelVals[0]);
		s16_t y = swap_int16(accelVals[1]);
		s16_t z = swap_int16(accelVals[2]);
		memcpy(&(vnd_long_value[(count*10)]), &uptime, sizeof(uptime));
		memcpy(&(vnd_long_value[(count*10) + 4]), &x, sizeof(accelVals[0]));
		memcpy(&(vnd_long_value[(count*10) + 6]), &y, sizeof(accelVals[1]));
		memcpy(&(vnd_long_value[(count*10) + 8]), &z, sizeof(accelVals[2]));
		count++;
		if(count == 2) {
			count = 0;
			bt_gatt_notify(NULL, &vnd_svc.attrs[1], &vnd_long_value, sizeof(vnd_long_value));
		}
	}
}
