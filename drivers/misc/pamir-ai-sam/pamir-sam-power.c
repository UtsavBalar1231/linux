// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) Power Manager
 *
 * Power management functionality.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"
#include <linux/reboot.h>

/* Sysfs attributes for power metrics */
static ssize_t current_ma_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);

	return sprintf(buf, "%u\n", priv->power_metrics.current_ma);
}
static DEVICE_ATTR_RO(current_ma);

static ssize_t battery_percent_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);

	return sprintf(buf, "%u\n", priv->power_metrics.battery_pct);
}
static DEVICE_ATTR_RO(battery_percent);

static ssize_t temperature_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);
	uint16_t temp = priv->power_metrics.temp_decidegc;

	return sprintf(buf, "%u.%u\n", temp / 10, temp % 10);
}
static DEVICE_ATTR_RO(temperature);

static ssize_t voltage_mv_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);

	return sprintf(buf, "%u\n", priv->power_metrics.voltage_mv);
}
static DEVICE_ATTR_RO(voltage_mv);

static ssize_t metrics_last_update_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);
	unsigned long age_ms;

	age_ms = jiffies_to_msecs(jiffies - priv->power_metrics.last_update_jiffies);

	return sprintf(buf, "%lu ms ago\n", age_ms);
}
static DEVICE_ATTR_RO(metrics_last_update);

/* Group all power metrics attributes */
static struct attribute *sam_power_attrs[] = {
	&dev_attr_current_ma.attr,
	&dev_attr_battery_percent.attr,
	&dev_attr_temperature.attr,
	&dev_attr_voltage_mv.attr,
	&dev_attr_metrics_last_update.attr,
	NULL
};

static const struct attribute_group sam_power_group = {
	.name = "power_metrics",
	.attrs = sam_power_attrs,
};

/**
 * send_boot_notification() - Notify RP2040 that Linux has booted
 * @priv: Private driver data
 *
 * Send a POWER_CMD_SET packet to inform RP2040 that Linux is running.
 * This should be called during driver initialization.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_boot_notification(struct sam_protocol_data *priv)
{
	struct sam_protocol_packet packet;
	int ret;

	dev_info(&priv->serdev->dev, "Sending boot notification to RP2040\n");

	packet.type_flags = TYPE_POWER | POWER_CMD_SET;
	packet.data[0] = 0x01;  /* Power state = running */
	packet.data[1] = 0x00;  /* No flags */

	ret = send_packet(priv, &packet);
	if (ret)
		return ret;
		
	/* Send version information after boot notification */
	dev_info(&priv->serdev->dev, "Sending version %s to RP2040\n", 
		PAMIR_SAM_VERSION_STRING);
		
	/* First send version command with major and minor version */
	ret = send_system_command(priv, SYSTEM_VERSION, 
		PAMIR_SAM_VERSION_MAJOR, PAMIR_SAM_VERSION_MINOR);
	if (ret)
		return ret;
		
	/* Then send extended version info with patch version */
	ret = send_extended_version_info(priv);
		
	return ret;
}

/**
 * send_shutdown_notification() - Notify RP2040 of system shutdown
 * @priv: Private driver data
 * @shutdown_mode: Mode of shutdown (0 = normal, 1 = emergency)
 *
 * Send a POWER_CMD_SHUTDOWN packet to inform RP2040 that Linux is shutting down.
 * This gives the microcontroller time to prepare for power loss.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_shutdown_notification(struct sam_protocol_data *priv, uint8_t shutdown_mode)
{
	struct sam_protocol_packet packet;

	dev_info(&priv->serdev->dev, "Sending shutdown notification to RP2040\n");

	packet.type_flags = TYPE_POWER | POWER_CMD_SHUTDOWN;
	packet.data[0] = shutdown_mode;  /* 0 = normal, 1 = emergency */
	packet.data[1] = 0x00;  /* No flags */

	return send_packet(priv, &packet);
}

/**
 * process_power_packet() - Process power management packet
 * @priv: Private driver data
 * @packet: Received packet
 *
 * Handle power state reports from the RP2040.
 */
void process_power_packet(struct sam_protocol_data *priv,
			  const struct sam_protocol_packet *packet)
{
	uint8_t cmd = packet->type_flags & POWER_CMD_MASK;
	uint8_t param = packet->type_flags & 0x0F;
	uint8_t data_low = packet->data[0];
	uint8_t data_high = packet->data[1];
	uint16_t value = (data_high << 8) | data_low;

	dev_dbg(&priv->serdev->dev,
	 "Power packet - Cmd: 0x%02x, Param: 0x%02x, Data: 0x%02x 0x%02x\n", 
	 cmd, param, data_low, data_high);

	switch (cmd) {
	case POWER_CMD_CURRENT:
		/* Current in mA */
		dev_info(&priv->serdev->dev, "Power current: %u mA\n", value);
		priv->power_metrics.current_ma = value;
		priv->power_metrics.last_update_jiffies = jiffies;
		break;

	case POWER_CMD_BATTERY:
		/* Battery percentage */
		if (value > 100)
			value = 100; /* Clamp to valid percentage */
		dev_info(&priv->serdev->dev, "Battery charge: %u%%\n", value);
		priv->power_metrics.battery_pct = value;
		priv->power_metrics.last_update_jiffies = jiffies;
		break;

	case POWER_CMD_TEMP:
		/* Temperature in 0.1°C */
		dev_info(&priv->serdev->dev, "Temperature: %u.%u°C\n", 
			value / 10, value % 10);
		priv->power_metrics.temp_decidegc = value;
		priv->power_metrics.last_update_jiffies = jiffies;
		break;

	case POWER_CMD_VOLTAGE:
		/* Voltage in mV */
		dev_info(&priv->serdev->dev, "Voltage: %u.%03u V\n", 
			value / 1000, value % 1000);
		priv->power_metrics.voltage_mv = value;
		priv->power_metrics.last_update_jiffies = jiffies;
		break;

	case POWER_CMD_QUERY:
		/* RP2040 is querying power status */
		/* Respond with current power state */
		{
			struct sam_protocol_packet response;

			response.type_flags = TYPE_POWER | POWER_CMD_QUERY;
			response.data[0] = 0x01; /* Power state - 1: Running */
			response.data[1] = 0x00; /* Reserved */
			send_packet(priv, &response);
		}
		break;

	case POWER_CMD_SET:
	case POWER_CMD_SLEEP:
	case POWER_CMD_SHUTDOWN:
	default:
		/* Handle other power commands */
		dev_info(&priv->serdev->dev,
	"Power command: 0x%02x, Param: 0x%02x\n", cmd, param);
		break;
	}
}

/* Reboot notifier block for shutdown notification */
static struct notifier_block sam_reboot_notifier;
static struct sam_protocol_data *g_power_priv;

/**
 * sam_reboot_notifier_call() - Reboot notifier callback
 * @nb: Notifier block
 * @code: Action code (reboot, shutdown, etc.)
 * @unused: Unused parameter
 *
 * Called during system reboot/shutdown to notify the RP2040.
 *
 * Return: NOTIFY_DONE
 */
static int sam_reboot_notifier_call(struct notifier_block *nb,
				    unsigned long code, void *unused)
{
	if (!g_power_priv)
		return NOTIFY_DONE;

	/* Only handle power-off and halt events */
	if (code == SYS_POWER_OFF || code == SYS_HALT)
		send_shutdown_notification(g_power_priv, 0); /* Normal shutdown */

	return NOTIFY_DONE;
}

/**
 * register_power_handlers() - Register power-related handlers
 * @priv: Private driver data
 *
 * Register reboot notifier for shutdown notification.
 *
 * Return: 0 on success, negative error code on failure
 */
int register_power_handlers(struct sam_protocol_data *priv)
{
	int ret;

	/* Store global pointer for reboot notifier */
	g_power_priv = priv;

	/* Configure reboot notifier */
	sam_reboot_notifier.notifier_call = sam_reboot_notifier_call;
	sam_reboot_notifier.priority = 0;

	/* Register reboot notifier */
	ret = register_reboot_notifier(&sam_reboot_notifier);
	if (ret)
		dev_err(&priv->serdev->dev,
		   "Failed to register reboot notifier: %d\n", ret);
	else
		dev_info(&priv->serdev->dev,
		    "Registered shutdown notification handler\n");

	/* Initialize power metrics */
	priv->power_metrics.current_ma = 0;
	priv->power_metrics.battery_pct = 0;
	priv->power_metrics.temp_decidegc = 0;
	priv->power_metrics.voltage_mv = 0;
	priv->power_metrics.last_update_jiffies = jiffies;

	/* Create sysfs interface for power metrics */
	ret = sysfs_create_group(&priv->serdev->dev.kobj, &sam_power_group);
	if (ret)
		dev_warn(&priv->serdev->dev, 
			"Failed to create power metrics sysfs group: %d\n", ret);
	else
		dev_info(&priv->serdev->dev, 
			"Created power metrics sysfs interface\n");
			
	/* Start power metrics polling */
	start_power_metrics_polling(priv);

	return ret;
}

/**
 * unregister_power_handlers() - Unregister power-related handlers
 * @priv: Private driver data
 *
 * Unregister reboot notifier.
 */
void unregister_power_handlers(struct sam_protocol_data *priv)
{
	if (g_power_priv) {
		/* Stop power metrics polling */
		stop_power_metrics_polling(priv);
	
		/* Remove sysfs interface */
		sysfs_remove_group(&priv->serdev->dev.kobj, &sam_power_group);
		
		unregister_reboot_notifier(&sam_reboot_notifier);
		g_power_priv = NULL;
		dev_dbg(&priv->serdev->dev,
		   "Unregistered shutdown notification handler\n");
	}
}

/**
 * request_power_metrics() - Request power metrics from RP2040
 * @priv: Private driver data
 *
 * Send a POWER_CMD_REQUEST_METRICS packet to request power metrics.
 * The RP2040 will respond with current, battery, temperature, and voltage.
 *
 * Return: 0 on success, negative error code on failure
 */
int request_power_metrics(struct sam_protocol_data *priv)
{
	struct sam_protocol_packet packet;

	dev_dbg(&priv->serdev->dev, "Requesting power metrics from RP2040\n");

	packet.type_flags = TYPE_POWER | POWER_CMD_REQUEST_METRICS;
	packet.data[0] = 0x00;  /* Reserved */
	packet.data[1] = 0x00;  /* Reserved */

	return send_packet(priv, &packet);
}

/**
 * power_poll_timer_callback() - Timer callback for power metrics polling
 * @timer: Timer list structure
 *
 * This function is called by the kernel timer subsystem to poll for power metrics.
 */
static void power_poll_timer_callback(struct timer_list *timer)
{
	struct sam_protocol_data *priv = from_timer(priv, timer, power_poll_timer);
	
	if (!priv || !priv->metrics_polling_enabled)
		return;
		
	/* Request updated metrics from the RP2040 */
	request_power_metrics(priv);
	
	/* Reschedule the timer */
	if (priv->metrics_polling_enabled) {
		unsigned long next_poll = msecs_to_jiffies(priv->config.power_poll_interval_ms);
		mod_timer(&priv->power_poll_timer, jiffies + next_poll);
	}
}

/**
 * start_power_metrics_polling() - Start polling for power metrics
 * @priv: Private driver data
 *
 * Initialize and start the timer for polling power metrics.
 */
void start_power_metrics_polling(struct sam_protocol_data *priv)
{
	if (!priv)
		return;
		
	dev_info(&priv->serdev->dev, "Starting power metrics polling (interval: %d ms)\n",
		 priv->config.power_poll_interval_ms);
		 
	/* Initialize the timer */
	timer_setup(&priv->power_poll_timer, power_poll_timer_callback, 0);
	
	/* Start polling */
	priv->metrics_polling_enabled = true;
	mod_timer(&priv->power_poll_timer, 
		  jiffies + msecs_to_jiffies(priv->config.power_poll_interval_ms));
}

/**
 * stop_power_metrics_polling() - Stop polling for power metrics
 * @priv: Private driver data
 *
 * Stop and delete the timer for polling power metrics.
 */
void stop_power_metrics_polling(struct sam_protocol_data *priv)
{
	if (!priv)
		return;
		
	dev_info(&priv->serdev->dev, "Stopping power metrics polling\n");
	
	priv->metrics_polling_enabled = false;
	del_timer_sync(&priv->power_poll_timer);
}
