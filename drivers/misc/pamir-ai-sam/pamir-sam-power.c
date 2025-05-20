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

	dev_info(&priv->serdev->dev, "Sending boot notification to RP2040\n");

	packet.type_flags = TYPE_POWER | POWER_CMD_SET;
	packet.data[0] = 0x01;  /* Power state = running */
	packet.data[1] = 0x00;  /* No flags */

	return send_packet(priv, &packet);
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
	__maybe_unused uint8_t data1 = packet->data[0];
	__maybe_unused uint8_t data2 = packet->data[1];

	dev_dbg(&priv->serdev->dev,
	 "Power packet - Cmd: 0x%02x, Param: 0x%02x\n", cmd, param);

	switch (cmd) {
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
		unregister_reboot_notifier(&sam_reboot_notifier);
		g_power_priv = NULL;
		dev_dbg(&priv->serdev->dev,
		   "Unregistered shutdown notification handler\n");
	}
}
