// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) LED Handler
 *
 * LED control functionality.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"

/* LED class device for control */
struct led_classdev *pamir_led;

/**
 * process_led_packet() - Process LED control packet
 * @priv: Private driver data
 * @packet: Received packet
 *
 * Handle LED control messages from the RP2040.
 */
void process_led_packet(struct sam_protocol_data *priv,
			const struct sam_protocol_packet *packet)
{
	uint8_t cmd_type = packet->type_flags & LED_CMD_EXECUTE;
	uint8_t led_id = packet->type_flags & LED_ID_MASK;
	uint8_t r = (packet->data[0] >> 4) & 0x0F;
	uint8_t g = packet->data[0] & 0x0F;
	uint8_t b = (packet->data[1] >> 4) & 0x0F;
	uint8_t time = packet->data[1] & 0x0F;

	/* Check if this is a completion acknowledgment */
	if (cmd_type == LED_CMD_EXECUTE && packet->data[0] == LED_COMPLETION) {
		dev_dbg(&priv->serdev->dev, 
			"LED sequence completed for LED ID %u, sequence length: %u\n", 
			led_id, packet->data[1]);
		
		/* Could emit a kernel event or signal userspace here */
		return;
	}

	dev_dbg(&priv->serdev->dev, 
		"LED packet - CMD: %s, LED ID: %u, R: %u, G: %u, B: %u, Time: %u\n",
		(cmd_type == LED_CMD_EXECUTE) ? "Execute" : "Queue", 
		led_id, r, g, b, time);

	/* If we have a registered LED class device, set its brightness */
	if (pamir_led) {
		/* For simplicity, convert RGB to brightness using average */
		int brightness = (r + g + b) / 3;
		brightness = (brightness * 255) / 15;  /* Scale to 0-255 range */

		led_set_brightness(pamir_led, brightness);
	}

	/* Forward LED status to user space via a sysfs attribute if needed */
}

/**
 * send_led_command() - Send LED control command
 * @priv: Private driver data
 * @led_id: LED ID (0-15)
 * @execute: true to execute sequence, false to queue
 * @r: Red component (0-15)
 * @g: Green component (0-15)
 * @b: Blue component (0-15)
 * @time: Time value (0-15) - delay between color changes
 *
 * Send a command to control the LED.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_led_command(struct sam_protocol_data *priv, uint8_t led_id,
		     bool execute, uint8_t r, uint8_t g, uint8_t b, uint8_t time)
{
	struct sam_protocol_packet packet;
	uint8_t cmd_flags;

	/* Validate parameters */
	if (led_id > 15) {
		dev_err(&priv->serdev->dev, "Invalid LED ID: %u (max 15)\n", led_id);
		return -EINVAL;
	}

	if (r > 15 || g > 15 || b > 15 || time > 15) {
		dev_err(&priv->serdev->dev, 
			"Invalid LED parameters: values must be 0-15\n");
		return -EINVAL;
	}

	/* Set command type and LED ID */
	cmd_flags = execute ? LED_CMD_EXECUTE : LED_CMD_QUEUE;
	packet.type_flags = TYPE_LED | cmd_flags | (led_id & LED_ID_MASK);
	
	/* Set color and time data */
	packet.data[0] = ((r & 0x0F) << 4) | (g & 0x0F);
	packet.data[1] = ((b & 0x0F) << 4) | (time & 0x0F);

	dev_dbg(&priv->serdev->dev, "Sending LED command: id=%u, %s, r=%u, g=%u, b=%u, time=%u\n",
		led_id, execute ? "execute" : "queue", r, g, b, time);

	return send_packet(priv, &packet);
}
