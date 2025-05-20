// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) Protocol Core
 *
 * Core protocol functionality for the SAM driver.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"

/**
 * calculate_checksum() - Calculate XOR checksum for packet
 * @packet: Packet to calculate checksum for
 *
 * Calculate the XOR checksum of all bytes in the packet.
 *
 * Return: Calculated checksum
 */
uint8_t calculate_checksum(const struct sam_protocol_packet *packet)
{
	return packet->type_flags ^ packet->data[0] ^ packet->data[1];
}

/**
 * verify_checksum() - Verify packet checksum
 * @packet: Packet to verify
 *
 * Check if the packet's checksum matches the calculated checksum.
 *
 * Return: true if checksum is valid, false otherwise
 */
bool verify_checksum(const struct sam_protocol_packet *packet)
{
	uint8_t calculated = calculate_checksum(packet);
	bool valid = (packet->checksum == calculated);
	
	if (!valid && packet->type_flags != 0) {  /* Avoid logging for all-zero packets */
		pr_debug("SAM: Checksum verification failed: got 0x%02x, expected 0x%02x\n",
			packet->checksum, calculated);
	}
	
	return valid;
}

/**
 * send_packet() - Send packet to device
 * @priv: Private driver data
 * @packet: Packet to send
 *
 * Prepare and send a packet, updating the checksum.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_packet(struct sam_protocol_data *priv,
		struct sam_protocol_packet *packet)
{
	int ret;

	/* Calculate checksum */
	packet->checksum = calculate_checksum(packet);

	dev_dbg(&priv->serdev->dev, "Sending packet: type=0x%02x, data=[0x%02x,0x%02x], checksum=0x%02x\n",
	     packet->type_flags, packet->data[0], packet->data[1], packet->checksum);

	mutex_lock(&priv->tx_mutex);
	ret = serdev_device_write(priv->serdev, (const unsigned char *)packet,
			       PACKET_SIZE, MAX_SCHEDULE_TIMEOUT);
	mutex_unlock(&priv->tx_mutex);

	if (ret != PACKET_SIZE) {
		dev_err(&priv->serdev->dev, "Failed to send packet: wrote %d of %d bytes\n",
		     ret, PACKET_SIZE);
		return -EIO;
	}
	
	if (priv->config.debug_level >= 3) {
		dev_dbg(&priv->serdev->dev, "Packet sent successfully\n");
	}

	return 0;
}

/**
 * send_system_command() - Send system control command
 * @priv: Private driver data
 * @action: System action (ping, reset, etc.)
 * @command: Command parameter
 * @subcommand: Subcommand parameter
 *
 * Send a system control command to the device.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_system_command(struct sam_protocol_data *priv, uint8_t action,
			uint8_t command, uint8_t subcommand)
{
	struct sam_protocol_packet packet;

	dev_dbg(&priv->serdev->dev, "Sending system command: action=0x%02x, cmd=0x%02x, subcmd=0x%02x\n",
	     action, command, subcommand);

	packet.type_flags = TYPE_SYSTEM | (action & 0x1F);
	packet.data[0] = command;
	packet.data[1] = subcommand;

	return send_packet(priv, &packet);
}

/**
 * process_packet() - Process a complete packet
 * @priv: Private driver data
 * @packet: Packet to process
 *
 * Process a received packet based on its type.
 */
void process_packet(struct sam_protocol_data *priv,
		    const struct sam_protocol_packet *packet)
{
	uint8_t type = packet->type_flags & TYPE_MASK;

	/* Update statistics */
	priv->packet_stats[type >> 5]++;

	dev_dbg(&priv->serdev->dev, "Processing packet: type=0x%02x, data=[0x%02x,0x%02x], checksum=0x%02x\n",
	     packet->type_flags, packet->data[0], packet->data[1], packet->checksum);

	/* Verify checksum first */
	if (!verify_checksum(packet)) {
		dev_warn(&priv->serdev->dev,
	  "Invalid checksum: got 0x%02x, expected 0x%02x\n",
	  packet->checksum, calculate_checksum(packet));
		return;
	}

	/* Process by type */
	switch (type) {
	case TYPE_BUTTON:
		dev_dbg(&priv->serdev->dev, "Processing button packet\n");
		process_button_packet(priv, packet);
		break;

	case TYPE_LED:
		dev_dbg(&priv->serdev->dev, "Processing LED packet\n");
		process_led_packet(priv, packet);
		break;

	case TYPE_POWER:
		dev_dbg(&priv->serdev->dev, "Processing power packet\n");
		process_power_packet(priv, packet);
		break;

	case TYPE_DISPLAY:
		dev_dbg(&priv->serdev->dev, "Processing display packet\n");
		process_display_packet(priv, packet);
		break;

	case TYPE_DEBUG_CODE:
		dev_dbg(&priv->serdev->dev, "Processing debug code packet\n");
		process_debug_code_packet(priv, packet);
		break;

	case TYPE_DEBUG_TEXT:
		dev_dbg(&priv->serdev->dev, "Processing debug text packet\n");
		process_debug_text_packet(priv, packet);
		break;

	case TYPE_SYSTEM:
		dev_dbg(&priv->serdev->dev, "Processing system packet\n");
		process_system_packet(priv, packet);
		break;

	case TYPE_EXTENDED:
		dev_dbg(&priv->serdev->dev, "Processing extended packet\n");
		process_extended_packet(priv, packet);
		break;

	default:
		/* Should never happen due to mask */
		dev_warn(&priv->serdev->dev, "Unknown packet type: 0x%02x\n", type);
		break;
	}

	/* Send acknowledgment if required */
	if (priv->config.ack_required && type != TYPE_BUTTON) {
		dev_dbg(&priv->serdev->dev, "Sending acknowledgment for packet\n");
		send_system_command(priv, SYSTEM_PING, 0x00, 0x00);
	}
}
