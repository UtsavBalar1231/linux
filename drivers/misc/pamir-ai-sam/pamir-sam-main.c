// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) Driver
 *
 * Driver for communication between host processor and RP2040 microcontroller
 * providing interface to sensor inputs and actuator outputs on CM5 device.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"

/* Module parameters */
static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0=off, 1=error, 2=info, 3=verbose)");

/**
 * sam_protocol_receive_buf() - Process received UART data
 * @serdev: Serial device
 * @data: Received data buffer
 * @count: Number of bytes received
 *
 * State machine for packet processing.
 *
 * Return: Number of bytes processed
 */
static size_t sam_protocol_receive_buf(struct serdev_device *serdev,
				     const unsigned char *data, size_t count)
{
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);
	size_t i;

	dev_dbg(&serdev->dev, "Received %zu bytes from UART\n", count);

	for (i = 0; i < count; i++) {
		/* Store byte in buffer */
		if (priv->rx_pos < RX_BUF_SIZE) {
			priv->rx_buf[priv->rx_pos++] = data[i];
			dev_dbg(&serdev->dev, "Stored byte 0x%02x at position %zu\n", 
				data[i], priv->rx_pos - 1);
		}

		/* Process packet when complete */
		if (priv->rx_pos >= PACKET_SIZE) {
			dev_dbg(&serdev->dev, "Processing complete packet\n");
			process_packet(priv, (const struct sam_protocol_packet *)priv->rx_buf);
			priv->rx_pos = 0;
		}
	}

	priv->last_receive_jiffies = jiffies;
	return count;
}

/* UART operations */
static const struct serdev_device_ops sam_protocol_serdev_ops = {
	.receive_buf = sam_protocol_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/**
 * sam_protocol_load_config() - Load driver configuration from device tree
 * @node: Device tree node
 * @config: Configuration structure to populate
 * @serdev: Serial device for logging
 *
 * Read device tree properties to configure driver behavior.
 */
static void sam_protocol_load_config(struct device_node *node,
				      struct sam_protocol_config *config,
				      struct serdev_device *serdev)
{
	/* Set defaults */
	config->debug_level = debug; /* Use module parameter as default */
	config->ack_required = false;
	config->recovery_timeout_ms = 1000;

	/* Override with device tree settings if present */
	of_property_read_u32(node, "debug-level", &config->debug_level);
	of_property_read_u32(node, "recovery-timeout-ms",
			  &config->recovery_timeout_ms);

	config->ack_required = of_property_read_bool(node, "ack-required");
	
	dev_info(&serdev->dev, "SAM driver configuration: debug=%d, ack=%d, timeout=%d ms\n",
		config->debug_level, config->ack_required, config->recovery_timeout_ms);
}

/**
 * sam_protocol_probe() - Probe function for SAM protocol driver
 * @serdev: Serial device
 *
 * Initialize hardware, set up input device, and register with the kernel.
 *
 * Return: 0 on success, negative error code on failure
 */
static int sam_protocol_probe(struct serdev_device *serdev)
{
	struct sam_protocol_data *priv;
	struct input_dev *input_dev;
	int ret;

	dev_info(&serdev->dev, "Probing SAM protocol driver\n");

	priv = devm_kzalloc(&serdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&serdev->dev, "Failed to allocate memory for driver data\n");
		return -ENOMEM;
	}

	/* Load configuration from device tree */
	sam_protocol_load_config(serdev->dev.of_node, &priv->config, serdev);

	input_dev = devm_input_allocate_device(&serdev->dev);
	if (!input_dev) {
		dev_err(&serdev->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	priv->input_dev = input_dev;
	priv->rx_pos = 0;
	priv->last_receive_jiffies = jiffies;
	priv->serdev = serdev;
	priv->debug_head = 0;
	priv->debug_tail = 0;

	/* Configure input device */
	dev_dbg(&serdev->dev, "Configuring input device\n");
	input_dev->name = "Pamir AI Signal Aggregation Module";
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_UP, input_dev->keybit);
	__set_bit(KEY_DOWN, input_dev->keybit);
	__set_bit(KEY_ENTER, input_dev->keybit);
	__set_bit(KEY_POWER, input_dev->keybit);

	dev_dbg(&serdev->dev, "Registering input device\n");
	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&serdev->dev, "Failed to register input device: %d\n",
	  ret);
		return ret;
	}

	/* Set up LED class device */
	dev_dbg(&serdev->dev, "Setting up LED class device\n");
	pamir_led = devm_kzalloc(&serdev->dev, sizeof(struct led_classdev), GFP_KERNEL);
	if (pamir_led) {
		pamir_led->name = "pamir:status";
		pamir_led->max_brightness = 255;
		pamir_led->brightness_set = NULL; /* We don't control LED from host yet */
		pamir_led->brightness = 0;

		ret = devm_led_classdev_register(&serdev->dev, pamir_led);
		if (ret) {
			dev_err(&serdev->dev, "Failed to register LED: %d\n", ret);
			pamir_led = NULL; /* Clear on failure */
		}
	}

	serdev_device_set_drvdata(serdev, priv);
	serdev_device_set_client_ops(serdev, &sam_protocol_serdev_ops);

	dev_dbg(&serdev->dev, "Opening serial device\n");
	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&serdev->dev, "Failed to open serdev: %d\n", ret);
		input_unregister_device(input_dev);
		return ret;
	}

	serdev_device_set_baudrate(serdev, 115200);
	serdev_device_set_flow_control(serdev, false);

	dev_dbg(&serdev->dev, "Setting up character device\n");
	ret = setup_char_device(priv);
	if (ret < 0) {
		dev_err(&serdev->dev, "Failed to set up char device: %d\n",
	  ret);
		serdev_device_close(serdev);
		input_unregister_device(input_dev);
		return ret;
	}

	/* Initialize work queue for deferred processing */
	dev_dbg(&serdev->dev, "Creating workqueue\n");
	priv->work_queue = create_singlethread_workqueue("sam_protocol_wq");
	if (!priv->work_queue) {
		dev_err(&serdev->dev, "Failed to create workqueue\n");
		cleanup_char_device(priv);
		serdev_device_close(serdev);
		input_unregister_device(input_dev);
		return -ENOMEM;
	}

	/* Test communication by sending a ping */
	dev_dbg(&serdev->dev, "Sending initial ping to test communication\n");
	if (send_system_command(priv, SYSTEM_PING, 0, 0) == 0)
		dev_info(&serdev->dev, "Initial communication test successful\n");
	else
		dev_warn(&serdev->dev, "Initial communication test failed\n");

	/* Send boot notification to RP2040 */
	dev_dbg(&serdev->dev, "Sending boot notification to RP2040\n");
	if (send_boot_notification(priv) == 0)
		dev_info(&serdev->dev, "Boot notification sent successfully\n");
	else
		dev_warn(&serdev->dev, "Failed to send boot notification\n");

	/* Register shutdown notification handler */
	dev_dbg(&serdev->dev, "Registering power handlers\n");
	register_power_handlers(priv);

	dev_info(&serdev->dev, "SAM protocol driver initialized and ready\n");
	return 0;
}

/**
 * sam_protocol_remove() - Remove function for SAM protocol driver
 * @serdev: Serial device
 *
 * Clean up resources when the driver is unloaded.
 */
static void sam_protocol_remove(struct serdev_device *serdev)
{
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);

	dev_info(&serdev->dev, "Removing SAM protocol driver\n");

	/* Unregister power notification handlers */
	dev_dbg(&serdev->dev, "Unregistering power handlers\n");
	unregister_power_handlers(priv);

	/* Send shutdown notification */
	dev_dbg(&serdev->dev, "Sending shutdown notification\n");
	send_shutdown_notification(priv, 0);

	if (priv->work_queue) {
		dev_dbg(&serdev->dev, "Destroying workqueue\n");
		destroy_workqueue(priv->work_queue);
	}

	/* Send a system reset or shutdown command */
	dev_dbg(&serdev->dev, "Sending system reset command\n");
	send_system_command(priv, SYSTEM_RESET, 0, 0);

	/* Clean up character device */
	dev_dbg(&serdev->dev, "Cleaning up character device\n");
	cleanup_char_device(priv);

	dev_dbg(&serdev->dev, "Closing serial device\n");
	serdev_device_close(serdev);
	input_unregister_device(priv->input_dev);
	
	dev_info(&serdev->dev, "SAM protocol driver removed\n");
}

#ifdef CONFIG_OF
static const struct of_device_id sam_protocol_of_match[] = {
	{ .compatible = "pamir-ai,sam" },
	{}
};

MODULE_DEVICE_TABLE(of, sam_protocol_of_match);
#endif

static struct serdev_device_driver sam_protocol_driver = {
	.probe = sam_protocol_probe,
	.remove = sam_protocol_remove,
	.driver = {
		.name = "pamir_sam_protocol",
		.of_match_table = of_match_ptr(sam_protocol_of_match),
	},
};

module_serdev_device_driver(sam_protocol_driver);

MODULE_ALIAS("serdev:pamir_sam_protocol");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Pamir AI Incorporated.");
MODULE_DESCRIPTION("PamirAI Signal Aggregation Module (SAM)");
