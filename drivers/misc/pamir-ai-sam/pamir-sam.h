/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pamir AI Signal Aggregation Module (SAM) Header
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#ifndef _PAMIR_SAM_H
#define _PAMIR_SAM_H

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/timer.h>

/* Debug levels */
#define SAM_DEBUG_OFF     0  /* No debugging */
#define SAM_DEBUG_ERROR   1  /* Errors only */
#define SAM_DEBUG_INFO    2  /* Informational messages */
#define SAM_DEBUG_VERBOSE 3  /* Verbose debugging */

#define DEVICE_NAME "pamir-sam"
#define MAX_DEVICES 1
#define RX_BUF_SIZE 32
#define TX_BUF_SIZE 256
#define DEBUG_QUEUE_SIZE 32

/* Protocol definitions */
#define PACKET_SIZE 4  /* Type+Flags (1B) + Data (2B) + Checksum (1B) */

/* Message types (3 most significant bits) */
#define TYPE_BUTTON     0x00  /* 0b000xxxxx */
#define TYPE_LED        0x20  /* 0b001xxxxx */
#define TYPE_POWER      0x40  /* 0b010xxxxx */
#define TYPE_DISPLAY    0x60  /* 0b011xxxxx */
#define TYPE_DEBUG_CODE 0x80  /* 0b100xxxxx */
#define TYPE_DEBUG_TEXT 0xA0  /* 0b101xxxxx */
#define TYPE_SYSTEM     0xC0  /* 0b110xxxxx */
#define TYPE_RESERVED   0xE0  /* 0b111xxxxx */
#define TYPE_MASK       0xE0  /* 0b11100000 */

/* Button event flags (5 least significant bits) */
#define BTN_UP_MASK     0x01  /* Button up event */
#define BTN_DOWN_MASK   0x02  /* Button down event */
#define BTN_SELECT_MASK 0x04  /* Button select event */
#define BTN_POWER_MASK  0x08  /* Power button event */

/* LED control flags */
#define LED_CMD_QUEUE    0x00  /* Queue command for later execution */
#define LED_CMD_EXECUTE  0x10  /* Execute all queued commands */
#define LED_ID_MASK      0x0F  /* LED identifier mask (0-15) */

/* LED completion states */
#define LED_COMPLETION   0xFF  /* Value in data[0] indicating sequence completion */

/* Power management commands */
#define POWER_CMD_QUERY           0x00  /* Query current power status */
#define POWER_CMD_SET             0x10  /* Set power state (boot notification) */
#define POWER_CMD_SLEEP           0x20  /* Enter sleep mode */
#define POWER_CMD_SHUTDOWN        0x30  /* System shutdown notification */
#define POWER_CMD_CURRENT         0x40  /* Current draw reporting */
#define POWER_CMD_BATTERY         0x50  /* Battery state reporting */
#define POWER_CMD_TEMP            0x60  /* Temperature reporting */
#define POWER_CMD_VOLTAGE         0x70  /* Voltage reporting */
#define POWER_CMD_REQUEST_METRICS 0x80  /* Request power metrics */
#define POWER_CMD_MASK            0xF0  /* Mask for power command bits */

/* Debug text flags */
#define DEBUG_FIRST_CHUNK  0x10  /* First chunk of debug text */
#define DEBUG_CONTINUE     0x08  /* Continuation of debug text */
#define DEBUG_CHUNK_MASK   0x07  /* Mask for chunk number */

/* System control actions */
#define SYSTEM_PING        0x00  /* Ping the rp2040 controller */
#define SYSTEM_RESET       0x01  /* Reset the rp2040 controller */
#define SYSTEM_VERSION     0x02  /* Get system version */
#define SYSTEM_STATUS      0x03  /* Get system status */
#define SYSTEM_CONFIG      0x04  /* Get system configuration */

/* Version information */
#define PAMIR_SAM_VERSION_MAJOR 1
#define PAMIR_SAM_VERSION_MINOR 0
#define PAMIR_SAM_VERSION_PATCH 0
#define PAMIR_SAM_VERSION_STRING "1.0.0"

/**
 * struct sam_protocol_packet - SAM packet structure
 * @type_flags: Type (3 bits) and flags (5 bits)
 * @data: 16-bit payload (2 bytes)
 * @checksum: XOR checksum of all previous bytes
 *
 * This structure represents the ultra-optimized 4-byte packet format.
 */
struct sam_protocol_packet {
	uint8_t type_flags;
	uint8_t data[2];
	uint8_t checksum;
} __packed;

/**
 * struct debug_code_entry - Debug code entry
 * @category: Debug category (0-31)
 * @code: Debug code (0-255)
 * @param: Parameter value (0-255)
 * @timestamp: Time when the debug code was received
 *
 * Entries for the debug code circular buffer.
 */
struct debug_code_entry {
	uint8_t category;
	uint8_t code;
	uint8_t param;
	unsigned long timestamp;
};

/**
 * struct sam_power_metrics - Power-related metrics
 * @current_ma: Current draw in mA
 * @battery_pct: Battery state of charge in percentage
 * @temp_decidegc: Temperature in 0.1°C units
 * @voltage_mv: Voltage in mV
 * @last_update_jiffies: Timestamp of last update
 *
 * This structure holds power-related metrics reported by the RP2040.
 */
struct sam_power_metrics {
	uint16_t current_ma;
	uint16_t battery_pct;
	uint16_t temp_decidegc;
	uint16_t voltage_mv;
	unsigned long last_update_jiffies;
};

/**
 * struct sam_protocol_config - configuration for SAM protocol
 * @debug_level: Current debug level (0-3)
 * @ack_required: Whether commands require acknowledgment
 * @recovery_timeout_ms: Timeout for protocol recovery
 * @power_poll_interval_ms: Polling interval for power metrics
 *
 * This struct holds configurable parameters for the SAM protocol driver
 * that can be set through device tree properties.
 */
struct sam_protocol_config {
	unsigned int debug_level;
	bool ack_required;
	unsigned int recovery_timeout_ms;
	unsigned int power_poll_interval_ms; /* Polling interval for power metrics */
};

/**
 * struct sam_protocol_data - private data for SAM protocol
 * @input_dev: pointer to input device
 * @rx_buf: Receive buffer for UART data
 * @rx_pos: Current position in receive buffer
 * @config: Protocol configuration
 * @last_receive_jiffies: Last time data was received
 * @serdev: pointer to serdev device
 * @cdev: character device for userspace communication
 * @dev_no: device number for char device
 * @tx_mutex: mutex for TX operations
 * @dev_class: device class for char device
 * @debug_codes: Circular buffer for debug codes
 * @debug_head: Head of debug code circular buffer
 * @debug_tail: Tail of debug code circular buffer
 * @debug_mutex: Mutex for debug code buffer access
 * @packet_stats: Statistics for received/processed packets
 * @rx_state: Current state of packet processing state machine
 * @work_queue: Workqueue for deferred processing
 * @power_metrics: Power-related metrics from RP2040
 * @power_poll_timer: Timer for polling power metrics
 * @metrics_polling_enabled: Flag to control polling
 *
 * This struct holds the runtime state of the SAM protocol driver.
 */
struct sam_protocol_data {
	struct input_dev *input_dev;
	uint8_t rx_buf[RX_BUF_SIZE];
	size_t rx_pos;
	struct sam_protocol_config config;
	unsigned long last_receive_jiffies;

	/* UART device */
	struct serdev_device *serdev;
	struct cdev cdev;
	dev_t dev_no;
	struct mutex tx_mutex;
	struct class *dev_class;

	/* Debug tracking */
	struct debug_code_entry debug_codes[DEBUG_QUEUE_SIZE];
	int debug_head;
	int debug_tail;
	struct mutex debug_mutex;

	/* Protocol state */
	uint64_t packet_stats[8];  /* Stats per message type */
	int rx_state;
	struct workqueue_struct *work_queue;
	
	/* Power metrics */
	struct sam_power_metrics power_metrics;
	struct timer_list power_poll_timer; /* Timer for polling power metrics */
	bool metrics_polling_enabled;       /* Flag to control polling */
};

/* Function declarations */
/* Protocol functions */
uint8_t calculate_checksum(const struct sam_protocol_packet *packet);
bool verify_checksum(const struct sam_protocol_packet *packet);
int send_packet(struct sam_protocol_data *priv, struct sam_protocol_packet *packet);
void process_packet(struct sam_protocol_data *priv, const struct sam_protocol_packet *packet);

/* Command senders */
int send_led_command(struct sam_protocol_data *priv, uint8_t led_id,
		     bool execute, uint8_t r, uint8_t g, uint8_t b, uint8_t time);
int send_system_command(struct sam_protocol_data *priv, uint8_t action,
			uint8_t command, uint8_t subcommand);

/* Power management functions */
int send_boot_notification(struct sam_protocol_data *priv);
int send_shutdown_notification(struct sam_protocol_data *priv, uint8_t shutdown_mode);
int register_power_handlers(struct sam_protocol_data *priv);
void unregister_power_handlers(struct sam_protocol_data *priv);
int request_power_metrics(struct sam_protocol_data *priv);
void start_power_metrics_polling(struct sam_protocol_data *priv);
void stop_power_metrics_polling(struct sam_protocol_data *priv);

/* Message handlers */
void process_button_packet(struct sam_protocol_data *priv,
			   const struct sam_protocol_packet *packet);
void process_led_packet(struct sam_protocol_data *priv,
			const struct sam_protocol_packet *packet);
void process_power_packet(struct sam_protocol_data *priv,
			  const struct sam_protocol_packet *packet);
void process_display_packet(struct sam_protocol_data *priv,
			    const struct sam_protocol_packet *packet);
void process_debug_code_packet(struct sam_protocol_data *priv,
			       const struct sam_protocol_packet *packet);
void process_debug_text_packet(struct sam_protocol_data *priv,
			       const struct sam_protocol_packet *packet);
void process_system_packet(struct sam_protocol_data *priv,
			   const struct sam_protocol_packet *packet);
void process_extended_packet(struct sam_protocol_data *priv,
			     const struct sam_protocol_packet *packet);
int send_extended_version_info(struct sam_protocol_data *priv);

/* Character device interface */
int setup_char_device(struct sam_protocol_data *priv);
void cleanup_char_device(struct sam_protocol_data *priv);

/* Exported globals */
extern struct led_classdev *pamir_led;

#endif /* _PAMIR_SAM_H */ 
