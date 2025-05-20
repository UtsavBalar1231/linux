===================================
Pamir AI Signal Aggregation Module
===================================

Overview
--------

The Pamir AI Signal Aggregation Module (SAM) driver provides an interface between a Linux host system and 
the RP2040 microcontroller in Pamir AI devices. It implements a compact,
efficient binary communication protocol designed for low overhead and high 
reliability.

The driver handles various functions including:

- Button input events (via Linux input subsystem)
- LED control (via Linux LED subsystem)
- Power management (boot/shutdown notifications)
- Display control for E-ink displays
- Debug and diagnostic information
- System management commands

Protocol Design
--------------

The protocol uses a fixed-size 4-byte packet format:

- Byte 0: Type flags (3 most significant bits) + subtype/data (5 least significant bits)
- Byte 1: Data byte 1
- Byte 2: Data byte 2
- Byte 3: Checksum (XOR of bytes 0-2)

This compact design provides several advantages:

- Fixed packet size makes parsing simpler and more reliable
- Low overhead for resource-constrained microcontrollers
- Fast processing on both ends
- Error detection through checksums

Message Types
------------

The protocol supports the following message types:

- Button events: Report button presses and releases
- LED control: Configure RGB LED color and animations
- Power management: Control power states and handle boot/shutdown notifications
- Display commands: Control E-ink display updates
- Debug codes: Send diagnostic codes and parameters
- Debug text: Send multi-packet text messages
- System commands: System control functions and status
- Extended commands: Reserved for future expansion

Driver Architecture
------------------

The driver is implemented as a set of modular components:

1. **Protocol Core**: Handles packet parsing, validation, and dispatching
2. **Input Handler**: Processes button events via the Linux input subsystem
3. **LED Handler**: Controls RGB LEDs via the Linux LED subsystem
4. **Power Manager**: Handles power state changes and boot/shutdown notifications
5. **Display Handler**: Communicates with the E-ink display controller
6. **Debug Handler**: Provides diagnostic information and debug logging
7. **System Handler**: Manages system control and status reporting
8. **Character Device**: Provides user space interface for raw packet I/O

Boot and Shutdown Notifications
------------------------------

The driver automatically sends notifications to the RP2040 microcontroller during Linux boot and shutdown:

- **Boot Notification**: Sent during driver initialization to inform the microcontroller that Linux has booted
- **Shutdown Notification**: Sent during system shutdown via the Linux reboot notifier system

These notifications allow the microcontroller to perform appropriate actions like LED animations
or prepare for power state changes.

Debug Options
------------

The driver supports a configurable debug level that can be set via module parameter or device tree:

- **Level 0 (SAM_DEBUG_OFF)**: No debug messages
- **Level 1 (SAM_DEBUG_ERROR)**: Error messages only
- **Level 2 (SAM_DEBUG_INFO)**: Basic informational messages plus errors
- **Level 3 (SAM_DEBUG_VERBOSE)**: Verbose debugging with detailed protocol information

To enable debugging when loading the module:

.. code-block:: bash

    sudo modprobe pamir-ai-sam debug=3

Or append to the kernel command line:

.. code-block:: bash

    pamir-ai-sam.debug=3

User Interface
-------------

The driver provides multiple interfaces:

- **Character Device**: /dev/pamir-sam for direct communication
- **Input Device**: Standard Linux input device for button events
- **LED Class Device**: Standard Linux LED control

Button Mapping
-------------

The driver maps hardware buttons to the following Linux key codes:

============== ============= ============
Hardware Button Linux Key Code Key Code Value
============== ============= ============
UP             KEY_UP        103
DOWN           KEY_DOWN      108
SELECT         KEY_ENTER     28
POWER          KEY_POWER     116
============== ============= ============

Troubleshooting
---------------

For troubleshooting issues with the SAM driver:

1. **Enable verbose logging**:

   .. code-block:: bash

       sudo modprobe pamir-ai-sam debug=3
       dmesg -w | grep -i pamir

2. **Check button events**:

   .. code-block:: bash

       sudo evtest /dev/input/by-id/input-pamir-ai-signal-aggregation-module

3. **Monitor raw communication**:

   .. code-block:: bash

       sudo cat /dev/pamir-sam | hexdump -C

4. **Verify device node creation**:

   .. code-block:: bash

       ls -la /dev/pamir-sam
       grep -i "pamir" /proc/devices

Configuration
------------

The driver is configured through device tree properties, including:

- Debug level
- Acknowledgment requirements
- Recovery timeout

See the device tree binding documentation for details.

Requirements
-----------

- Linux kernel 5.10 or later
- Serial device bus (serdev) support
- LED class support (for LED control)
- Input subsystem support (for button events) 
