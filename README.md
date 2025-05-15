# STM32 DWM3000 Positioning System

This project implements a UWB (Ultra-Wideband) positioning system using an STM32 microcontroller and the DWM3000 module.

## 📦 Repository Structure

example_selection.h # Defines the device role

config_option.c # Contains UWB configuration parameters

ds_twr_responder_sts.c # Responder configuration file

## 🛠️ Configuration

### 1. Adjust the UWB settings in config_option.c:

## UWB Configuration Options

The system uses the following default configuration in `config_option.c`:

```c
dwt_config_t config_options = {
    .chan = 5,               // Channel number (5 or 9)
    .txPreambLength = DWT_PLEN_1024,  // Preamble length
    .rxPAC = DWT_PAC32,      // Preamble acquisition chunk size
    .txCode = 9,             // TX preamble code
    .rxCode = 9,             // RX preamble code
    .nsSFD = 3,              // SFD mode
    .dataRate = DWT_BR_850K,  // Data rate
    .phrMode = DWT_PHRMODE_STD, // PHY header mode
    .phrRate = DWT_PHRRATE_STD, // PHY header rate
    .sfdTO = (1024 + 1 + 8 - 8), // SFD timeout
    .stsMode = DWT_STS_MODE_1, // STS mode
    .stsLength = DWT_STS_LEN_128, // STS length
    .pdoaMode = DWT_PDOA_M0   // PDOA mode
};

```

Configuration Notes:
Channel Number: Choose between 5 or 9 depending on initiator config parameters.

Preamble Length: Higher values (e.g., DWT_PLEN_1024) increase range but may reduce throughput.

The responder parameters must match the initiator parameters.

### 2. Configure UWB system to support different initiators

```c

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 2, 'V', 'F', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'F', 'B', 2, 0xE1, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 2, 'V', 'F', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            // Extra 24 bytes initialized to zero
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
```

Bytes 5–6: 'B', 2 — These likely represent the destination address, identifying the device intended to receive the message.

Change 'dwt_setaddress16(0x242);' in ds_twr_responder_sts.c file according to byte 6. For example for 'A', 1 it will be 'dwt_setaddress16(0x141);' and so on.

Bytes 7–8: 'V', 'F' — These likely represent the source address, identifying the device that sent the message.

Сhange Bytes 7–8 according to the initiator ('V E' or 'V F').

Byte 9: Function code indicating the message type (e.g., poll, response, final)

Assign unique identifiers for each anchors. For example:

Initiator A: 'A', 1

Initiator B: 'B', 2

Initiator C: 'C', 3


### 3. Change default development environment

To use default cube id go to .ioс file and change Toolchain / IDE.

### 4. Build and Flash

Use your preferred STM32 development environment to compile and flash the firmware.
