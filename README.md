# ESP32 X20P LoRa RTCM Transmitter

ESP32 transmitter firmware for sending ZED-X20P RTCM3 correction messages over a REYAX RYLR896 LoRa module to a rover receiver.

## Description

This project reads RTCM3 correction frames from a ZED-X20P GNSS receiver through an ESP32 UART, validates each RTCM frame using CRC-24Q, Base64-encodes the binary data, splits large frames into LoRa-safe fragments, and transmits them through a RYLR896 LoRa module using AT commands.

The current transmitter is configured for one-way RTCM streaming. It sends each valid RTCM frame once and does not wait for an ACK from the rover.

## Hardware

- ESP32-WROOM / ESP32 development board
- ZED-X20P GNSS receiver configured as RTK base
- REYAX RYLR896 LoRa module
- 3.3 V power supply
- Common ground between all modules

## Wiring

### ZED-X20P to ESP32

| ZED-X20P | ESP32 |
| --- | --- |
| UART TX | GPIO26 |
| GND | GND |

The ESP32 only receives RTCM from the X20P, so the X20P RX line is not used by this firmware.

### RYLR896 to ESP32

| RYLR896 | ESP32 |
| --- | --- |
| TXD | GPIO16 |
| RXD | GPIO17 |
| VCC | 3.3 V |
| GND | GND |

Use 3.3 V TTL UART levels. Do not connect RS-232 level signals.

## Default Settings

| Setting | Value |
| --- | --- |
| USB debug baud | 115200 |
| X20P UART baud | 115200 |
| RYLR896 UART baud | 115200 |
| Base address | 2 |
| Rover address | 1 |
| Network ID | 5 |
| LoRa frequency | 866000000 Hz |
| LoRa RF parameter | `AT+PARAMETER=7,9,1,4` |
| LoRa RF power | `AT+CRFOP=15` |
| Rover ACK | Disabled |

For longer range or better robustness, use this RF parameter on both transmitter and rover:

```text
AT+PARAMETER=7,7,1,4
```

## Arduino IDE Setup

1. Open `ESP32_X20P_LoRa_Base.ino` in Arduino IDE.
2. Select the correct ESP32 board.
3. Set the upload port.
4. If your board does not have PSRAM, set PSRAM to `Disabled`.
5. Upload the sketch.
6. Open Serial Monitor at `115200` baud.

## Expected Serial Monitor Output

On successful startup:

```text
RYLR896 detected at 115200 baud
ESP32 X20P -> RYLR896 RTCM base bridge READY
X20P UART=115200, LoRa UART=115200, base=2, rover=1, network=5
Frame ACK=OFF, max attempts=3
```

When RTCM is being transmitted:

```text
RTCM 1074 sent seq=0000 len=... queue=...
```

The status line shows input and transmission counters:

```text
STATUS bytes=... valid=... invalid=... queue=... sent=... fragments=...
```

## Troubleshooting

### PSRAM warning

If Serial Monitor shows:

```text
psram: PSRAM ID read error: 0xffffffff
```

Disable PSRAM in Arduino IDE. This warning is not related to RTCM or LoRa transmission.

### RYLR896 configuration fails

Check:

- RYLR896 VCC is stable 3.3 V
- ESP32 GPIO17 goes to RYLR896 RXD
- ESP32 GPIO16 goes to RYLR896 TXD
- ESP32 and RYLR896 share GND
- The module responds to `AT` at one of the supported baud rates

### No RTCM frames

Check:

- X20P UART baud is 115200
- X20P RTCM output is enabled on the UART connected to ESP32 GPIO26
- X20P TX is connected to ESP32 GPIO26
- X20P and ESP32 share GND

### Rover does not get corrections

Check:

- Rover LoRa settings match the transmitter settings
- Rover address is `1`
- Base address is `2`
- Network ID is `5`
- Frequency and RF parameters match on both LoRa modules
- Antennas are connected to both LoRa modules
- The rover firmware decodes Base64 fragments and forwards binary RTCM to the F9P UART

## Notes

- RTCM is binary data. If you view binary RTCM directly in PuTTY or a serial terminal, it will look like garbage characters.
- The F9P rover must receive binary RTCM, not readable HEX text.
- The RTCM data rate must fit within the LoRa link capacity. Start with a small RTCM message set such as 1005/1006 plus GPS/Galileo MSM4 at 1 Hz.
