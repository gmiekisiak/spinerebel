# NimBrace Firmware v2.2.6

Arduino sketch for the NimBrace spine monitoring device.

## Hardware

- **MCU**: Raspberry Pi Pico 2W (RP2350)
- **IMU**: 2× BNO085 on I2C (0x4A, 0x4B) at 400 kHz
- **Storage**: SD card via SPI
- **RTC**: PCF8523
- **Battery**: BQ27441-G1 fuel gauge
- **UI**: RGB LEDs, vibration motor, pain button

## Build

Requires Arduino IDE with the following board and libraries:

- Board: `earlephilhower/arduino-pico` (Raspberry Pi Pico 2W)
- Libraries:
  - `SparkFun_BNO080_Arduino_Library`
  - `RTClib`
  - `SD` (built-in)

All three `.ino` files must be in the same `nimbrace/` folder (Arduino IDE
compiles them as a single sketch).

## Files

| File | Contents |
|------|----------|
| `nimbrace.ino` | Main: setup, loop, IMU reading, mode switching, LED control |
| `logging.ino` | SD card logging, IMU stuck detection, voltage monitoring |
| `cradle.ino` | Serial command protocol, file transfer, battery reading |

## Operation

The device has two modes:

- **Logging**: Writes 92-byte binary packets to SD at ~91 Hz. Green breathing LED.
- **Transfer**: When USB/cradle detected, stops logging and accepts serial commands
  for file retrieval.

## Serial Protocol

Connect at 115200 baud. Commands are `CMD:<command>:<arg>`.

Key commands: `LIST`, `READ_FILE:<name>`, `ARCHIVE:<name>`, `STATUS`, `SET_TIME:<unix>`,
`GET_ID`, `PING`. Send `CMD:HELP` for the full list.

## Data Format

See `data/README.md` for the binary packet format.

## License

© 2025–2026 Grzegorz Miekisiak / SpineRebel Technology.
Provided for reference. See repository LICENSE.
