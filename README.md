# SimpleArduino

Arduino firmware sketches for the RobotsMadeSimple robot ecosystem. Each sketch
is standalone firmware for a specific board and role, driven over a line-based
serial protocol (115200 baud, newline-terminated ASCII) by the host computer
running [SimpleRobotController](https://github.com/RobotsMadeSimple/SimpleRobotController).

## Sketches

| Board | Sketch | Version | Caps | Role |
|-------|--------|---------|------|------|
| Arduino Nano | [`Nano/NanoEdgeDevice`](Nano/NanoEdgeDevice) | 1.1.0 | `io,neopixel,loadcell` | Edge device — IO reads, configuration, NeoPixel status light and HX711 load cell. Commands: `ID?`, `GET`, `CFG`, `SET`, `NEO`. Requires the Adafruit NeoPixel library. |
| Arduino Uno (+ CNC Shield V3/V4) | [`Uno/AuxStepperDriver`](Uno/AuxStepperDriver) | 1.1.0 | `stepper,io` | Auxiliary stepper axis driver — runs trapezoidal velocity profiles on-device, plus digital IO on the spare pins. Commands: `ID?`, `M`, `C`, `S`, `X`, `E`, `D`, `CFG`, `GET`, `SET`. |

## Firmware versioning

Every sketch answers `ID?` with a single line that identifies the board, its
firmware version and what it can do:

```
ID:<DEVICE_ID>;fw=<major.minor.patch>;caps=<cap1,cap2,...>
```

For example `ID:ROBOT_NANO_001;fw=1.1.0;caps=io,neopixel,loadcell` or
`ID:AUX_STEPPER_001;fw=1.1.0;caps=stepper,io`. The `ID:<DEVICE_ID>` prefix is
byte-for-byte what the un-versioned firmware sent, so older controllers still
match it; a controller that sees no `;fw=` treats the board as legacy
(version 1.0.0, unknown capabilities).

Capability names are lowercase, comma-separated, no spaces:

| Cap | Meaning |
|-----|---------|
| `io` | Digital input/output pins via `CFG`/`GET`/`SET` and `CHG`/`STATE` |
| `neopixel` | `CFG` type `N` + `NEO` |
| `loadcell` | `CFG` type `L` + `LC` streaming |
| `stepper` | `M`/`C`/`S`/`X`/`E`/`D` axis commands |

Each sketch holds its version and caps in `FW_VERSION` / `FW_CAPS` next to
`DEVICE_ID`. Bump the version with every change that ships:

- **patch** — fixes that change no commands or replies
- **minor** — a capability or command is added (update `FW_CAPS` too)
- **major** — the protocol changes incompatibly

`1.0.0` is retroactively the un-versioned firmware; both sketches start at `1.1.0`.

## Layout

Sketches are grouped by board. Each sketch lives in its own folder (matching the
Arduino IDE requirement that a `.ino` file sit in a folder of the same name), so
you can open any of them directly in the Arduino IDE.

## Dependencies

- **NanoEdgeDevice** — [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
