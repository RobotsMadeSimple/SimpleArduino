# SimpleArduino

Arduino firmware sketches for the RobotsMadeSimple robot ecosystem. Each sketch
is standalone firmware for a specific board and role, driven over a line-based
serial protocol (115200 baud, newline-terminated ASCII) by the host computer
running [SimpleRobotController](https://github.com/RobotsMadeSimple/SimpleRobotController).

## Sketches

| Board | Sketch | Role |
|-------|--------|------|
| Arduino Nano | [`Nano/NanoEdgeDevice`](Nano/NanoEdgeDevice) | Edge device — IO reads, configuration, and NeoPixel status light. Commands: `ID?`, `GET`, `CFG`, `SET`, `NEO`. Requires the Adafruit NeoPixel library. |
| Arduino Uno (+ CNC Shield V3/V4) | [`Uno/AuxStepperDriver`](Uno/AuxStepperDriver) | Auxiliary stepper axis driver — runs trapezoidal velocity profiles on-device. Commands: `ID?`, `M`, `C`, `S`, `X`, `E`, `D`. |

## Layout

Sketches are grouped by board. Each sketch lives in its own folder (matching the
Arduino IDE requirement that a `.ino` file sit in a folder of the same name), so
you can open any of them directly in the Arduino IDE.

## Dependencies

- **NanoEdgeDevice** — [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
