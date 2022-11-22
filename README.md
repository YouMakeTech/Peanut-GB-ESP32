# Peanut-GB-ESP32
A Game Boy (DMG) emulator for the Espressif ESP32 microcontroller based off the Peanut-GB library

## Hardware required
- ESP32 Wroom DevKit
- ILI9225 Screen
- MAX98357A amplifier
- Passive Piezo buzzer

## ESP32 pins assignment

### Buttons (connect pin to GND when a button is pressed)
- UP (GPIO0)
- DOWN (GPIO2)
- LEFT (GPIO4)
- RIGHT (GPIO5)
- BUTTON A (GPIO18)
- BUTTON B (GPIO19)
- SELECT (GPIO21)
- START (GPIO22)

### ILI9225
- VCC (+3.3V)
- GND (GND)
- LED (+3.3V)
- CLK (GPIO14)
- SDI (GPIO13)
- RS (GPIO26)
- RST (GPIO27)
- CS (GPIO15)

### MAX98357A I2S 3W class D Amplifier Breakout Module
- VIN (+3.3V)
- GND (GND)
- BCLK (GPIO32) Serial clock
- LRC (GPIO25) Left or Right Channel (also called WS Word Select)
- DIN (GPIO33) Serial Data
