# ESP32-S3 MCU — Jupiter Pinout

**Board:** ESP32-S3-DevKitC-1 (N16R8), socketed into 2 × 22 machined-pin headers
**Target:** Jupiter Motion Controller Board 2026
**Status:** source of truth for schematic capture. Updated 2026-08-19.

> Pins are listed in **physical header order** — left header top-to-bottom, then right header
> top-to-bottom — to match the Espressif DevKitC-1 pinout diagram.

| PIN | Connects To | Pull-Up / Pull-Down | Value | Comments |
|---|---|---|---|---|
| 3V3 | — | — | — | NC. DevKit LDO output — do not back-feed |
| 3V3 | — | — | — | NC |
| RST | Test point | — | — | Optional reset button to GND |
| GPIO4 | On-board divider tap | — | — | **BATT_SENSE.** ADC1_3. 100 k/20 k from VCC + 100 nF + BAT54S |
| GPIO5 | CN2 pin 6 | — | — | M2_ENC_B |
| GPIO6 | J-PROX-L pin 2, via 10 k series | Pull-UP | 4.7 kΩ | PROX_LEFT. Also BAT54S to 3V3/GND + 100 nF |
| GPIO7 | J-PROX-R pin 2, via 10 k series | Pull-UP | 4.7 kΩ | PROX_RIGHT. Also BAT54S to 3V3/GND + 100 nF |
| GPIO15 | BNO086 RST | Pull-UP | 10 kΩ | Active low |
| GPIO16 | TCA9548A SDA · J-I2C pin 3 | Pull-UP | **2.2 kΩ** | I2C_SDA. 4k7 too weak at 400 kHz |
| GPIO17 | TCA9548A SCL · J-I2C pin 4 | Pull-UP | **2.2 kΩ** | I2C_SCL |
| GPIO18 | TCA9548A RESET | Pull-UP | 10 kΩ | Active low |
| GPIO8 | Q1 gate (2N7002), via 100 Ω | Pull-DOWN | 10 kΩ | IR_EMIT. PD at gate — beacon off at boot |
| GPIO3 | — | — | — | NC. Strapping (JTAG sel) |
| GPIO46 | — | — | — | NC. Strapping (LOG) |
| GPIO9 | J-EXP pin 1 | — | — | **SPARE** — the only one. ADC1_8 |
| GPIO10 | BNO086 CS | Pull-UP | 10 kΩ | FSPICS0 |
| GPIO11 | BNO086 SDA/SDI | — | — | FSPID (MOSI) |
| GPIO12 | BNO086 SCL/SCK | — | — | FSPICLK |
| GPIO13 | BNO086 SDO/DI | — | — | FSPIQ (MISO) |
| GPIO14 | BNO086 INT | — | — | Active low |
| 5V | Board 5 V rail | — | — | From BUCK#2. Powers DevKit LDO |
| GND | GND plane | — | — | |
| GND | GND plane | — | — | |
| GPIO43 | J-CONSOLE pin 2 | — | — | U0TXD |
| GPIO44 | J-CONSOLE pin 3 | — | — | U0RXD |
| GPIO1 | U-AMP1 output | — | — | M1_ISENSE. 1 kΩ + 1 µF RC filter |
| GPIO2 | U-AMP2 output | — | — | M2_ISENSE. 1 kΩ + 1 µF RC filter |
| GPIO42 | DRV8870 #2 IN1 | **Pull-DOWN** | **10 kΩ** | M2_DIR. ⚠ Mandatory — floating = wheel spin |
| GPIO41 | DRV8870 #2 IN2 | **Pull-DOWN** | **10 kΩ** | M2_PWM. ⚠ Mandatory |
| GPIO40 | CN1 pin 6 | — | — | M1_ENC_B |
| GPIO39 | CN1 pin 5 | — | — | M1_ENC_A |
| GPIO38 | Onboard RGB LED | — | — | STATUS_LED. `RGB@IO38` — no board wire. `neopixelWrite()` |
| GPIO37 | — | — | — | NC. Octal PSRAM (R8) |
| GPIO36 | — | — | — | NC. Octal PSRAM (R8) |
| GPIO35 | — | — | — | NC. Octal PSRAM (R8) |
| GPIO0 | — | — | — | NC. BOOT button / strapping |
| GPIO45 | — | — | — | NC. Strapping (VDD_SPI) |
| GPIO48 | DRV8870 #1 IN1 | **Pull-DOWN** | **10 kΩ** | M1_DIR. ⚠ Mandatory |
| GPIO47 | CN2 pin 5 | — | — | M2_ENC_A |
| GPIO21 | DRV8870 #1 IN2 | **Pull-DOWN** | **10 kΩ** | M1_PWM. ⚠ Mandatory |
| GPIO20 | — | — | — | NC. USB_D+ |
| GPIO19 | — | — | — | NC. USB_D− |
| GND | GND plane | — | — | |
| GND | GND plane | — | — | |

**44 pins. 24 assigned. GPIO9 is the only spare.**

---

## ⚠ Changed since the earlier draft

**GPIO4 is no longer spare.** The battery voltage divider moved onto the MCB, because VCC is
already present there as the buck input — so the divider taps it locally instead of arriving on a
wire from the power board. This deletes R1/R2 from the power perf board **and** removes one
chassis wire.

```
   VCC ──[100 k]──┬──► GPIO4 (ADC1_3)
                  ├── 100 nF → GND
                  ├── BAT54S: pin1→GND, pin3→node, pin2→3V3
              [20 k]
                  │
                 GND
```

Keep **100 k / 20 k** — ratio 0.1667 gives 2.80 V at a 16.8 V pack, and it matches the existing
firmware constant `BATTERY_V_DIV = 0.16510` so recalibration starts close. Drop to 50 k/10 k only
if ADC readings prove noisy (source impedance halves; drain rises 140 µA → 280 µA, both trivial).

**INA226 removed from GPIO16/17.** Not being fitted — the existing divider does the job and an
INA226 would mean another off-board module, a 5 mΩ shunt spliced into the 40 A pack line, and four
more wires. `J-I2C` remains available if continuous current monitoring is ever wanted.

---

## Critical resistors, by consequence

| Type | Pins | Value | If omitted |
|---|---|---|---|
| **Pull-DOWN** ⚠ | 21, 41, 42, 48 | 10 kΩ | **Floating driver inputs at boot → wheel spin.** Recorded incident |
| Pull-DOWN | Q1 gate (IR) | 10 kΩ | Beacon could latch on at boot |
| **Pull-UP** | 16, 17 | **2.2 kΩ** | Marginal I²C at 400 kHz with mux + 7 cabled ToFs |
| Pull-UP | 10, 15, 18 | 10 kΩ | Undefined CS/RST state before firmware init |
| Pull-UP | 6, 7 | 4.7 kΩ | 12 V open-collector reaching a 3.3 V pin |

---

## Not in this table

These are downstream of the MCU and easy to forget:

- **14 × 4.7 kΩ** pull-ups on the mux's downstream channels (SD0–SD6, SC0–SC6). The TCA9548A does
  **not** pass the upstream 2.2 kΩ pair through — every ToF segment needs its own.
- DRV8870, BNO086, TCA9548A and connector pinouts — see
  `NEW_MOTION_CONTROL_BOARD_2026.md` §3.3.

## Header connectors

| Connector | Pins | Carries |
|---|---|---|
| **J-EXP** | 3 | GPIO9 · 3V3 · GND |
| **J-CONSOLE** | 3 | GND · GPIO43 (TX) · GPIO44 (RX) |
| **J-I2C** | 4 | 3V3 · GND · SDA · SCL — upstream bus, also the temporary-OLED port |
| **J-TOF7** | 4 | Spare mux channel |

**Expansion is via I²C, not GPIO.** Anything on the upstream bus or behind the mux costs zero
pins. Spend GPIO9 deliberately — it is the last one.
