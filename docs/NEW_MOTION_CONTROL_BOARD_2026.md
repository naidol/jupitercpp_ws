# New Motion Control Board — 2026

**Purpose:** design brief for the successor to `Jupiter ESP32 Drv8870 12vDC ver3_1`
(EasyEDA, JLCPCB-002, drawn 2024-06-29). Captures every change made to the robot since that
board was fabricated, so the schematic can be redrawn and sent to JLCPCB in one pass.

**Status:** specification only. Nothing here has been drawn yet.
**MCU is NOT fixed.** ESP32-WROOM-32 is what ver3_1 and the current firmware use, but the
board is specified so the MCU can change — see §3. Everything in §5–§12 is
MCU-independent unless explicitly marked.
**Source of truth for pin assignments:** `firmware/esp32/include/jupiter_config.h`.
**Predecessor schematic:** `~/Documents/Jupiter_ESP32_Board_easyeda.pdf`.

> ### ⚠ Before drawing anything
> Contradictions between the old schematic, the firmware and the docs. Cheap to settle with a
> multimeter, expensive to get wrong on a fabricated board. Detailed in §10.
>
> 0. **MCU and IMU are NOT fixed** — ESP32-WROOM-32 / ESP32-S3 / RP2350, and BNO055 vs BNO085.
>    See §2. Everything else in this document is MCU-independent and can proceed regardless.
> 1. ~~What feeds DRV8870 `VM`?~~ **RESOLVED** — a regulated **12 V** from an external
>    16.8 V → 12 V buck. That buck moves **onto** the new board (§11). See §10.1 for the firmware
>    bug this exposes.
> 2. ~~Battery divider resistor values?~~ **RETIRED** — the divider is deleted; the INA226
>    supplies bus voltage over I²C (§5). Open in its place: **where does the INA226 shunt go**
>    — motion board (its own draw) or the pack main line (total, and the only thing that
>    answers the flat-on-dock question)?
> 3. Three GPIOs are defined twice in firmware (13, 15, 4). Confirm which function is physically wired.

---

## 1. What changed since ver3_1

| # | Change | Consequence for the new board |
|---|---|---|
| 1 | **4WD → 2WD.** Two driven wheels + one rear caster (180 mm behind the drive axle) | Two DRV8870 channels are now dead weight |
| 2 | **Battery monitoring added.** Resistive divider → GPIO34 | Currently off-board / flying. Needs to be a designed circuit |
| 3 | **Dock proximity sensors added.** 2 × LJ18A3-8-Z/BX inductive, **12 V**, NPN open-collector | 12 V logic arriving at a 3.3 V pin with no protection |
| 4 | **Dock IR emitter added.** TSAL6400 + 220 Ω on GPIO4, 38 kHz | Driven straight from a GPIO. Under-driven, see §7 |
| 5 | **Wheels changed** 65 mm rubber → 100 mm AGV | Firmware constants only, no PCB impact |
| 6 | **Strapping-pin pull-downs needed** | Not on ver3_1 at all. See §8 — this is the most likely cause of intermittent boot failures |
| 7 | **I²C expansion:** TCA9548A mux + up to 7 × VL53L0X ToF | New. Currently hand-wired and **not working** — see §9 |
| 8 | Position-control mode (`/wheel_move`) added to firmware | Software only, no PCB impact |
| 9 | **BNO055 → BNO086**, magnetometer for off-map use | §2.5. Place away from motors; SPI + INT/RST |
| 10 | **SSD1306 OLED deleted** | §9. Purpose gone, invisible in service, `setup()` hang risk |
| 11 | **Pack + motor current sensing added** | §5, §4. Both target measured defects, not spec-chasing |
| 12 | **MCU not fixed** — WROOM-32 / S3 / RP2350 | §2. Board is specified MCU-neutral; socket the module |

---

## 2. MCU and IMU — the open platform decision

Raised 2026-08-19. **Neither is settled.** This section exists so the rest of the document can be
read as MCU-neutral: the stated objective — *reduce the wiring burden on the chassis* — is
delivered by the PCB layout, not by the choice of microcontroller.

### 2.1 What is MCU-independent — do it regardless

Everything below stands whichever MCU is fitted. Do not hold these hostage to a platform decision:

- Battery divider as a designed block, not flying wires (§5)
- Protection on the 12 V open-collector prox inputs (§6)
- A proper low-side driver for the IR emitter (§7)
- **TCA9548A mux on the board** with per-channel pull-ups and keyed ToF connectors (§9) — this is
  the single biggest wiring win, and it retires the hand-wired mux that never enumerated
- Three-rail power with the external buck brought onboard (§11)
- Reverse-polarity protection, regen clamp, thermal pours (§11)

### 2.2 MCU candidates

| | ESP32-WROOM-32 (ver3_1) | **ESP32-S3** | RP2350 / Pico 2 |
|---|---|---|---|
| micro-ROS | proven, in service | supported, same ecosystem | ⚠ **official Pico port targets RP2040 — RP2350 parity MUST be verified** |
| Firmware reuse | 100% | ~95% (same LEDC/ADC/FreeRTOS APIs) | **rewrite** |
| USB | external CP2102 | **native** | native |
| WiFi | yes | yes | only on Pico 2 **W** |
| Encoders | GPIO interrupts | GPIO interrupts | **PIO — hardware quadrature, zero CPU** |
| Era | 2016 | current | current |

### 2.3 The CP2102 argument for native USB

This is not cosmetic, and it is tied to a recorded safety incident. From the parking lot:

> unpowered CP2102 holding EN during Thor's ~30 s cold boot → firmware never runs → both motor
> inputs float → **continuous spin** until Thor is up

Native USB **deletes that part and that failure mode**. Any MCU with native USB (S3 or RP2350)
removes it; staying on WROOM-32 means keeping the CP2102 and relying on the §8 pull-downs alone
to hold the drivers in COAST through the float window.

### 2.4 RP2350 — the honest trade

**The real attraction is PIO.** Hardware quadrature decoding with no missed edges and no CPU cost
is a structural improvement over GPIO interrupts, and this project has a history of encoder
defects: the `getRPM` 0.0-return that produced a 2.6× odometry error, and the CPR recalibration.

**The cost is a firmware rewrite, not a port.** LEDC drives motor PWM *and* the 38 kHz IR carrier;
the dual-core pinning is load-bearing (IR burst timing on core 0, micro-ROS on core 1); the ADC
path uses `esp_adc_cal`. The 4-state auto-reconnect machine, position-control (`/wheel_move`) mode
and the prox reflex would all need revalidating from scratch.

**GATE CHECKED 2026-08-19 — NOT CLEARED.** The official
`micro-ROS/micro_ros_raspberrypi_pico_sdk` is titled *"Raspberry Pi Pico (RP2040) and micro-ROS
integration"* and contains **no mention of RP2350, Pico 2 or `PICO_PLATFORM`**.

The crux is the precompiled `libmicroros.a`: built for **Cortex-M0+ (ARMv6-M, soft-float)**, where
RP2350 is **Cortex-M33 (ARMv8-M, hard-float FPU)**. Using it means rebuilding micro-ROS against a
`cortex-m33` toolchain with a matching float ABI. micro-ROS supports custom builds so this is
likely achievable — but it is a project in itself and unproven on this target. Treat any claim
that RP2350 + micro-ROS is turnkey with suspicion until someone has actually built it.

⚠ **Beware RP2350-vs-RP2040 arguments.** Most published enthusiasm for RP2350 + micro-ROS compares
it to the RP2040, not to an ESP32. Checked against this firmware, those advantages evaporate:

| Common claim | Against Jupiter |
|---|---|
| 520 KB SRAM removes memory pressure | Firmware uses **22.3% — 73 KB of 327 KB**. No pressure exists |
| Hardware FPU vs software emulation | ESP32's Xtensa LX6 **already has a single-precision FPU** |
| Dedicate core 0 to micro-ROS, core 1 to control | Firmware **already pins cores** (`xTaskCreatePinnedToCore`, IR task on core 0) |
| `rcl` deadlocks across cores were an RP2040 flaw | `rcl`/`rclc` is not thread-safe on any MCU — a micro-ROS constraint, not silicon |
| Eliminates custom serial parsers | There are none. micro-ROS has been the transport since the start |
| Native USB-CDC, no UART bridge | Real win over the CP2102 — but **ESP32-S3 has native USB too** |

**And even PIO may not be unique.** ⚠ *Correction, 2026-08-19:* an earlier draft of this section
claimed PIO encoder decoding was the sole surviving RP2350 advantage. The **ESP32-S3 has the PCNT
peripheral** — a hardware pulse counter with quadrature support, 4 units, no CPU per edge. Verify
against the ESP-IDF PCNT documentation, but if it does what it appears to, the last substantive
argument for RP2350 disappears and the S3 case becomes clear-cut.

**Net:** RP2350 buys little the S3 does not already provide, at the cost of a firmware rewrite and
an unproven micro-ROS port.

### 2.5 IMU — DECIDED: BNO086, magnetometer fitted but not trusted by default

**Decision (2026-08-19): fit the BNO086.** The reasoning is not the usual spec-sheet comparison,
which does not apply here — it is off-map operation. Both parts of that matter, so both are recorded.

#### Why the stock BNO055-vs-BNO08x comparison does NOT apply indoors

The firmware already runs magnetometer-free:

```cpp
// imu_bno055.cpp — magnetometer is already OUT of the fusion loop
bno.setMode(OPERATION_MODE_IMUPLUS);
```

| Common claim for BNO08x | Against this robot |
|---|---|
| Drift 1–2°/min → <0.5°/min | **Not comparable.** Those are 9-DOF figures. IMUPLUS yaw is pure gyro integration; the BNO08x's magnetometer-free mode (Game Rotation Vector) drifts for the same physical reason |
| Calibration "drops state randomly" | Mag calibration is irrelevant in IMUPLUS, and stored gyro/accel offsets are restored at boot |
| I²C clock stretching hangs the bus | A **Broadcom BCM2835 (Raspberry Pi)** defect. ESP32 and RP2350 I²C both handle stretching correctly |
| 100 Hz → 400 Hz report rate | IMU publishes at ~18 Hz. Nowhere near the constraint |
| Built-in Tare | Nicer than offset math, but offset restore already works |

**And magnetic heading is unusable indoors regardless of chip.** Hard/soft-iron calibration
corrects distortion **fixed relative to the sensor** (the robot's own steel). Rebar is fixed
relative to the **building** — as the robot drives the distortion changes, and no calibration can
track a field that varies with position. Reinforced concrete, structural steel, AC wiring and lift
motors make indoor magnetic north unreliable *in principle*.

Indoors the absolute heading reference is the **map** (AMCL + S2E), plus the **dock** — `contact=3`
is a known pose to millimetres, so every successful docking is a heading re-zero more trustworthy
than any magnetometer.

#### Why fit a 9-axis part anyway — off-map operation

The robot may leave the map: building lobby, paved garden. There AMCL has nothing to match and
dead reckoning drifts without bound.

**The magnetometer is strongest exactly where the lidar is weakest.** An open paved area has few
walls and little structure, so scan matching degrades — while the magnetic environment is at its
cleanest. Indoors the reverse holds. They fail in opposite environments, which makes them
complementary rather than redundant.

Second use: **re-entering the map.** Global re-localisation is slow and error-prone; an absolute
heading prior collapses the search space.

⚠ Temper expectations for the **lobby** — still rebar in the slab, structural steel, and **lift
motors**, among the largest magnetic disturbers there are. The **paved garden** is the realistic case.

This requirement also **rules out 6-axis alternatives** (e.g. ICM-42688-P, which has better raw gyro
performance but no magnetometer at all).

#### Architecture: mode-switchable, gated on the sensor's own trust signal

Not "magnetometer on or off":

- **Indoors** — Game Rotation Vector (6-axis, mag-free) → `/imu/data`, as today
- **Outdoors** — Rotation Vector (9-axis, mag-fused) when absolute heading is wanted
- **Gate on the sensor, not a manual switch.** The BNO08x reports a **per-report accuracy status
  (0–3)** and a heading accuracy estimate. Publish it, and the EKF or a supervisor can weight or
  reject magnetic yaw dynamically. The indoor→outdoor transition is gradual, not binary — walking
  out of a doorway the field improves progressively and the accuracy estimate tracks it.

Ties into [[project_operational_modes]]: "outdoor mode" becomes a profile that enables the 9-axis
report and permits the EKF to trust absolute yaw.

#### IMU placement — on-board, with an escape hatch

**What calibration can and cannot absorb decides this.**

- **Static** distortion fixed to the robot — the drive motors' permanent magnets, chassis steel,
  the battery's steel cans — is **calibratable**. That is precisely what hard/soft-iron calibration
  does, and what the BNO086 handles in the background.
- **Dynamic** distortion — field that varies with **motor current** — is **not** calibratable,
  because it changes with load from moment to moment.

So the thing to get away from is **current-carrying conductors**, not magnets.

**Rough magnitude.** For a straight conductor, `B ≈ 2×10⁻⁷ · I / r` tesla:

| Current | 30 mm | 100 mm |
|---|---|---|
| 3.3 A (both motors at the ISEN limit) | ~22 µT | ~6.6 µT |

Against a horizontal geomagnetic component that in **southern Africa is comparatively weak** (steep
dip angle means less of the field lies in the horizontal plane used for heading), those are not
small numbers. Check local values rather than assuming a mid-northern-latitude figure.

**But return-path cancellation dominates distance.** Route each motor's `+` and `−` as a **tight
pair** and the far field collapses far faster than 1/r. Good layout beats separation, and is free.

**The decisive point: the largest disturbers are probably OFF the board anyway** — the drive
motors themselves and the battery pack, both on the chassis. Moving the IMU 60 mm across the PCB
does not escape those. On-board placement is therefore unlikely to be the deciding factor.

**Recommendation: fit it on the board**, which also serves the project's stated aim of *reducing*
chassis wiring. Place it:

- **Diagonally opposite** the buck inductor, the DRV8870s and the pack input — the inductor is a
  ferrite core carrying DC bias and is a worse offender than the motor traces
- With **no high-current ground return** flowing beneath it
- Clear of ferrous parts — inductors, steel standoffs, some connector shells
- Ideally near the **drive-axle centreline**, which keeps rotational (centripetal) artefacts out of
  the accelerometer. Minor at this robot's speeds — ω²r is ~0.05 m/s² at 0.5 rad/s and 0.2 m,
  against 9.81 — but free if the board sits there anyway

**Escape hatch — take it.** Also footprint a **6-pin connector** carrying the same SPI + INT + RST +
3V3 + GND, so the IMU can be relocated to a daughterboard on a ribbon if measurement says it must.
Populate one or the other. Cost: one connector and a few cm². That way the placement decision does
not have to be right first time.

**And you can measure it rather than guess.** The BNO086 reports **magnetic field magnitude and a
per-report accuracy status**. Drive the motors through their current range and watch both. That
settles on-board-versus-ribbon definitively, on the actual robot, in an afternoon.

#### Practicalities

- **Interface: SPI preferred** over I²C — keeps the IMU off the bus about to carry the mux and
  several ToFs. Costs a few pins, which are available.
- **INT and RST must be brought out.** The SH-2 protocol is interrupt-driven and misbehaves without them.
- **Blast radius:** ~176 lines (`imu_bno055.h` + `.cpp`), a different library (SH-2 / Adafruit
  BNO08x), and a recalibration. Contained.
- Do **not** expect less heading drift indoors — Game Rotation Vector drifts like IMUPLUS does.
  The gain is off-map capability and better fusion, not indoor accuracy.

### 2.6 Recommendation

**Decouple.** Respin the board for the wiring wins now; treat the MCU migration as a separate,
gated evaluation.

- **ESP32-S3** is the low-risk modernisation: retires the 2016-era part *and* the CP2102, keeps
  the firmware, toolchain, WiFi and micro-ROS path essentially intact.
- **RP2350** only after the micro-ROS question is answered, and understanding it is a rewrite.
- **BNO086 — DECIDED (§2.5).** Fitted for off-map operation (garden/lobby), magnetometer
  available but not trusted by default. Place it away from the motors and power stage.
- **SSD1306 OLED — DELETED (§9).** Its only job was BNO055 figure-8 calibration monitoring; it
  is invisible once the robot is assembled, and it hard-hangs `setup()` when absent.

Whatever is chosen, footprint the board so the **MCU sits on a module/socket** rather than being
soldered down, so a future change does not mean another respin.

---

## 3. Pin map — current ESP32 firmware (reference)

From `jupiter_config.h`. **Bold = in active use on the 2WD robot.**

⚠ **This is ESP32-WROOM-32 specific.** It records what the robot runs today and is the
authority for *functions the board must provide* — 2 × PWM/DIR, 2 × quadrature encoder,
2 × prox in, 1 × IR out, 1 × ADC, I²C, USB serial. The GPIO numbers themselves only survive if
the ESP32 is retained (§2).

### Drive (retained)

| Function | GPIO | Notes |
|---|---|---|
| **MOTOR1_PWM** | **32** | Left wheel |
| **MOTOR1_DIR** | **33** | |
| **MOTOR1_ENC_A** | **26** | |
| **MOTOR1_ENC_B** | **25** | |
| **MOTOR2_PWM** | **23** | Right wheel |
| **MOTOR2_DIR** | **19** | |
| **MOTOR2_ENC_A** | **18** | |
| **MOTOR2_ENC_B** | **5** | ⚠ strapping pin |

### Motors 3 & 4 — no longer wheels

Still instantiated in firmware (`motor3`, `motor4` objects exist and are set to 0), but drive
nothing. Their pins are the reuse pool.

| Old function | GPIO | Now |
|---|---|---|
| MOTOR3_PWM | 27 | **free** |
| MOTOR3_DIR | 14 | **free** |
| MOTOR3_ENC_B | 12 | **free** — ⚠ strapping pin, see §8 |
| MOTOR3_ENC_A | 13 | **reused → PROX_LEFT** |
| MOTOR4_PWM | 17 | **free** |
| MOTOR4_DIR | 16 | **free** |
| MOTOR4_ENC_A | 4 | **reused → IR_EMIT** |
| MOTOR4_ENC_B | 15 | **reused → PROX_RIGHT** — ⚠ strapping pin |

### Everything else

| Function | GPIO | Notes |
|---|---|---|
| **I²C SDA** | **21** | `Wire.begin(21, 22)`, 400 kHz |
| **I²C SCL** | **22** | |
| **BATTERY ADC** | **34** | ADC1_CH6, 11 dB atten, 12-bit. Input-only pin — no pull-up possible |
| **PROX_LEFT** | **13** | `INPUT_PULLUP`; LOW = metal detected |
| **PROX_RIGHT** | **15** | `INPUT_PULLUP`; LOW = metal detected |
| **IR_EMIT** | **4** | 38 kHz via LEDC ch 4 (0–3 are the motors) |
| **ESP32_LED** | **2** | ⚠ strapping pin |
| USB serial | — | micro-ROS transport, **460800 baud**, appears as `/dev/jupiter_esp32` |

### Free after the 2WD cut

**GPIO 12, 14, 16, 17, 27** — five pins, plus whatever the ToF fan-out doesn't consume.
Ample headroom. GPIO 12 should be spent last, or given a pull-down (§8).

---

### 3.1 PROPOSED pin map — ESP32-S3 (2 drivers, BNO086, no OLED)

Assumes **ESP32-S3-WROOM-1**, native USB, WiFi enabled. **Verify against the datasheet for the
exact module variant before drawing** — pin availability differs between PSRAM options.

**Analog — ADC1 only (GPIO1–10). ADC2 is unusable while WiFi is active.**

| Signal | GPIO | Note |
|---|---|---|
| M1_ISENSE | **1** | ADC1_CH0 — motor current (§4) |
| M2_ISENSE | **2** | ADC1_CH1 |
| *spare ADC* | **4** | thermistor / future |

**Motor drive + encoders**

| Signal | GPIO | Note |
|---|---|---|
| M1_PWM | **21** | LEDC. ⚠ 10 k pull-down |
| M1_DIR | **38** | ⚠ 10 k pull-down |
| M1_ENC_A | **39** | PCNT unit 0 |
| M1_ENC_B | **40** | |
| M2_PWM | **41** | LEDC. ⚠ 10 k pull-down |
| M2_DIR | **42** | ⚠ 10 k pull-down |
| M2_ENC_A | **47** | PCNT unit 1 |
| M2_ENC_B | **5** | ⚠ moved off GPIO48 — see below |

**BNO086 — SPI (FSPI IO_MUX pins)**

| Signal | GPIO |
|---|---|
| IMU_CS | **10** |
| IMU_MOSI | **11** |
| IMU_SCLK | **12** |
| IMU_MISO | **13** |
| IMU_INT | **14** |
| IMU_RST | **15** |

**I²C — mux, ToFs, INA226**

| Signal | GPIO |
|---|---|
| SDA | **16** |
| SCL | **17** |
| MUX_RST | **18** |

**Dock + status**

| Signal | GPIO | Note |
|---|---|---|
| PROX_LEFT | **6** | via protection (§6) |
| PROX_RIGHT | **7** | |
| IR_EMIT | **8** | LEDC 38 kHz → MOSFET (§7) |
| STATUS_LED | **48** | Uses the DevKit's **onboard RGB LED**. GPIO9 now free |

⚠ **GPIO48 carries the onboard RGB LED on most ESP32-S3-DevKitC-1 revisions.** An encoder signal
sharing that net would drive the WS2812's data input — harmless to the LED, but an unnecessary
capacitive load on an odometry signal. `M2_ENC_B` therefore moves to **GPIO5**, and GPIO48 becomes
the status LED, which the DevKit already provides. **GPIO9 is now spare.** Verify the LED pin
against your specific DevKit revision — some use GPIO38.

**23 pins used.** Deliberately untouched: GPIO0/3/45/46 (strapping), 19/20 (native USB),
26–32 (SPI flash), 43/44 (UART0 — keep as a fallback console header). GPIO33–37 are spare **only
on non-octal-PSRAM modules** (they are consumed on N8R8/N16R8) — do not design them in until the
variant is fixed.

#### ▶ The INA226 deletes the battery divider

It measures **bus voltage as well as current**, to 36 V — comfortably covering a 16.8 V pack. That
removes the 100 k/20 k divider, its filter cap, one ADC channel, the ESP32 ADC's non-linearity and
`esp_adc_cal` — **and retires §10.2 entirely**, since the "100 k/20 k or 100 k/22 k?" question stops
mattering. Voltage and current both arrive over I²C, pre-calibrated.

#### Rules that travel with this pin map

1. **10 k pull-downs on all four motor drive pins.** Non-negotiable — a floating-input cold-boot
   wheel spin is on record. Pin numbers change; the physics does not.
2. **No analog outside GPIO1–10.** ADC2 dies whenever WiFi is up.
3. **Bring out UART0 (43/44)** as a header even though micro-ROS runs over USB — fallback console.
4. **MUX_RST on a GPIO**, so firmware can recover a wedged I²C bus without a power cycle.

### 3.2 MCU mounting — DECIDED: socketed DevKit

**DECISION 2026-08-19: socket an ESP32-S3-DevKitC-1 into female headers**, as ver3_1 does with its
ESP32 DevKit. A bare WROOM-1 was considered and the full parts list is retained below, but the
socket is the chosen path.

**Nothing is lost that matters:**

- Every wiring win in this document is unaffected — battery divider, pull-downs, onboard mux, ToF
  connectors, current sensing, three-rail power. None of it cares how the MCU is mounted.
- **USB-Serial-JTAG debugging still works.** The DevKitC-1 has **two USB-C ports** — native USB and
  the UART bridge. Use the native one and you get the §2.3 debug channel that micro-ROS does not own.
- The MCU stays **field-replaceable in seconds**, which matters on a robot that lives in the house.

**What is given up:** the bridge chip stays on the DevKit, so the ver3_1 *"unpowered bridge holds EN
→ floating motor inputs → wheel spin"* mechanism is not structurally deleted. **The §8 motor-drive
pull-downs remain the real fix and are mandatory.**

#### What the board must provide for a socketed DevKit

Four things. That is the entire list.

| Item | Detail |
|---|---|
| **2 × female header** | ESP32-S3-DevKitC-1 is **2 × 22**. Use **machined-pin sockets**, not stamped — this board vibrates and drives into a dock |
| **5 V** to the DevKit's 5V pin | Its onboard LDO derives the module's 3.3 V, exactly as ver3_1 does via J1 |
| **GND** | |
| **Board 3V3 rail** for IMU, mux, ToFs | From the SSP1117 (§11). ⚠ Do **not** draw sensor current from the DevKit's own 3V3 pin — that LDO is sized for the module |

No USB connector, no ESD diodes, no EN RC, no boot/reset buttons, no CC resistors. The DevKit has
all of it.

#### ⚠ Check before finalising the footprint

- **Header pitch and row spacing** against the exact DevKit you buy — clone board outlines vary.
- **GPIO48 / onboard RGB LED** (§3.1) — verify which pin drives it on your revision.
- **Mechanical clearance** for the two USB-C connectors; leave them accessible once mounted.
- **GPIO33–37** are only usable on non-octal-PSRAM DevKits. The §3.1 map avoids them, so an
  **N16R8 DevKit is fine** — its PSRAM simply goes unused.

---

### 3.2b Bare-module parts list — NOT the chosen path, retained for reference

Everything below is required **only** if the socket decision is reversed. It is kept because it is
the harder path to reconstruct later, and because a future revision may take it once the analogue
side and the mux are proven.

#### Group 1 — Module essentials (4 parts)

| Part | Value | Why |
|---|---|---|
| Ceramic cap | **100 nF** | Decoupling, as close to the 3V3 pin as physically possible |
| Bulk cap | **22 µF** | Local energy for WiFi TX bursts — the module draws hundreds of mA in spikes |
| Resistor | **10 kΩ** EN → 3V3 | Pull-up, holds the module out of reset |
| Cap | **1 µF** EN → GND | With the 10 k, forms the **RC power-on reset**. Without it the module can boot before the rail is stable |

#### Group 2 — Boot / reset control (3 parts)

| Part | Detail |
|---|---|
| **Tactile switch** ×2 | **EN→GND** (reset) and **GPIO0→GND** (boot) |
| Resistor **10 kΩ** | GPIO0 → 3V3 pull-up |

Download mode is the classic dance: hold **BOOT**, tap **RESET**, release **BOOT**. In practice
USB-Serial-JTAG usually gets you there without touching either, but you want them for recovery.

#### Group 3 — USB-C, native (4 parts)

| Part | Detail |
|---|---|
| **USB-C receptacle**, 16-pin SMD | e.g. TYPE-C-31-M-12 — widely stocked at LCSC. Prefer 16-pin with through-hole mounting lugs; this connector gets handled |
| **5.1 kΩ** ×2 | **CC1→GND and CC2→GND.** ⚠ **Miss these and the host never enumerates.** They are what declares the board a USB *device*. The single most common first-board USB bug |
| **USBLC6-2SC6** (SOT-23-6) | ESD clamp on D+/D−. Not optional on something that gets plugged and unplugged in a workshop |

Wiring: **D+ → GPIO20, D− → GPIO19**, plus VBUS and GND.
**No auto-reset transistor pair** — USB-Serial-JTAG does reset and download-mode entry natively.
That is one more job the CP2102 existed to do (§2.3).

#### Group 4 — USB vs pack power (decide, do not default)

Two supplies can meet here. Getting it wrong reproduces the ver3_1 failure class.

- ❌ **Do not tie VBUS to the 3V3/5V rail** — back-feed into the buck output.
- ❌ **Do not leave the module unpowered with USB connected** — an unpowered chip with a live host
  on its pins is exactly the condition that let the CP2102 hold EN through Thor's cold boot and
  float the motor inputs into a wheel spin.

Two acceptable answers:

1. **Simplest:** VBUS carries **data only**, never power. The robot must be powered to flash.
   Zero extra parts, zero risk. Perfectly reasonable for a robot that is always on a bench or a dock.
2. **Nicer:** OR the pack rail and VBUS through a **load switch or P-FET ideal-diode**, so the
   module is powered whenever *either* is present and neither back-feeds. Costs ~1 part and lets
   you flash with the pack disconnected.

Either way, the §8 motor-drive pull-downs remain the real defence and are not optional.

#### ⚠ Layout rules — where first boards actually fail

Components are the easy part. These are not:

- **ANTENNA KEEP-OUT.** The WROOM-1's PCB antenna needs a clear zone — **no copper on any layer**
  beneath or beside it, including ground pour. Standard practice is to place the module at a board
  **edge with the antenna overhanging** the outline. Get this wrong and WiFi range collapses, with
  no other symptom. Espressif specifies the exact dimensions.
- **Decoupling placement.** The 100 nF must be at the pin, not "nearby".
- **Strapping pins** — GPIO0, 3, 45, 46 must be free to sit at their default levels at boot (§8).
  GPIO45 selects flash voltage; do not drive it.
- **Test points** on EN, GPIO0, 3V3, GND, TX0/RX0. Cheap now, priceless when it will not boot.

#### 📄 Read these before drawing

Not optional for a first bare-module design:

- **ESP32-S3-WROOM-1 datasheet** — pinout, antenna keep-out dimensions, recommended decoupling
- **Espressif Hardware Design Guidelines (ESP32-S3)** — contains a **reference schematic** that
  covers Groups 1–3 exactly. Copy it rather than deriving it

---

## 4. Motor drive — 2 channels

Keep the ver3_1 topology, it works. Per channel:

- **DRV8870DDAR**, IN/IN mode (`INx` = DIR, `INy` = PWM)
- **200 mΩ** ISEN current-sense resistor
- **100 nF + 10 µF** decoupling at VM
- `VREF` → 3.3 V
- 6-pin **DB128V-5.08-6P-GN-S** connector carrying M+, M−, +3.3 V, ENC_A, ENC_B, GND

**PWM:** 8 kHz, **10-bit — `PWM_MAX` = 1023**. Note this explicitly on the schematic; it has
already caused one external review to compute duty cycles 4× wrong by assuming 8-bit.

**Decision — how many channels to populate:**

| Option | For | Against |
|---|---|---|
| **2 channels only** (recommended) | Smaller, cheaper, less heat, simpler routing | No path back to 4WD without a respin |
| 4 footprints, 2 populated | Keeps 4WD option; JLC can DNP | Larger board, connectors still cost panel space |

Recommendation: **2 channels**. The chassis is committed to 2WD + caster, and the docking
solution (`dock_aligner_v3`) is built around exactly that geometry — arc segments sized around a
single rear caster. Going back to 4WD would invalidate it.

### ▶ ADD: motor current sensing

The DRV8870's **ISEN** pin already develops a voltage across the 200 mΩ sense resistor — add a
current-sense amplifier per channel and that becomes a measurable motor current.

This targets a live defect. Stall detection is currently **inferred from encoder progress rate**,
which is why a segment that actually travelled 0.284 m against a commanded 0.250 m was still
declared `STALLED` (2026-08-14). The wheels were turning at roughly half speed, not locked.

Real current distinguishes the three cases the firmware currently cannot:

| | Current | Encoder progress |
|---|---|---|
| Locked rotor / jam | **high** | none |
| Running slow (loop not settled) | moderate | slow |
| Open circuit / disconnected motor | **none** | none |

That turns `MOVE_STALL_MS` from a timing heuristic into a measurement, and it would have made the
docking failures diagnosable in one run rather than three.

---

## 5. Battery monitoring — DECIDED: INA226, no divider

**DECISION 2026-08-19: the resistive divider is DELETED from the new board.** An **INA226**
provides both bus voltage and current over I²C, pre-calibrated.

#### What that removes

| Gone | Why |
|---|---|
| 100 k/20 k divider + filter cap | INA226 measures bus voltage directly, to 36 V |
| One ADC channel (was GPIO34) | freed |
| `esp_adc_cal` path | the value arrives calibrated over I²C |
| `BATTERY_V_DIV = 0.16510` | the hand-fitted fudge factor disappears |
| **§10.2 entirely** | "100 k/20 k or 100 k/22 k?" stops mattering — see that section |

#### What it adds beyond voltage

- **True charge/discharge current and direction** → an honest `power_supply_status`, which is
  currently hardcoded because the firmware genuinely cannot tell
- **Coulomb counting** → real state-of-charge instead of a voltage guess
- The data to size the docked low-power profile ([[project_operational_modes]])
- Early warning that a docked robot is **net-discharging**, before it goes flat

#### ⚠ OPEN: where does the shunt go?

This decides whether the INA226 answers the question it was added for.

| Shunt position | Measures | Answers "is it net-charging?" |
|---|---|---|
| **On the motion board** | that board's draw only (motors + sensors) | ❌ — Thor's 40–60 W is drawn elsewhere |
| **In the pack's main line** | **total pack current** | ✅ |

The parking-lot item *"measure charge-in vs draw per load"* exists because **the robot drained flat
on the dock twice** — charge-in (~75 W) was less than full-bringup draw, so it net-discharged while
apparently charging. Only total pack current answers that.

The INA226 need not sit at the shunt: it takes VIN+/VIN− across the shunt plus a bus-voltage sense,
so **2–3 thin sense wires** run from the pack distribution point to the board. That is a small
addition against the reduce-wiring goal, and worth it.

**Decide before layout.** Sizing note: total pack current is roughly Thor (~3.5 A at 16.8 V) plus
motors (3.3 A ISEN-limited) plus the rest — size the shunt and its power rating for ~8–10 A, not
for the motion board alone.

---

## 6. Dock proximity sensors — needs protection

**2 sensors** (not 3): `PROX_LEFT` and `PROX_RIGHT`. Firmware encodes them as a bitmask —
**bit0 = left, bit1 = right**, so the "seated" state everyone quotes as `contact=3` simply means
*both engaged*. Charging is gated on it.

**Part:** LJ18A3-8-Z/BX inductive, **12 V supply**, **NPN open-collector** output.
**Firmware:** `INPUT_PULLUP`, HIGH = clear, LOW = metal detected. Debounce 5 × 20 ms cycles.

⚠ **This is the weakest circuit on the robot.** An NPN open-collector output pulls to GND when
active and *floats* when inactive — and it floats **inside a 12 V device**. Today the only thing
holding the pin safe is the ESP32's internal pull-up to 3.3 V. Any leakage, or one wiring error,
puts 12 V on a 3.3 V input.

**Specify per channel:**

```
  SENSOR OUT ──── 10 k ────┬──── GPIO (13 / 15)
                           |
                          BAT54S  →  3.3 V
                           |
                          ─┴─  100 nF
                          GND
```

plus a **4.7 k pull-up to 3.3 V** on the GPIO side, so the input level never depends on the
firmware remembering to enable `INPUT_PULLUP`.

Better still if there's room: an **optocoupler** (e.g. LTV-357T) per channel. Full galvanic
separation between the 12 V sensor loop and the logic, and it removes any question about ground
offsets between the sensor supply and the ESP32.

**Connector:** 3-pin per sensor (+12 V, OUT, GND). Keyed. These get unplugged often.

---

## 7. Dock IR emitter — currently under-driven

**Part:** TSAL6400 + 220 Ω on GPIO4. 38 kHz carrier via LEDC ch 4, burst-gated
(600 µs on/off × 10, then 40 ms gap ≈ 19 packets/s) to keep the dock's TSOP AGC happy.
Fires only while seated **and** below `BATTERY_FULL_STOP` = 16.70 V.

⚠ **Driven directly from a GPIO through 220 Ω from 3.3 V.** That gives roughly
(3.3 − 1.35) / 220 ≈ **9 mA**. The TSAL6400 is rated to 100 mA continuous and its whole value is
optical range. We are running it at under a tenth of what it can do, and the ESP32's 40 mA
per-pin limit means it can never do better on this topology.

**Specify a low-side driver:**

```
  +5 V ──── R(set) ──── TSAL6400 ──── MOSFET drain
                                       (2N7002 / BSS138)
                    GPIO4 ──── 100 Ω ──── gate
                                       source ──── GND
                                       10 k gate pull-down
```

With 5 V and R(set) ≈ 33 Ω this gives ~110 mA peak — well within the LED's pulsed rating at this
duty cycle, and roughly **12× the current optical output**. That is margin for a dirtier dock face
or a wider approach cone, which is exactly what the docking envelope wants.

---

## 8. Strapping-pin pull-downs — do not skip

⚠ **ESP32-specific (§2).** Strapping pins are an ESP32 boot mechanism; RP2350 has a different
boot scheme and this section would be rewritten for it. The *motor-input* pull-downs at the end
of this section apply to EVERY MCU — a floating driver input during reset is undefined on any
part, and on this robot it caused a recorded wheel-spin incident.

This is the item most likely to be silently costing reliability today, and ver3_1 has none of it.

The ESP32 samples several GPIOs at reset to decide boot mode and flash voltage. If anything
external holds them the wrong way at power-up, the chip boots wrong or not at all.

| GPIO | Strapping role | Current use | Required |
|---|---|---|---|
| **12** (MTDI) | **Flash voltage select. HIGH at boot = 1.8 V flash → board does not boot** | free (was MOTOR3_ENC_B) | **10 k pull-DOWN, mandatory** |
| **15** (MTDO) | HIGH at boot = normal; LOW silences boot log | PROX_RIGHT | 10 k pull-**up** to 3.3 V |
| **5** | Must be HIGH at boot | MOTOR2_ENC_B | 10 k pull-up to 3.3 V |
| **2** | Must be LOW/floating at boot | onboard LED | 10 k pull-down |
| **0** | LOW = download mode | (devkit button) | leave to the devkit |

GPIO 12 is the dangerous one. An encoder or ToF cable that happens to sit high at power-up will
make the board appear dead. **Fit the pull-down even if the pin is left unused** — it costs one
resistor and removes an entire class of intermittent fault.

Add pull-downs on **all four motor DIR/PWM lines** as well: during reset the ESP32's pins are
high-impedance, and a floating DRV8870 input is an undefined motor state. A robot that twitches
on power-up is a robot that can drive off a bench.

---

## 9. I²C bus, mux and ToF fan-out

### Current bus

| Device | Address | Notes |
|---|---|---|
| BNO055 IMU | `0x28` | on-board, works |
| ~~SSD1306 OLED~~ | ~~`0x3C`~~ | **DELETED on the new board — see below** |
| TCA9548A mux | `0x70` | **hand-wired, never enumerated — treated as dead** |
| VL53L0X ToF | `0x29` | all identical, hence the mux |

`Wire.begin(21, 22)` at **400 kHz** (BNO055 fast mode).

### SSD1306 OLED — deleted

Fitted on ver3_1 to watch BNO055 figure-8 calibration convergence. **Remove it.** Three reasons:

1. Its job is gone. The BNO086 self-calibrates in the background, and the figure-8 ritual with it.
2. It is **invisible in service** — mounted on chassis level 1, nobody sees it once assembled.
   The 7" panel (`jupiter_display`) is the real user surface.
3. It is a **hard-hang risk**. Parking-lot item: `setup_oled_display()` spins in a `for(;;)` if the
   OLED does not answer, so a dead or unpowered display bricks the whole MCU before micro-ROS
   starts — brain-dead in exactly the safe-debug state. Deleting the part deletes the failure mode.

**Keep the bench capability without the part:** the keyed I²C header needed for the mux doubles as
an OLED port, so one can be plugged in temporarily for bench work. On an ESP32-S3 the
USB-Serial-JTAG console supersedes it entirely (§2.3).

### The mux, honestly

The hand-wired TCA9548A never appeared at any address, with correct 3.3 V (the breakout is
1.65–5.5 V, **no regulator**, so 3.3 V is right), RST high and A0–A2 grounded. A bare VL53L0X
wired to the same pins enumerated at `0x29` immediately, so the bus and the wiring method were
never at fault. The chip is marked **PW548A** — a clone — where a TI part was advertised.

**Put the mux on the PCB.** That removes the entire failure mode:

- TCA9548A (or PCA9548A) in TSSOP-24, JLC-assembled
- 100 nF decoupling at the chip
- A0/A1/A2 hard-grounded → `0x70`
- **RST pulled up 10 k to 3.3 V**, and brought out to a spare GPIO so firmware can reset a wedged bus
- 8 × 4-pin JST-SH or JST-PH connectors (3.3 V, GND, SDn, SCn)

### Pull-ups — get these right

| Segment | Requirement |
|---|---|
| Upstream (MCU ↔ mux, IMU) | **2.2 k** to 3.3 V. 4.7 k is marginal at 400 kHz once several devices and cable capacitance are on the bus |
| Each downstream channel | **4.7 k** to 3.3 V, **on the board**, one pair per active channel |

The TCA9548A does **not** pass pull-ups through — every downstream segment needs its own pair.
Relying on the pull-ups inside each GY-530 breakout works, but the value then depends on which
module is plugged in.

### ToF power budget

7 × VL53L0X at ~20 mA ≈ **140 mA** on 3.3 V, plus the IMU and the mux. That is a material load —
see §11, it is the main argument for replacing the linear regulators.

Keep the ToFs on **3.3 V**. Measured: 22.9 MCPS of return signal on the 3.3 V rail is a perfectly
healthy VCSEL. An earlier theory that the GY-530's onboard LDO was browning out at 3.3 V was
tested and is **wrong**.

### ⚠ Reconsider the ToF part — VL53L5CX multizone

The near-field blind wedge measured on 2026-08-14 is a **single-zone geometry limitation**, not a
mounting error: one ~25° cone means tilting up to reject the floor necessarily blinds you to low
objects nearby. At 85 mm and 8° up-tilt, a shoe-box-height object at 389 mm is invisible.

A **VL53L5CX** (4×4 or 8×8 zones, ~63° FoV) resolves floor and obstacle in **different zones
simultaneously**, so the trade disappears — no tilt compromise, and far fewer sensors needed for
the same coverage. It addresses the measured defect rather than working around it.

Costs to check before committing: larger I²C payload (64 zones), more host processing, higher unit
price, and different footprint/optics. Worth evaluating against VL53L0X while the board is open,
since connector choice and bus budget depend on it.

### ⚠ Mechanical warning — this is not a PCB problem but it will waste a day

The ToFs were completely non-functional in their 3D printed mounts: **~4 % valid readings**,
values uncorrelated with reality. Out of the mount, changing nothing else, the same sensor gave
**175/175 valid**, signal 0.50 → 22.94 MCPS, background 14.20 → 0.04 MCPS.

Optical crosstalk — the printed bore returned the sensor's own laser into its receiver a few mm
away. Because it is fixed to the sensor it is *constant*, so it masquerades as an innocent
"ambient" number and no change of room or target shifts it.

**Whoever prints the new mounts:** black matte filament (light PLA reflects strongly at 850 nm),
aperture **≥ 6–7 mm** chamfered outward, and the module face **flush or proud — never recessed**.
Full detail in `firmware/i2c_scan/src/main.cpp`.

Also unresolved: a large per-profile range bias (DEFAULT read **+80 mm**, LONG-RANGE **−72 mm**,
against a tape-measured 600 mm target). Whatever profile ships must be fixed at build time and
offset-calibrated via `ALGO_PART_TO_PART_RANGE_OFFSET_MM`. **Never switch profile at runtime.**

---

## 10. The three contradictions to resolve first

### 10.1 RESOLVED — VM is a regulated 12 V, and `voltageScale()` is compensating for nothing

**The board is fed 12 V from an external 16.8 V → 12 V buck.** ver3_1's `+12 V` is accurate.
The motors are **rated 12 V DC**, so this rail is not optional — it is what keeps them in spec.

⚠ **This exposes a firmware bug.** The duty compensation reads the **pack**, not the rail:

```c
#define MOTOR_V_NOMINAL   14.4f   // <- describes a rail that does not exist
#define MOTOR_V_COMP_MIN  0.80f   // 16.8 V -> 0.857
#define MOTOR_V_COMP_MAX  1.25f   // 12.0 V -> 1.200
```

`BATTERY_ADC` on GPIO34 measures the 4S pack (confirmed: it read 15.06 V on 2026-08-12, which is a
pack voltage, not a 12 V rail). `voltageScale()` then scales commanded duty by
`MOTOR_V_NOMINAL / V_pack` — but the motors sit behind a regulator and **never see the pack**.

Consequence: across a discharge from 16.8 V to 13 V the scale factor moves 0.857 → 1.108, swinging
commanded duty by roughly **29 % for no physical reason**. Drive behaviour drifts with state of
charge. This is a plausible contributor to the docking repeatability problem, since every trial
ran at a different pack voltage.

**Verify first:** measure VM at a DRV8870 VM pin with the pack full, then again near-empty. If VM
holds ~12 V in both cases, the compensation is spurious.

**Then fix:** set `MOTOR_V_NOMINAL = 12.0` and clamp `voltageScale()` to unity (or delete it).
The mechanism is only correct for an unregulated, pack-fed VM. Note this will shift the effective
duty of the `dock_aligner_v3` tune — **revalidate docking after the change**, it is not a
transparent edit.

### 10.2 ~~Battery divider resistor values~~ — RETIRED 2026-08-19

**No longer applicable.** The divider is deleted from the new board; the INA226 supplies bus
voltage over I²C (§5). Retained only as a record of why the old `BATTERY_V_DIV = 0.16510` differs
from nominal — it was fitted against a multimeter to absorb both the resistor tolerance and the
ESP32 ADC's non-linearity. Both problems vanish with the divider.

<details><summary>original text</summary>



- `CLAUDE.md` says **R1 = 100 k, R2 = 22 k** → ratio 0.1803
- `jupiter_config.h` says *"nominal 20/120"* → **R1 = 100 k, R2 = 20 k** → ratio 0.1667
- Calibrated in firmware: **0.16510**

0.16510 is far closer to the 20 k figure. **Read the actual resistors before drawing the block.**
Getting this wrong shifts every battery reading and the `BATTERY_FULL_STOP = 16.70 V` charge cutoff.

</details>

### 10.3 Triple-defined GPIOs

`13`, `15` and `4` are each defined **twice** in `jupiter_config.h` — once as a motor-3/4 encoder,
once as prox/IR — and the firmware still constructs `motor3_encoder` and `motor4_encoder` on those
same pins. On the physical 2WD robot the prox/IR meaning is the live one, but the stale encoder
objects are still attaching to them.

Harmless today only because motors 3 and 4 drive nothing. **On the new board, delete the motor-3/4
encoder definitions entirely** so the mapping is unambiguous, and clean up the firmware to match.

---

## 11. Power supply — bring the external buck onboard

**The change:** the board is currently fed 12 V by an **external 16.8 V → 12 V buck** sitting
elsewhere on the robot. The new board takes the **raw 4S pack** and generates all three rails
itself. One input, one fuse, one reverse-protection stage, one less box to mount and wire.

### Target architecture

```
  PACK 12–16.8 V ──► fuse ──► ideal-diode FET ──► BUCK #1 ──► 12 V (5 A) ──┬──► DRV8870 VM ×2
                                                                            ├──► prox sensors ×2
                                                                            └──► BUCK #2 ──► 5 V (2 A)
                                                                                     │
                                                                                     ├──► ESP32 DevKit VIN
                                                                                     ├──► IR emitter driver (§7)
                                                                                     └──► SSP1117-3.3 ──► 3.3 V
                                                                                              │
                                                                                              └──► IMU, mux, 7 × ToF
```

**Two switchers and one linear.** 5 V is derived from the 12 V rail rather than from the pack, so
only one stage ever sees pack voltage — fewer high-voltage parts, simpler protection.

The 3.3 V rail stays **linear**, deliberately. It feeds a photon-counting ToF array and sits beside
the 12-bit pack-voltage ADC; switching ripple is the last thing either wants. Fed from 5 V its
dissipation is only (5 − 3.3) × 0.4 A ≈ **0.7 W** — comfortable in SOT-223. **The existing
SSP1117-3.3 is retained**, simply re-sourced from 5 V instead of 12 V.

### Rail loads

| Rail | Feeds | Current |
|---|---|---|
| **12 V** | 2 × DRV8870 VM, prox sensors, buck #2 | **≤ 3.3 A** motors (see sizing) + ~0.2 A |
| **5 V** | ESP32 DevKit VIN, IR driver, J3 | ~250–350 mA |
| **3.3 V** | BNO086, mux, 7 × ToF, VREF | ~200–400 mA |

Note the ESP32 DevKit does **not** sit on the 3.3 V rail — ver3_1 feeds it 5 V into VIN and it
regulates its own 3.3 V onboard.

### Sizing buck #1 — bounded by the existing ISEN design

The DRV8870s are already current-limited by the 200 mΩ sense resistors and the 3.3 V VREF:

```
I_TRIP = V_REF / (10 × R_ISEN) = 3.3 V / (10 × 0.2 Ω) = 1.65 A per channel
```

Two channels → **3.3 A absolute worst case, including stall**. Add ~0.2 A for prox and the 5 V
stage. A **5 A** part gives comfortable margin. *Confirm the factor of 10 against the DRV8870
datasheet before relying on it.*

### Dropout behaves correctly here — no boost needed

A 12 V rail from a 12–16.8 V pack sounds marginal, but with **12 V-rated motors** it is exactly right:

| Pack | 12 V rail | Motors |
|---|---|---|
| 16.8 V (full) | regulated **12.0 V** | at rating |
| ~12.5 V | buck approaches full duty | at rating |
| 12.0 V (flat) | **passes through**, ~12 V | still within rating |

The rail never exceeds 12 V and never needs to exceed its input. Choose a controller supporting
**high / 100 % duty cycle** so it degrades into pass-through rather than hiccuping.

### Buck #1 requirements (12 V @ 5 A)

| Parameter | Requirement |
|---|---|
| V_in | 12–17 V operating, **≥28 V rated** for transient margin |
| V_out | 12.0 V |
| I_out | ≥4 A continuous, 5 A preferred |
| Topology | **Synchronous** — at 71–100 % duty a diode rectifier wastes real power |
| Duty | **High / 100 % pass-through capable** — the requirement most parts fail |

Candidates: **TPS54560** (5 A, 60 V), **TPS54540** (5 A, 42 V), or an MPS equivalent. Verify LCSC
stock and JLC Basic/Extended status before committing — Extended parts carry a per-feeder setup
fee that is material on a one-off run.

**Easiest starting point:** whatever the existing external buck already is. It is proven in this
exact application. Read the part number off it and replicate the topology.

### Buck #2 requirements (5 V @ 2 A from 12 V)

**MP1584EN** (SOIC-8-EP, 3 A, 4.5–28 V) or **AP63205WU-7** (TSOT-23-6, 2 A, 3.8–32 V).
Supporting parts: 10 µH shielded inductor ≥3 A sat; 2 × 10 µF/25 V X7R input + 100 nF;
2 × 22 µF/16 V X7R output; 100 nF bootstrap; feedback divider **52.3 k / 10 k 1 %**
(0.8 V ref → 4.98 V); 100 k enable pull-up.

⚠ Inductor and frequency-set values are a **starting point, not a verified design** — confirm
against the datasheet's typical-application table for the actual input range.

### ⚠ Regenerative kickback — design this in

A buck **cannot sink current**. When the motors decelerate or reverse, energy returns up the rail
and pumps the 12 V node. Previously that energy went back toward the pack through the external
buck's path; with the regulator onboard and the motors behind it, it has nowhere to go.

Add on the 12 V rail, near the drivers:
- **Bulk electrolytic, 220–470 µF / 25 V**
- **TVS or zener clamp at ~15 V**

DRV8870 brake mode shorts the winding and dissipates most of it in the motor, so this is insurance
rather than the primary path — but it is cheap insurance on the rail the whole board hangs off.

### Thermal

At 3.3 A the 12 V rail delivers ~40 W. Even at 93 % efficiency that is ~3 W in the buck stage.
Give the IC and inductor **copper pour and a thermal via array**; do not tuck them under the
DevKit where there is no airflow.

### Switcher layout

- Keep each **switch node** (IC → inductor) as small as physically possible — it is the main radiator.
- Input cap **directly** across the IC's VIN/GND, shortest possible loop.
- Route **feedback traces away from the inductors**; reference to output-cap ground.
- **Physically separate both switchers from the I²C fan-out, ToF connectors and the battery ADC
  node.** Power at the input end, sensors at the far end.

### Input stage

- Retain the **S1206-FA-8.0A** fuse and DBT50-8.25-2P terminal — confirm both are rated for pack
  current at the new input (they were sized for a 12 V input, now carrying similar current).
- Add **reverse-polarity protection**: P-FET ideal diode, *not* a series Schottky — the drop is
  wasted heat at motor currents.
- Retain the rail indicator LEDs (+12 V, +5 V, +3.3 V). Genuinely useful at the bench.

### Lower-risk alternative

Footprint **ready-made buck modules** on 4-pin headers instead of discrete switchers. Zero layout
risk, hand-fitted after assembly, not JLC-assemblable. For a one-off build that is a rational
trade — and for buck #1 it is effectively what the robot already runs.

Retain from ver3_1:
- Input fuse **S1206-FA-8.0A** and the DBT50-8.25-2P terminal
- Rail indicator LEDs (+12 V, +3.3 V, +5 V) — genuinely useful at the bench
- 10 µF bulk per rail

Add:
- **Reverse-polarity protection** on the pack input (P-FET ideal-diode, not a series Schottky —
  the drop is wasted heat at motor currents)
- A **common-mode choke or at least a bulk cap** near the motor connectors. Two DRV8870s switching
  at 8 kHz on the same board as a 400 kHz I²C bus and a photon-counting ToF array is worth
  a moment's layout thought.

---

## 12. Layout notes

- **Keep the ESP32 as a socketed DevKit.** Soldered-down WROOM is neater, but the devkit has been
  reliable, is field-replaceable, and carries a proven USB-serial path at 460800 baud. Not the
  place to introduce risk on a respin.
- **Separate motor ground return from logic ground**, joining at a single star point near the
  input. Motor current sharing an I²C return is a classic source of exactly the intermittent bus
  faults that are so painful to chase.
- **Keep I²C traces short and paired.** The bus now fans out to a mux and up to 7 remote sensors —
  it is no longer a two-device local bus.
- **Silkscreen the pin functions**, including `PWM_MAX = 1023 (10-bit)` and the prox polarity
  (`LOW = contact`). Future-you will thank present-you.
- Mounting holes and outline: match ver3_1 unless the chassis has changed.

---

## 13. Bill of materials — changes from ver3_1

**Remove:**

- 2 × DRV8870DDAR (U3, U4), 2 × 200 mΩ (R3, R4), decoupling C5–C8, 2 × 6-pin connectors
  (CN3, CN4) — the 4WD→2WD cut
- **SSD1306 OLED (OLED1)** — purpose gone with BNO055 calibration, invisible in service, and a
  `setup()` hard-hang risk (§9)
- **BNO055** — superseded by BNO086 (§2.5)
- **CP2102** — not on the board either way; the DevKit carries its own (§3.2)
- **Battery divider (R1/R2 + filter cap)** — replaced by the INA226 (§5)

**Add:**

| Item | Qty | Note |
|---|---|---|
| **BNO086 IMU** | 1 | **§2.5.** SPI preferred; bring out INT + RST. Place AWAY from motors/power stage |
| **INA226 + shunt** | 1 | **§5.** Bus voltage AND current — replaces the divider entirely. ⚠ Shunt position still open: motion board vs pack main line |
| **Current-sense amplifier** | 2 | **§4.** One per DRV8870 ISEN — turns stall detection from a heuristic into a measurement |
| TCA9548A / PCA9548A, TSSOP-24 | 1 | **TI or NXP part — reject PW548A clones** |
| 4-pin JST connectors (ToF) | 8 | one per mux channel. **Confirm pinout against the chosen ToF — VL53L5CX is under evaluation (§9)** |
| Keyed I²C bench header | 1 | doubles as the temporary-OLED port (§9) |
| **ESP32-S3-DevKitC-1** | 1 | §3.2. Socketed. Any flash size; **N16R8 is fine** — the §3.1 map avoids GPIO33–37, so its PSRAM simply goes unused |
| **Female headers, 2 × 22, machined-pin** | 2 | §3.2 — machined not stamped; this board vibrates |
| 3-pin connectors (prox) | 2 | 12 V, keyed |
| BAT54S clamp diodes | 3 | 2 × prox, 1 × battery ADC |
| 2N7002 / BSS138 | 1 | IR emitter driver |
| Buck #1 IC, 12 V @ 5 A sync (TPS54540/54560 class) | 1 | **replaces the external 16.8→12 V buck** |
| Buck #2 IC, 5 V @ 2 A (MP1584EN / AP63205) | 1 | fed from 12 V; SSP1117-3.3 is RETAINED, re-sourced from 5 V |
| Inductors, shielded | 2 | one per buck; #1 sat rating ≥ 1.5× peak motor current |
| Buck passives (caps, FB dividers, BST) | ~16 | see §11 |
| Bulk electrolytic 220–470 µF / 25 V | 1 | 12 V rail, motor regen |
| TVS / zener clamp ~15 V | 1 | 12 V rail, motor regen |
| Resistors: 10 k pull-down/up (strapping) | ~8 | §8 |
| Resistors: 2.2 k (upstream I²C pull-up) | 2 | |
| Resistors: 4.7 k (per-channel I²C pull-up) | 16 | 2 per active mux channel |
| 100 nF decoupling | ~6 | mux, ADC node, prox inputs |
| Ideal-diode reverse protection FET | 1 | |

---

## 14. Open questions for Logan

1. **7 ToFs or 8?** The mux has 8 channels. Are all 7 front-facing, or are some cliff/rear?
   Placement drives connector positions on the board edge.
2. **Is the 12 V rail still needed at all** if VM turns out to be pack-fed (§10.1)? The prox sensors
   need 12 V, so probably yes — but it may become a small dedicated supply rather than the main rail.
3. **Board outline / mounting** — unchanged from ver3_1?
4. **JLC assembly scope** — full assembly, or hand-solder the connectors? Affects whether the mux
   must be a JLC "basic" part.

---

*Written 2026-08-12. Pin data from `firmware/esp32/include/jupiter_config.h` @ `ba0a7fe`.
Predecessor: `Jupiter ESP32 Drv8870 12vDC ver3_1`, EasyEDA, JLCPCB-002, 2024-06-29.*
