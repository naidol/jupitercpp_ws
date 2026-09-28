# New Motion Control Board — 2026

**Purpose:** design brief for the successor to `Jupiter ESP32 Drv8870 12vDC ver3_1`
(EasyEDA, JLCPCB-002, drawn 2024-06-29). Captures every change made to the robot since that
board was fabricated, so the schematic can be redrawn and sent to JLCPCB in one pass.

**Status:** in schematic capture (EasyEDA). PSU and IMU sheets drawn; MCU sheet drawn.
**MCU is FIXED: ESP32-S3-DevKitC-1 (N16R8), socketed** — decided §3.2. The MCU-neutral language
in §2 is retained as the record of how that was chosen, not as a live option.
**Predecessor schematic:** `~/Documents/Jupiter_ESP32_Board_easyeda.pdf`.

---

## ⭐ THIS FILE IS THE SINGLE SOURCE OF TRUTH

Everything about the 2026 board redesign lives here. The standalone
`ESP32_S3_MCU_JUPITER_PINOUT.md` was **absorbed into §3.1 on 2026-08-24** and reduced to a stub —
maintaining two pin maps is what let them drift apart in the first place.

**If any two sections disagree, the AS-BUILT sections below win.** Everything else is design
rationale and may lag.

| Need | Go to |
|---|---|
| **Every MCU pin, in header order** | **§3.1** — the pin map. Nothing else records pin facts |
| **BNO086 breakout wiring** | **§3.1** — 16-pin table, `PS0`/`WAK` circuit, continuity checks |
| **Power rails, as built** | **§11** — three parallel bucks, verified feedback dividers |
| Connector pinouts, net-by-net | §3.3 |
| Why the BNO086 and not the BNO055 | §2.5 |
| Why there are no spare GPIO | §3.1 |
| What pack-current monitoring costs | §5 |

**Decisions reversed during design — do not re-open without reading why:**

| Was | Now | Where |
|---|---|---|
| INA226 replaces the battery divider | **Divider retained**, on-board, GPIO4 | §5, §10.2 |
| 3.3 V from an SSP1117 linear off 5 V | **Third MP1584 buck off VCC** | §11 |
| Cascade `VCC → 12 V → 5 V → 3.3 V` | **Three bucks in parallel off VCC** | §11 |
| Buck #1 must be synchronous | **Non-sync is correct here** — reasoning was backwards | §11 |
| `M1_DIR` on GPIO38 | **GPIO48** — 38 is the onboard RGB | §3.1 |
| 5 V rail at 4.98 V (52.3 k/10 k) | **5.09 V (118 k/22 k)** — leave it | §11 |

> ### ⚠ Before drawing anything
> Contradictions between the old schematic, the firmware and the docs. Cheap to settle with a
> multimeter, expensive to get wrong on a fabricated board. Detailed in §10.
>
> 0. **MCU and IMU are NOT fixed** — ESP32-WROOM-32 / ESP32-S3 / RP2350, and BNO055 vs BNO085.
>    See §2. Everything else in this document is MCU-independent and can proceed regardless.
> 1. ~~What feeds DRV8870 `VM`?~~ **RESOLVED** — a regulated **12 V** from an external
>    16.8 V → 12 V buck. That buck moves **onto** the new board (§11). See §10.1 for the firmware
>    bug this exposes.
> 2. ~~Battery divider resistor values?~~ **SETTLED 2026-08-22** — the INA226 is **not fitted**;
>    the divider stays, moves on-board, and is **100 k / 20 k** on GPIO4 (§5). Open in its place:
>    **nothing measures pack current**, so "is the docked robot net-charging?" remains unanswered
>    → parking lot.
> 3. Three GPIOs are defined twice in firmware (13, 15, 4). Confirm which function is physically wired.

---

## 1. What changed since ver3_1

| # | Change | Consequence for the new board |
|---|---|---|
| 1 | **4WD → 2WD.** Two driven wheels + one rear caster (180 mm behind the drive axle) | Two DRV8870 channels are now dead weight |
| 2 | **Battery monitoring added.** Resistive divider → GPIO34 | Currently off-board / flying. Needs to be a designed circuit |
| 3 | **Dock proximity sensors added.** 2 × LJ18A3-8-Z/BX inductive, **12 V**, NPN open-collector | 12 V logic arriving at a 3.3 V pin with no protection |
| 4 | **Dock IR emitter added.** TSAL6400, 38 kHz — **GPIO8** on the new board (was GPIO4 on ver3_1) | Driven straight from a GPIO. Under-driven, see §7 |
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

**Mounting — DECIDED 2026-08-24: socket it on the board.** 2 × 8-pin **machined-pin female
sockets**, same approach as the DevKit (§3.2). Removable, replaceable, no chassis wiring.

**Escape hatch — the socket already is one.** If measurement says the IMU must move, pull the
module and plug a pin-header-terminated ribbon into the same socket. Crude, but it costs zero
board area and zero parts today.

For a *clean* relocation option, footprint a dedicated connector in parallel and populate one or
the other. ⚠ **It is 10 conductors, not 6 or 8** — earlier drafts undercounted:

`CS · SI · SO · SCK · INT · RST · WAK/PS0 · PS1 · 3V3 · GND`

A 2×5 IDC or 10-way JST. Drop to 9 by strapping `PS1` to 3V3 at the remote end. Cost: one
connector and a few cm², so the placement decision does not have to be right first time.

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

### 3.1 GPIO ALLOCATION — ⭐ THE SOURCE OF TRUTH

> **MCU: ESP32-S3-WROOM-1U-N16**, soldered (§3.2). **Updated 2026-08-26.**
> **EDA: KiCad v10.** Passives **0805**.
>
> ⚠ **This table lists GPIO, NOT module pad numbers.** Take pad numbers from KiCad's official
> `RF_Module:ESP32-S3-WROOM-1` symbol/footprint — maintained against the Espressif datasheet. Do
> **not** hand-transcribe a pad list or trust a third-party library; that is the exact class of
> error chased on the DRV8870 symbol.
>
> **Every pin fact lives here and nowhere else.** If another section disagrees, this table wins.

| GPIO | Signal | Connects To | PU / PD | Value | Notes |
|---|---|---|---|---|---|
| **0** | — | — | — | — | **NC.** Strapping. BOOT button |
| **1** | M1_ISENSE | U14-3 via R39 | — | — | ADC1_0 |
| **2** | M2_ISENSE | U14-5 via R40 | — | — | ADC1_1 |
| **3** | — | — | — | — | **NC.** Strapping (JTAG sel) |
| **4** | BATT_SENSE | On-board divider tap | — | — | ADC1_3. 100 k/20 k from VCC (§5) |
| **5** | **BOARD_TEMP** | NTC divider | — | — | ▶ **ADC1_CH4.** 10 k NTC (B3950) to 3V3, 10 k to GND, 100 nF. Added 2026-09-15 |
| **6** | PROX_LEFT | CN3 pin 3, via 10 k series | **PU** | 4.7 kΩ | ⚠ 4k7 on the **SENSOR** side (§6) |
| **7** | PROX_RIGHT | CN3 pin 6, via 10 k series | **PU** | 4.7 kΩ | ⚠ 4k7 on the **SENSOR** side (§6) |
| **8** | IR_EMIT | Q1 gate, via 100 Ω | **PD** | 10 kΩ | PD at the **gate** (§7) |
| **9** | IMU_WAKE | BNO086 `WAK` | **PU** | 10 kΩ | `R29` — shared node with `PS0` |
| **10** | IMU_CS | BNO086 `CS` | **PU** | 10 kΩ | FSPICS0. `R27` |
| **11** | IMU_MOSI | BNO086 `SI` | — | — | FSPID |
| **12** | IMU_SCLK | BNO086 `SCK` | — | — | FSPICLK |
| **13** | IMU_MISO | BNO086 `SO` | — | — | FSPIQ |
| **14** | IMU_INT | BNO086 `INT` | — | — | Active low |
| **15** | IMU_RST | BNO086 `RST` | **PU** | 10 kΩ | Active low. `R28` |
| **16** | I2C_SDA | TCA9548A · J-I2C | **PU** | **2.2 kΩ** | 4k7 too weak at 400 kHz |
| **17** | I2C_SCL | TCA9548A · J-I2C | **PU** | **2.2 kΩ** | |
| **18** | MUX_RST | TCA9548A `RESET` | **PU** | 10 kΩ | Active low |
| **19** | — | USB-C D− | — | — | Own receptacle now (§4) |
| **20** | — | USB-C D+ | — | — | |
| **21** | M1_PWM | DRV8870 #1 **IN2** | **PD** | **10 kΩ** | ⚠ Mandatory |
| **35** | **SPARE** | J-EXP | — | — | ▶ Freed by N16 (no PSRAM) |
| **36** | M2_ENC_A | CN2 pin 5 | — | — | ▶ **moved from IO47** 2026-09-07 |
| **37** | M2_ENC_B | CN2 pin 6 | — | — | ▶ **moved from IO5** 2026-09-07 |
| **38** | STATUS_LED | **Plain LED + series R** | — | — | ▶ Freed — no DevKit RGB. ⚠ **Not addressable** — see below |
| **39** | M1_ENC_A | CN1 pin 5 | — | — | |
| **40** | M1_ENC_B | CN1 pin 6 | — | — | |
| **41** | M2_PWM | DRV8870 #2 **IN2** | **PD** | **10 kΩ** | ⚠ Mandatory |
| **42** | M2_DIR | DRV8870 #2 **IN1** | **PD** | **10 kΩ** | ⚠ Mandatory |
| **43** | U0TXD | J-CONSOLE | — | — | |
| **44** | U0RXD | J-CONSOLE | — | — | |
| **45** | — | — | — | — | **NC.** Strapping (VDD_SPI) |
| **46** | — | — | — | — | **NC.** Strapping (LOG) |
| **47** | **SPARE** | J-EXP | — | — | ▶ freed 2026-09-07 |
| **48** | M1_DIR | DRV8870 #1 **IN1** | **PD** | **10 kΩ** | ⚠ Mandatory |

**27 GPIO assigned · STATUS_LED on 38 · 2 SPARES (35, 47) · strapping 0/3/45/46 left NC.**

#### ▶ Encoder regroup — 2026-09-07

`M2`'s two encoder channels were split across opposite corners of the module: `M2_ENC_A` on IO47
(bottom edge) and `M2_ENC_B` on IO5 (left edge). **Both arrive on one cable from CN2 pins 5 and 6**,
so one of them had to cross the whole module. `M1`'s pair (IO39/IO40) was already adjacent.

| Signal | Was | Now | Module pin |
|---|---|---|---|
| `M2_ENC_A` | IO47 | **IO36** | 29 |
| `M2_ENC_B` | IO5 | **IO37** | 30 |

All four encoder lines now sit on the **right edge** at module pins 29, 30, 32, 33 — one cable run
per motor, no crossings.

**And it upgrades the spares.** IO5 is **ADC1_CH4** — one of only ten ADC1 pins, previously spent on
a digital encoder input. Spares are now one per edge, one analogue-capable:

| Spare | Edge | Capability |
|---|---|---|
| ~~IO5~~ | left | ▶ **SPENT 2026-09-15 on BOARD_TEMP** — the NTC it was freed for |
| IO35 | right | digital only |
| IO47 | bottom | digital only |

⚠ **`IMU_RST` deliberately stays on IO15.** Moving it to the bottom edge with the rest of the IMU
was considered and **rejected**: the only free bottom pin is IO47, which sits between IO21 (`M1_PWM`)
and IO48 (`M1_DIR`). A coupled spike on a static, 10 k-pulled-up reset line would reset the IMU
intermittently. IO15's neighbours on the left edge (IO16/IO17 I²C) are the sensor side anyway, which
is where §2.5 places the BNO086.

#### ▶ What the module change bought

| | DevKit N16**R8** | Module N16 |
|---|---|---|
| GPIO35/36/37 | consumed by octal PSRAM | **FREE — 3 spares** |
| GPIO38 | onboard RGB LED | free; spent on your own STATUS_LED |
| Spare GPIO | **0** | **3** (one per edge; IO5 is ADC-capable) |
| `J-EXP` | power tap, no I/O | **real 5-pin header: 3 GPIO + 3V3 + GND** |

**GPIO33/34 are NOT brought out** on WROOM-1/1U — do not design them in.

▶ **ADC2 is now usable.** The "all analogue on GPIO1–10" rule existed only because ADC2 shares
hardware with the WiFi radio. **WiFi is never enabled on this robot** (§3.2), so GPIO11–20 are valid
ADC inputs too. All are currently allocated, but it is a real degree of freedom if one frees up.

#### Board thermistor — `BOARD_TEMP` on IO5, added 2026-09-15

```
   +3.3V ──[RT1 NTC 10k B3950]──┬──────────► IO5 (ADC1_CH4)
                                ├──[R 10k]── GND
                                └──[C 100nF]─ GND
```

Voltage **rises** with temperature. 0 °C → 0.76 V · 25 °C → 1.65 V · 50 °C → 2.43 V ·
70 °C → 2.81 V · 85 °C → 2.98 V. Self-heating 0.27 mW, negligible.

**Why:** Jupiter is always-on inside an **aluminium chassis** with poor convection, carrying two
motor drivers and two switchers, and nothing measured board temperature. The HUD already shows CPU
and GPU temps; this completes it. A stalled motor cooking a DRV8870 or a buck running hot was
previously invisible until something failed.

⚠ **Placement decides whether it is worth fitting.** A thermistor in a quiet corner measures the
air in a box. Put it **beside the TPS54560 and its inductor** (~3 W at full motor load, the largest
single dissipator — §11) or between the two DRV8870s. Keep it **off the ground pour feeding the
switcher's thermal vias**, or it reads the plane rather than the board.

⚠ **Use the NTC's ACTUAL B value in firmware.** A B = 4100 substitute reads ~3 °C off at 70 °C if
the code assumes 3950. The circuit does not change; the constant does.

```cpp
float r_ntc  = R_FIXED * (3.3f / v_adc - 1.0f);
float temp_k = 1.0f / (1.0f/298.15f + (1.0f/BETA) * logf(r_ntc / 10000.0f));
```

#### Spare GPIO — no external pulls

⚠ **IO35 and IO47 need NO pull-up or pull-down.** The boot-window argument that makes the motor
pull-downs mandatory does not apply: a floating spare on an empty header drives nothing. Use the
**ESP32's internal pulls** — `pinMode(35, INPUT_PULLDOWN)` — which costs no parts and does not
constrain a future use. An external 10 k would have to be fought by any later device wanting the
opposite polarity.

▶ **AS BUILT: 330 Ω series resistor per pin** (`R35`, `R36`). Limits a 3V3-into-GPIO miswire on the
4-pin header to ~10 mA. Pin order is `3V3 · IO35 · IO47 · GND` — power at the ends, signals in the
middle. The internal pull-downs are unaffected — they sit on the chip side of the
resistor.

⚠ Consequence: 300 Ω plus cable capacitance is an RC. At ~100 pF that is a 30 ns corner — invisible
for slow digital, switches and interrupts, but **bypass the resistors if a fast bus ever goes on
this header**.

#### Pull-downs — 5 × 10 kΩ to GND

Between reset and the first `pinMode()`, every GPIO is high-Z. These are safety, not tidiness.

| GPIO | Signal | Why |
|---|---|---|
| **21, 48, 41, 42** | M1/M2 PWM + DIR | ⚠ Floating DRV8870 inputs are undefined. Both low = **COAST**. **Cold-boot wheel spin is on record.** ▶ **These four live on the MOTOR DRIVER sheet, placed at the DRV8870 input pins** (§3.3) — not on the MCU sheet |
| **8** | IR_EMIT | Holds Q1 off through boot so the beacon cannot ask the dock to energise. **At the MOSFET gate, not the pin** |

#### Pull-ups

| GPIO | Value | Why |
|---|---|---|
| **16, 17** | **2.2 kΩ** | I²C is open-drain — no device drives high. Value set by bus capacitance: mux + J-I2C + 7 cabled ToF segments. **4.7 k cannot meet the 300 ns rise-time spec at 400 kHz** with that load. 2.2 k sinks ~1.5 mA, inside every device's 3 mA rating |
| **6, 7** | 4.7 kΩ | LJ18A3 is NPN open-collector — sinks only, floats when inactive. ⚠ **On the SENSOR side of the 10 k series** (§6) |
| **10** `R27` | 10 kΩ | IMU `CS` active-low — holds the BNO086 deselected through the boot float window |
| **15** `R28` | 10 kΩ | IMU `RST` active-low |
| **18** | 10 kΩ | Mux `RESET` active-low; also lets firmware recover a wedged bus |
| **9** `R29` | 10 kΩ | Straps `PS0 = 1` (SPI mode) **and** holds `WAKE` deasserted. Cannot be a hard tie — `PS0`/`WAK` share a die pad |

#### BNO086 breakout — every pin, by silkscreen name

**Part: SparkFun BNO086 breakout, 16-pin.** Named by **silkscreen**, not die pin.

> ⚠ **Silkscreen vs symbol:** the board labels pin 14 **`SCK`**; some symbols label it `SDK`.
> ⚠ **Clone modules:** verify pin order, 0.1" pitch and row spacing on the physical part.
> **The Qwiic connectors are unused** — JST-SH 4-pin **I²C only** (GND · 3V3 · SDA · SCL), the two
> ports wired in parallel for daisy-chaining. No SPI on them. The board's **`I2C` solder jumper** is
> the pull-up jumper — cut it if the 5↔12 continuity check beeps.

| # | Silk | Goes to | Resistor |
|---|---|---|---|
| 1 | `PS0` | **R29 node** | 10 k PU → 3V3 |
| 2 | `PS1` | **3V3** | ⚠ own branch — must **not** share R29's node |
| 3 | `GND` | GND | — |
| 4 | `3V3` | 3.3 V rail | `C28` 100 nF + `C29` 10 µF **ceramic X7R 0805**, behind ferrite `U8` — LCSC `C21519` TDK MPZ2012S601AT000, 0805, 600 Ω@100 MHz, 100 mΩ (§11) |
| 5 | `SDA` | **NC** | I²C only |
| 6 | `SCL` | **NC** | I²C only |
| 7 | `RST` | **NC** | same net as 9 |
| 8 | `INT` | **GPIO14** | — |
| 9 | `RST` | **GPIO15** | 10 k PU (`R28`) |
| 10 | `WAK` | **GPIO9** | via R29 node |
| 11 | `CS` | **GPIO10** | 10 k PU (`R27`) |
| 12 | `SI` | **GPIO11** | MOSI |
| 13 | `SO` | **GPIO13** | MISO |
| 14 | `SCK` | **GPIO12** | SCK |
| 15 | `3V3` | **NC** | ⚠ same net as 4 — leave open or the ferrite is shorted |
| 16 | `GND` | GND | — |

**Protocol select: SPI is `PS1 = 1, PS0 = 1`.** Verify against the BNO08x datasheet.

⚠ **`PS0` strapped through R29, never hard-wired.** `PS0` and `WAKE` share a die pad. Hard-tie
`PS0` to the rail *and* drive `WAK` from GPIO9 and every wake pulse shorts the ESP32's output driver.

✅ **CONFIRMED on the physical module 2026-08-27: breakout pin 1 (`PS0`) and pin 10 (`WAK`) are the
same net.** So the single 10 k on `PS0` serves as both the SPI-mode strap *and* the `WAKE` pull-up,
and GPIO9 drives that shared node. **No separate pull-up on `WAK` is needed** — do not add one.

⚠ **`PS1` on its own branch.** Sharing R29's node means asserting WAKE drags `PS1` low, and
`PS1:PS0 = 0:0` is **I²C mode** — latched at reset, so a reset during WAKE boots the wrong protocol.

⚠ **Ferrite bead placement (§11):** in series between the 3.3 V rail and the IMU's whole local
domain, `C28`/`C29` on the **IMU side**. Tapping pin 4 upstream of the bead strands the decoupling
behind 600 Ω of HF impedance — **worse than no bead**. And **pin 15 must be NC** or current routes
in through it and straight past the filter.

**Continuity checks on the physical breakout:**

| Check | Status |
|---|---|
| 1 `PS0` ↔ 10 `WAK` | ✅ **CONFIRMED same net** 2026-08-27 |
| 5 `SDA` ↔ 12 `SI` | ⬜ open — if it beeps, **cut the breakout's `I2C` pull-up jumper** |
| 7 `RST` ↔ 9 `RST` | ⬜ open — leaving 7 NC is only safe if these are one net |
| 4 `3V3` ↔ 15 `3V3` | ⬜ open — expected to beep; confirms pin 15 must stay NC |

**Escape valve:** move the BNO086 to **I²C** on GPIO16/17 — frees GPIO9/10/11/12/13, five pins, for
zero parts. Fitting `PS0`/`PS1` as jumper links preserves this.

#### Rules that travel with this map

1. **10 k pull-downs on all four motor drive pins.** Non-negotiable — cold-boot wheel spin is on record.
2. **Analogue prefers GPIO1–10** (ADC1). ADC2 is available only because WiFi stays off — do not
   design analogue onto it without re-reading that assumption.
3. **UART0 (43/44) brought out** as `J-CONSOLE` even though micro-ROS runs over USB.
4. **MUX_RST on a GPIO**, so firmware can recover a wedged I²C bus without a power cycle.
5. **Three spares, and that is all.** Anything further goes on I²C behind the mux.

### 3.2 MCU — DECIDED: ESP32-S3-WROOM-1U-N16, soldered

**DECISION 2026-08-26. This REVERSES the 2026-08-19 socketed-DevKit decision.**

**Part: ESP32-S3-WROOM-1U-N16.** Both suffixes are load-bearing:

- **`U` = external antenna (U.FL/IPEX), no PCB antenna.** The chassis is **aluminium profile**, which
  would cripple a PCB antenna anyway — and the WROOM-1's trace antenna demands a **keep-out**: no
  copper, no traces, no pour, ideally overhanging the board edge. On a 100×100 with two bucks, two
  drivers and a connector bank, surrendering a corner *and* forbidding routing through it is a real
  cost. The 1U deletes that constraint and lets the module sit mid-board with pour underneath.
- **`N16` = 16 MB flash, NO PSRAM.** This is what frees **GPIO35/36/37** (§3.1). An `R8` part
  consumes all three for octal PSRAM, which is why the DevKit had zero spares.

**WiFi is never enabled.** micro-ROS runs over native USB; nothing on this robot needs the radio.
An unconnected U.FL is therefore harmless — and it is what makes ADC2 usable (§3.1).

#### What this trades away

⚠ **Field-replaceability.** §3.2's earlier decision socketed the DevKit precisely so *"the MCU stays
field-replaceable in seconds, which matters on a robot that lives in the house."* Soldered, a dead
ESP32 is a dead motherboard. **Accepted deliberately** in exchange for ~750 mm² of board and 3 GPIO.

#### What is already INSIDE the module — do not fit these

| Integrated | Consequence |
|---|---|
| **40 MHz crystal + load caps** | ⚠ **No external oscillator.** A bare QFN56 would need one; the module does not |
| **16 MB SPI flash** (N16) | No external flash, and GPIO26–32 stay internal |
| RF matching network + U.FL | No RF layout to control |
| Chip-level decoupling | Still add your own at the 3V3 pin (below) |

▶ **The optional 32.768 kHz RTC crystal is NOT in the module**, and its pins on the S3 are
**GPIO15/GPIO16** — both allocated here (IMU_RST, I2C_SDA). That door is closed and it does not
matter: a 32 kHz crystal only buys accurate timekeeping across **deep sleep**, and this ESP32 never
sleeps — it runs micro-ROS continuously.

#### What the board must now provide itself

The DevKit supplied all of this. The bare module does not:

| Item | Detail |
|---|---|
| **USB-C receptacle** | 16-pin, through-hole lugs. D+ → GPIO20, D− → GPIO19 |
| **2 × 5.1 kΩ** | CC1→GND, CC2→GND. ⚠ **Miss these and the host never enumerates.** The single most common first-board USB bug |
| **USBLC6-2SC6** | ESD clamp on D+/D− |
| **EN RC** | 10 kΩ pull-up to 3V3 + 1 µF to GND. ⚠ *"Do not leave the EN pin floating"* — Espressif |
| **BOOT + RESET buttons** | Or at minimum pads. No auto-reset transistor pair needed — USB-Serial-JTAG does reset and download-mode entry natively (§2.3) |
| **STATUS_LED** | GPIO38 — **plain LED + series resistor** (1 kΩ ≈ 1.3 mA, or 330 Ω for bright). ⚠ **Not WS2812/SK6812**: both need ≥3.5 V and there is no 5 V rail to level-shift from. Firmware moves `neopixelWrite()` → `digitalWrite()`/LEDC blink patterns. Colour is no loss — the 7" HUD carries mode, sensor health, temps and battery; this is a bring-up/heartbeat indicator. An RGB LED would cost 3 GPIO — **all three spares** — and kill `J-EXP` |
| **3.3 V** | The module has **no onboard LDO** — it runs directly off buck #2 (§11) |

#### ⚠ Assembly

**Hot air (Atten ST-862) + trinocular scope available; SMT likely machine-assembled.** That is what
makes the module and the MP2338's SOT583 viable.

The module's castellations are hand-solderable, but there is an **EPAD underneath** for ground and
thermal that an iron cannot reach. Either reflow it, or put a **hole in the PCB under the EPAD** so
it can be soldered from below. Relying on castellations alone for ground is off-spec.

**Passives are 0805** throughout (was 1206) — smaller, still comfortable under the scope.

### 3.2b Bare-module implementation — ⭐ THE CHOSEN PATH (2026-08-26)

> Previously headed *"NOT the chosen path, retained for reference"*. **The socket decision was
> reversed on 2026-08-26** (§3.2), so everything below is now **required**, not optional.

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

**Wiring — DATA ONLY, VBUS unconnected (§3.2b Group 4):**

| Net | To |
|---|---|
| D+ | **GPIO20** |
| D− | **GPIO19** |
| **CC1** | **5.1 kΩ → GND** |
| **CC2** | **5.1 kΩ → GND** |
| USBLC6 `Vbus` pin | ⚠ **+3.3V** — see below |
| Shell tabs | GND |
| **VBUS (4 pins)** | **unconnected** |

⚠ **The CC resistors are mandatory precisely BECAUSE VBUS is unused.** They are how the board
declares itself a USB device (UFP). **Thor has USB-C ports**, so on a C-to-C cable the host detects
attachment *entirely through CC* — no Rd on this side and it never enumerates, never even looks at
D+/D−, with no diagnostic. (An A-to-C cable might still work, since the cable carries its own 56 kΩ
CC pull-up and a legacy host detects via the D+ pull-up — so this fails on one cable and not
another, which is a bad afternoon.) **Two separate resistors, one per CC pin** — never one bridging
both, that breaks orientation detection.

⚠ **The USBLC6-2SC6's `Vbus` pin is the positive CLAMP RAIL, not a power input.** With VBUS
unconnected there is nothing to reference it to, so tie it to **+3.3V** — which is also correct,
since D+/D− only swing 0–3.3 V. Tighter clamping than referencing 5 V, and ESD energy dumps into a
rail with bulk capacitance behind it. Left on a floating VBUS net it looks wired and does nothing.

**Not needed:** SBU, SSTX/SSRX (USB 2.0 only — hence the 16-pin receptacle), and no auto-reset
transistor pair.

**Operational consequence:** the board must be powered from the pack to flash. The ESP32-S3's
USB-Serial-JTAG needs no VBUS sensing — it enables its D+ pull-up whenever the module is powered,
the host sees it, enumeration proceeds.
**No auto-reset transistor pair** — USB-Serial-JTAG does reset and download-mode entry natively.
That is one more job the CP2102 existed to do (§2.3).

#### Group 4 — USB vs pack power — ▶ RESOLVED 2026-08-26

Two supplies can meet here. Getting it wrong reproduces the ver3_1 failure class.

- ❌ **Do not tie VBUS to the 3V3 rail** — back-feed into the buck output.
- ❌ **Do not leave the module unpowered with USB connected** — an unpowered chip with a live host
  on its pins is exactly the condition that let the CP2102 hold EN through Thor's cold boot and
  float the motor inputs into a wheel spin.

⭐ **DECIDED: VBUS carries data only, never power.** Leave the receptacle's VBUS pin unconnected.
Zero parts, zero risk, no back-feed in either direction. The robot must be powered to flash — true
of every bench and dock session anyway. **This option was impossible with a socketed DevKit**, whose
VBUS lives inside a board you cannot modify; owning the receptacle is what makes it available.

*Alternative if you ever want to flash with the pack disconnected:* OR the pack rail and VBUS
through a **P-FET ideal-diode**. ~1 part.

⚠ **The `D5` SS34 ORing Schottky analysis is OBSOLETE.** It solved this problem for a socketed
DevKit's 5V pin. There is no 5 V rail and no 5 V input any more (§11).

Either way, the §8 motor-drive pull-downs remain the real defence and are not optional.

#### ⚠ Layout rules — where first boards actually fail

Components are the easy part. These are not:

- ~~**ANTENNA KEEP-OUT**~~ — **N/A.** The **1U** variant has no PCB antenna, which is precisely why
  it was chosen (§3.2): no keep-out, no copper exclusion, no forced board-edge placement. The module
  may sit mid-board with pour underneath. This was the single largest layout constraint and it is
  gone. (If a plain WROOM-1 is ever substituted, the keep-out comes back and it is unforgiving —
  WiFi range collapses with no other symptom.)
- **Decoupling placement.** The 100 nF must be at the pin, not "nearby".
- **Strapping pins** — GPIO0, 3, 45, 46 must be free to sit at their default levels at boot (§8).
  GPIO45 selects flash voltage; do not drive it.
- **Test points** on EN, GPIO0, 3V3, GND, TX0/RX0. Cheap now, priceless when it will not boot.

#### 📄 Read these before drawing

Not optional for a first bare-module design:

- **ESP32-S3-WROOM-1 datasheet** — pinout, antenna keep-out dimensions, recommended decoupling
- **Espressif Hardware Design Guidelines (ESP32-S3)** — contains a **reference schematic** that
  covers Groups 1–3 exactly. Copy it rather than deriving it

### 3.3 Complete interconnect — every net, both ends

Generated 2026-08-19. **This is the schematic, in table form.** §3.1 lists which GPIO carries which
signal; this lists what is at the far end of each one, plus the sections that never touch the MCU.

Connector/part references used below: `CN1/CN2` motor+encoder, `J-PROX-L/R` prox, `J-TOF0..7` ToF,
`J-CONSOLE` UART0, `DRV1/DRV2` DRV8870, `U-AMP1/2` current-sense op-amp, `Q1` IR MOSFET,
`R-SENSE1/2` 200 mΩ.

#### DRV8870 ×2 — motor drivers

| Pin | DRV1 (Motor 1) | DRV2 (Motor 2) |
|---|---|---|
| **1** GND | board GND (node P) | node P |
| **2** IN2 | **GPIO21** `M1_PWM_IO21` | **GPIO41** `M2_PWM_IO41` |
| **3** IN1 | **GPIO48** `M1_DIR_IO48` | **GPIO42** `M2_DIR_IO42` |
| **4** VREF | 3V3 | 3V3 |
| **5** VM | 12 V + 100 nF + 10 µF **≥25 V** | same |
| **6** OUT1 | M1+ | M2+ |
| **7** ISEN | 200 mΩ → node P | same |
| **8** OUT2 | M1− | M2− |
| **PAD** | **board GND (node P)** — datasheet: *"connect to board ground"*, multi-layer pour + via array | same |

⚠ **DIR is on IN2, PWM is on IN1** — matches the drawn schematic. In IN/IN mode the two are
interchangeable (swapping them merely reverses the motor), so this is a labelling convention, not a
constraint. **Firmware must match this table**, not the reverse.

⚠ **Verified pinout, TI DRV8870 DDA (8-pin HSOP), 2026-08-26:**
`1=GND · 2=IN2 · 3=IN1 · 4=VREF · 5=VM · 6=OUT1 · 7=ISEN · 8=OUT2 · PAD=GND`.
An earlier note in this file questioned the EasyEDA symbol's numbering — **the symbol is correct**;
the query was wrong. Do not re-open.

**IN1/IN2 have internal pulldowns** (datasheet). The external 10 k pull-downs remain justified —
~10× stronger against active leakage onto the net, which is the actual ver3_1 wheel-spin mechanism.

▶ **DECIDED 2026-09-07: the four pull-downs live on THIS sheet, at the DRV8870 inputs** — moved from
the MCU sheet. Reasons, in order:

1. **Convention** — a hold-off resistor belongs at the input it defines. Anything between resistor
   and pin weakens that.
2. **Documentation** — the motor sheet now answers *"are these inputs defined through boot?"* in one
   place, which is the entire reason they exist.
3. **Marginal reliability** — an open trace between module and driver still leaves the input held low.

⚠ **Placement, not just sheet:** KiCad does not force placement from sheet assignment. Put them
**physically at the DRV8870 input pins**, on the logic side, away from R_ISEN / node P / the VM
decoupling. They are 0805 signal parts and must not compete with the power stage for space.

⚠ **A superseded argument — do not resurrect it.** An earlier revision justified the driver-end
placement as *"pulling the DevKit from its socket leaves the driver inputs floating."* **That died
with the socket** (§3.2): the module is soldered, so the resistor is a board component either way.
In normal operation both positions are electrically identical; the reasons above are what decide it.
| VREF | 3V3 | 3V3 |
| VM | **12 V** (BUCK#1) | 12 V |
| GND / PAD | GND plane + thermal vias | GND plane |
| OUT1 / OUT2 | CN1 pin 1 / pin 2 | CN2 pin 1 / pin 2 |
| ISEN | R-SENSE1 200 mΩ → GND, **and** → U-AMP1 IN+ | R-SENSE2 → GND, → U-AMP2 IN+ |

100 nF + 10 µF at each VM. `I_TRIP = VREF/(10·R_ISEN) = 1.65 A` per channel.

#### CN1 / CN2 — motor + encoder (6-pin DB128V-5.08)

| Pin | Signal | CN1 | CN2 |
|---|---|---|---|
| 1 | M+ | DRV1 OUT1 | DRV2 OUT1 |
| 2 | M− | DRV1 OUT2 | DRV2 OUT2 |
| 3 | +3V3 | encoder supply | encoder supply |
| 4 | GND | GND plane | GND plane |
| 5 | ENC_A | GPIO39 | GPIO47 |
| 6 | ENC_B | GPIO40 | GPIO5 |

#### BNO086 — SPI

> **▶ The full 16-pin breakout table lives in §3.1** and is not repeated here — that is the one
> place pin facts are recorded. §3.1 also carries the `PS0`/`WAK` shared-pad circuit (`R29`), the
> `PS1` separate-branch rule, and the two continuity checks on the physical module.

Summary for layout only: SPI on GPIO10–13, `INT` GPIO14, `RST` GPIO15, `WAK` GPIO9, `PS0` via
`R29` 10 k pull-up, `PS1` direct to 3V3, decoupling `C28` 100 nF + `C29` 10 µF at the 3V3 pin, and
a **ferrite bead** ahead of that decoupling (§11).

**Placement: away from the drivers, buck inductor and pack traces (§2.5).**

#### TCA9548A — mux and ToF fan-out

| Pin | To | Resistor |
|---|---|---|
| VCC / GND | 3V3 / GND (100 nF) | — |
| SDA / SCL | GPIO16 / GPIO17 | 2k2 PU each → 3V3 |
| RESET | GPIO18 | 10k PU → 3V3 |
| A0/A1/A2 | **GND** → `0x70` | — |
| SD0/SC0 | J-TOF0 — ToF front-LEFT | 4k7 PU each |
| SD1/SC1 | J-TOF1 — ToF front-CENTRE | 4k7 PU each |
| SD2/SC2 | J-TOF2 — ToF front-RIGHT | 4k7 PU each |
| SD3/SC3 | J-TOF3 — ToF rear-LEFT | 4k7 PU each |
| SD4/SC4 | J-TOF4 — ToF rear-RIGHT | 4k7 PU each |
| SD5/SC5 | J-TOF5 — ToF side-LEFT | 4k7 PU each |
| SD6/SC6 | J-TOF6 — ToF side-RIGHT | 4k7 PU each |
| SD7/SC7 | J-TOF7 — spare | 4k7 PU each |

**14 pull-ups for 7 active channels.** The mux does **not** pass upstream pull-ups through — every
downstream segment needs its own pair. All ToFs keep address `0x29`; that is the point of the mux.

#### J-TOF0..7 (4-pin JST)

`1 = 3V3 · 2 = GND · 3 = SDn · 4 = SCn`. XSHUT and GPIO1 left unconnected — the module's onboard
10k holds XSHUT enabled. 7 × ~20 mA ≈ **140 mA** on 3V3.

#### BATT_SENSE divider — GPIO4 (§5)

| Node | Value | Note |
|---|---|---|
| VCC → tap | **100 kΩ** 1 % | VCC is already on the board as the buck input |
| tap → GND | **20 kΩ** 1 % | ratio 0.1667 → **2.80 V** at a 16.8 V pack |
| tap → GND | 100 nF | filter |
| tap | BAT54S | pin1→GND, pin3→node, pin2→3V3 |
| tap → GPIO4 | — | ADC1_3 |

> **The INA226 is NOT fitted** (§5). `J-I2C` remains free if pack-current monitoring is ever
> revisited.

#### J-PROX-L / J-PROX-R (3-pin, keyed)

**AS BUILT: `CN3`, one 6-pin DB128V-5.08-6P-GN-S** carrying both sensors:
`1 = +12 V · 2 = GND · 3 = PROX_L OUT` · `4 = +12 V · 5 = GND · 6 = PROX_R OUT`.

Power pins are adjacent deliberately — a slip between neighbouring terminals lands **GND** on OUT
rather than +12 V on OUT. (An earlier draft specified two 3-pin connectors with `2 = OUT, 3 = GND`;
the single block is as drawn and is fine given the protection network below.)

Per channel, in this order:

- At the **SENSOR node** (the connector pin): **4k7 pull-up → 3V3** ⚠
- Then **10k series** → toward the MCU
- At the **GPIO node**: **BAT54S** (pin3→node, pin1→GND, pin2→3V3), **100 nF** to GND, then GPIO6/7

⚠ **Pull-up order is critical — see §6.** With the 4k7 at the GPIO node instead, an active sensor
reads 2.24 V and never registers, so `/dock/contact` never reaches 3.

#### IR charge-enable emitter (§7)

```
              +12V
                │
            [R_set]  2 × 50 Ω 0805 in series  (or 1 × 100 Ω 1206)
                │
                └────────────────► J-IR pin 1 ──► TSAL6400 ANODE   (off-board)
                                                        ▼
                     ┌─────────── J-IR pin 2 ◄── TSAL6400 CATHODE
                     │
                     │  Q1 DRAIN
   GPIO8 ──[R_g 100Ω]──┤ GATE     2N7002 / BSS138 (SOT-23)
                     │  SOURCE ──► GND
                  [R_pd 10k]
                     │
                    GND        ⚠ pull-down AT THE GATE
```

| Ref | Value | Note |
|---|---|---|
| `R_set` | **2 × 50 Ω 0805 in series** (or 1 × 100 Ω 1206) | ⚠ 129 mW average exceeds one 0805 — the one sanctioned exception to §12 |
| `Q1` | 2N7002 / BSS138 SOT-23 | AO3400A is a true logic-level part with far lower R_DS(on) at V_GS = 3.3 V if margin is wanted |
| `R_g` | 100 Ω 0805 | GPIO8 → gate |
| `R_pd` | 10 kΩ 0805 | **gate → GND, at the GATE** — holds the beacon off through boot even with the GPIO8 trace open |
| `J-IR` | 2-pin | ⚠ The TSAL6400 is **off-board** — it must aim at the dock's TSOP, and the board sits inside the chassis |

≈106 mA peak vs ~9 mA on ver3_1.

#### Spare pins and expansion connectors

▶ **Two spare GPIO** (§3.1) — `IO35`, `IO47`, both digital only. `IO5` went to `BOARD_TEMP`.

| Connector | Pins | Carries | Why |
|---|---|---|---|
| **J-EXP** (`J5`) | **4** | 1 = 3V3 · 2 = **`J-EXP_IO35`** · 3 = **`J-EXP_IO47`** (each via **330 Ω series**) · 4 = GND | ⚠ **Both are DIGITAL ONLY** — ADC1 is IO1–10, ADC2 is IO11–20, so neither can take an analogue input. Mark that on the silkscreen. IO5 (the ADC-capable spare) went to `BOARD_TEMP` on 2026-09-15 |
| **J-CONSOLE** | 3 | 1 = GND · 2 = **GPIO43** U0TXD · 3 = **GPIO44** U0RXD | Fallback serial console. micro-ROS runs over native USB, so UART0 stays free for debugging |
| **J-I2C** | 4 | 1 = 3V3 · 2 = GND · 3 = SDA · 4 = SCL | Bench header on the **upstream** bus. Doubles as the temporary-OLED port that replaced the deleted SSD1306 (§9) |
| **J-TOF7** | 4 | mux channel 7, unpopulated | Spare ToF channel |

⚠ **The real expansion path is the mux, not GPIO — and now it is the ONLY one.** Anything I²C —
more ToFs, an INA226, an environmental sensor — costs **zero pins** behind the TCA9548A or on the
upstream bus. GPIO has run out; plan every addition this way.

**The escape valve, if a dedicated pin is ever unavoidable:** move the **BNO086 from SPI to I²C**,
which frees GPIO9/10/11/12/13 — five pins — at the cost of sharing the bus with the mux (§3.1).

#### U-AMP1 / U-AMP2 — motor current sense (one dual op-amp)

**DECIDED 2026-08-25: TLV9002IDR, LCSC `C398360`.** Dual, SOIC-8, RRIO, 1.8–5.5 V, 1 MHz.
One package covers both channels. **100 nF decoupling at pin 8.**

| Alternative | LCSC # | V_os max | Error at gain 9.2 |
|---|---|---|---|
| **TLV9002IDR** ⭐ | `C398360` | **±0.4 mV** | ~2 mA |
| MCP6002-I/SN | `C116706` | ±4.5 mV | ~22 mA |
| LMV358IDR | `C63813` | ±9 mV | ~46 mA |

All three share the standard dual-op-amp SOIC-8 pinout — substitute on stock without a layout
change. Offset is the deciding spec: gain 9.2 multiplies it, and it sets the floor on how
confidently "this motor is drawing **no** current" can be asserted (the disconnected-motor case).

⚠⚠ **DO NOT USE LM358 / LM324 / TL072.** The LM358 is the reflex dual op-amp and a **JLC Basic
part**, which makes it the likely accidental substitution. Its output cannot swing near V+ —
typically `V+ − 1.5 V`, so **~1.8 V max on a 3.3 V rail** against a 3.04 V full scale. Its input
range does include ground, so it works at low current and **clips above ~1.0 A** — exactly where
stall detection lives. It would pass a gentle bench test and fail at the one moment it matters.
Rail-to-rail **output** is not optional here.

`IN+` ← DRV ISEN node · `IN−` ← Rg **1k 1%** to GND with Rf **8k2 1%** to OUT (gain ≈ **9.2**) ·
`OUT` → **1k** → GPIO1/GPIO2 with **1 µF** to GND · supply 3V3/GND + **100 nF**.

Full-scale: 1.65 A × 200 mΩ = 0.33 V × 9.2 = **3.04 V**, near ADC full scale. The 1k+1µF gives
fc ≈ 159 Hz — averages the 8 kHz PWM chopping. τ = 1 ms, ~3 ms to settle, far faster than any
stall timeout.

⚠ **KELVIN-CONNECT `Rg`'s GROUND TO ITS OWN SENSE RESISTOR.** This is low-side sensing: R_ISEN's
ground pad carries **full motor current**. For a non-inverting amp:

```
Vout = Vin+ × (1 + Rf/Rg)  −  V_Rg_gnd × (Rf/Rg)
```

The error path is therefore **where Rg's bottom end lands**, amplified by **8.2** — *not* where the
op-amp's V− pin sits. `1.65 A × 10 mΩ of shared plane = 16.5 mV` → **135 mV** at the ADC, against
a 3.04 V full scale. ~4 %, load-dependent, so it reads as excess gain rather than a fixable offset.

This is 4-wire **force/sense** measurement — the same idea as a DMM in milliohm mode or a supply
with remote sense leads. `Rg` draws only `3 V / 9.2 kΩ ≈ 330 µA`, so a dedicated trace from the
sense resistor's pad carries no meaningful IR drop.

**How to express it in EasyEDA** — a plain netlist cannot, since both ends are called `GND` and the
router will drop each to the nearest plane via. Name it instead:

1. Create a net **`AGND`**. Everything analogue returns to it: both `Rg`, both ADC filter caps, the
   op-amp decoupling cap, and the op-amp's **V− pin**.
2. Tie `AGND` to `GND` through a **0 Ω 1206 link (R48)** at **exactly one point** — node **P**
   below.

**Node P — where R48 lands.** The **ground side (bottom)** of both sense resistors, at the point
where the two merge, *before* that node runs to the main plane:

```
     DRV8870 #1                              DRV8870 #2
       ISEN ──┬──► M1_ISEN                     ISEN ──┬──► M2_ISEN
         [R_S1 200mΩ]                            [R_S2 200mΩ]
              │                                       │
   DRV#1 GND ─┤                                       ├─ DRV#2 GND
              └───────────────┬───────────────────────┘
                              ●  P   ← local star node
                  ┌───────────┴────────────┐
             [R48  0 Ω]           wide pour + via array
                  │                        │
                AGND                 main GND plane (3.3 A)
```

- **Top of each sense resistor = the signal.** **Bottom = the motor return**, up to 1.65 A. The
  bottom can never be `AGND` — `AGND` must carry no current, that is the entire premise.
- **Both channels are served by one tie.** `V(S1) ≈ V(S2) ≈ V(P)`; both motor currents then flow
  P → plane *together*, and that drop is common to signal and `Rg` alike, so it cancels. This is
  why the two sense resistors must be placed **adjacent** — spread them and P stops being one node.
- ⚠ **Both DRV8870 `GND` pins belong at P.** The chip's internal current-limit comparator measures
  ISEN against `VREF/10` **referenced to its own GND pin**. Put that pin elsewhere on the plane and
  the 1.65 A trip threshold shifts by the intervening drop.

**Where to DRAW it:** on the **motor driver sheet**, physically between R1 and R2 — one end to the
`GND` node their bottoms sit on, other end to an `AGND` symbol. Add a text note beside it, because
the netlist cannot carry placement:

> `R48 — PLACE AT R1/R2 GROUND PADS. Single AGND↔GND tie. Do not relocate.`

**Part: UNI-ROYAL 1206W4F0000T5E, LCSC `C17888`** (0 Ω, 1206, 250 mW). Alt `C19290` (±5% grade —
tolerance is meaningless on a 0 Ω). Jumper current ~2 A against an `AGND` load of **~800 µA**
(2 × Rg at ~330 µA + TLV9002 quiescent ~120 µA) — ~2500× margin. **R48 does NOT need a
high-current part**; it carries almost nothing, which is the entire point.

⚠ **Do not substitute a ferrite bead.** Common reflex for AGND/DGND bridges, but it inserts
impedance in a return path and current practice is against it. This section is low-frequency; a
0 Ω is predictable.

⚠ **R48 is never DNP.** It is the only path to ground for the entire analogue section — omit it and
`AGND` floats and both channels are dead.

**Why a 0 Ω part and not a plain wire:** drawn as a direct connection, EasyEDA merges `AGND` into
`GND` and the router may bond them anywhere — including under the motor return path. You would have
the name without the behaviour. R48 keeps them distinct in DRC so the single crossing is where you
put it. (A net-tie/bridge primitive works identically; a 0 Ω 1206 suits the hand-build rule and is
visible on the board.)

| Approach | Error at 1.65 A | % of full scale |
|---|---|---|
| `Rg` on a random plane via, 10 mΩ away | 135 mV | **4.4 %** |
| **Single `AGND` tie at the sense resistors** ⭐ | ~25 mV | ~0.8 % |
| Separate `GND_S1`/`GND_S2` per channel | ~5 mV | ~0.2 % |

**Single `AGND` is the recommended level.** A fixed 0.8 % gain error calibrates out against a clamp
meter. What does *not* calibrate out is **cross-talk** — motor 2's current appearing in motor 1's
reading — because it varies with what the other motor is doing. Per-channel separation only buys
the last 0.6 %.

**The rule underneath all of it: place U9 and its four gain resistors immediately beside the two
DRV8870 sense resistors.** Short beats clever.

▶ **AS BUILT: 1 kΩ + 1 µF at each `IN+`** (`fc ≈ 159 Hz`) — two cascaded 159 Hz poles with the
output filter. Response ~8 ms total, against stall timeouts in the hundreds of ms; at 0.3 m/s the
robot moves 2.4 mm in that window. More PWM rejection than the 100 nF originally specified, the
op-amp then never slews at all, and **all four filter caps are one value** — worth something on a
hand-built board. No loading concern: ~330 µA peak through the series resistor perturbs the ISEN
node by 0.02 %. The original rationale (`fc ≈ 1.6 kHz`). The ISEN voltage is
chopped at 8 kHz; at **low duty the ON pulse shortens**, and TLV9002's 2 V/µs slew plus settling
means the amp cannot fully settle within a ~1–2 % duty pulse — it **under-reads**. Low duty is
exactly what `dock_aligner_v3`'s slow position segments run at, which is when stall detection most
needs to be honest. The input RC averages before the gain stage and removes the slew dependency.
Two parts per channel.

⚠ **Calibrate empirically, do not trust the gain arithmetic.** What R_ISEN actually sees depends
on the decay mode: in **brake/slow decay** (which `dock_aligner_v3` uses) recirculating current
flows through the low-side FETs and therefore through R_ISEN; in **coast/fast decay** it returns
through the high-side body diodes and bypasses it entirely. The filtered mean is therefore not the
motor's mean current by a fixed factor. Calibrate against a clamp meter across the working range.

⚠ **Ripple is handled.** The op-amps sit on buck #3's switched 3.3 V, unlike `BATT_SENSE` (which
runs off the DevKit's own LDO — §11). Supply ripple passes at PSRR, but the 159 Hz post-filter
gives ~70 dB at 500 kHz. No extra filtering needed.

---

---

## 4. Motor drive — 2 channels

Keep the ver3_1 topology, it works. Per channel:

- **DRV8870DDAR**, IN/IN mode (`INx` = DIR, `INy` = PWM)
- **200 mΩ** ISEN current-sense resistor
- **100 nF + 10 µF / 35 V** decoupling at VM (⚠ 35 V — see §11 regen clamp)
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

That turns `MOVE_STALL_MS` from a timing heuristic into a measurement.

#### ▶ Why this is worth the board area — ranked honestly

> ⚠ **Corrected 2026-08-26.** An earlier version of this section justified current sensing on
> *"the low-obstacle layer is ONE 25° ToF cone"* (`CLAUDE.md`). That describes **today's robot** —
> mux dead, one sensor fitted. **The new board provisions 7 ToFs** (§3.3: front-L/C/R, rear-L/R,
> side-L, side-R). The collision-detection case therefore belongs mostly to the ToF ring, not here.
> What follows is what survives that correction.

**1. Floor interaction — the strongest remaining case.** The 7 ToFs sit at **80 mm** and watch the
horizon; the cone is only ~44 mm tall at 100 mm range, so **nothing sees the floor under the
robot.** Wheel climbing a door threshold (the reason for the 100 mm AGV wheels), a rug edge, the
rear caster jammed on a cable — all invisible to every ToF, all visible as current. Torque during a
threshold climb is the difference between "made it" and "stuck, wheels turning."

**2. Disconnected motor vs jammed motor.** Indistinguishable today — no encoder counts either way.
With current they are opposite extremes. No amount of ToF changes this.

**3. The DRV8870 has NO fault pin.** Its 1.65 A current limit is internal and completely silent —
it chops and reports nothing. Without a sense amp there is **zero visibility** into the motors'
electrical state, ever.

**4. Independent modality — defence in depth.** The 7 ToFs hang off cabled JSTs behind the
TCA9548A, and that mux has already failed once. A ToF that loses its connector is a **silent**
blind spot. Current shares neither the bus nor the cables.

**5. Residual coverage gaps.** 7 × 25° FoV = **175° of 360°** even with ideal aiming, with the
holes on the front- and rear-quarter diagonals. Current is a backstop there, not the primary.

#### ⚠ What it does NOT do — do not justify it on these

- **It does not improve docking precision.** `dock_aligner_v3` is solved on encoder position
  control; current is not in that loop. It improves failure *diagnosis*, not success rate.
- **It is NOT the primary obstacle sensor.** That is the 7-ToF ring (§3.3, §9). Current sensing is
  a backstop for the diagonals and for floor interaction the ring cannot see.
- **It does not fix the 2026-08-14 stall on its own.** Re-read those numbers: the segment
  travelled **0.284 m against a commanded 0.250 m** — it **overshot** and was still flagged
  `STALLED`. That is a bug in the stall heuristic, fixable in firmware for free. It is the reason
  the blindness was *noticed*, not proof a sensor was the only cure.

#### Cost, and the trim if area is tight

~14 parts (SOIC-8 + 8 R + 5 C + the 0 Ω), ≈230 mm² in 1206 — about **5 % of a 100×100 board**.
The real area consumers elsewhere are the three buck inductors, the DevKit socket (~25×55 mm) and
the connector bank.

- **Trim first:** drop the optional input RC (R39/R40 + their caps). Four fewer parts, circuit
  unaffected. Revisit only if under-reading is observed at creep duty.
- **If still tight: footprint it and mark DNP.** Zero assembly cost, simpler bring-up, pads waiting.
  The footprint cannot be added later; the parts always can.
- **Do not cut it entirely** — it is the only proprioceptive sense the robot has.

---

## 5. Battery monitoring — DECIDED: on-board divider, no INA226

**DECISION 2026-08-22 — this REVERSES the 2026-08-19 INA226 decision.** The divider stays. It moves
**onto** the motion board and taps VCC locally, on **GPIO4** (ADC1_3).

```
   VCC ──[100 k]──┬──► GPIO4 (ADC1_3)
                  ├── 100 nF → GND
                  ├── BAT54S: pin1→GND, pin3→node, pin2→3V3
              [20 k]
                  │
                 GND
```

**Values: 100 k / 20 k, 1 %.** Ratio 0.1667 → 2.80 V at a 16.8 V pack, within 1 % of the existing
firmware constant `BATTERY_V_DIV = 0.16510`, so recalibration starts close (§10.2).

#### Why the reversal

The INA226 was the better *instrument*. It lost on **installation cost**, which is the axis this
whole respin is being judged on:

| INA226 | On-board divider |
|---|---|
| Another off-board module to mount | Two resistors and a cap, already on the board |
| A **5 mΩ shunt spliced into the 40 A pack main line** | Nothing touches the pack line |
| 2–3 sense wires from the pack distribution point | **Zero** new chassis wires |
| New I²C device, new driver code | `esp_adc_cal` path already exists and works |

VCC is already present on the MCB as the buck input, so the divider taps it locally rather than
arriving on a wire from the power board. That deletes R1/R2 from the power perf board **and**
removes a chassis wire — the stated goal of the respin.

#### ⚠ What the reversal COSTS — read this, it is not free

The INA226 was not adopted for voltage. It was adopted for **current**, and that capability is now
**gone**. Specifically unanswered again:

| Lost | Consequence |
|---|---|
| True charge/discharge current **and direction** | `power_supply_status` stays hardcoded — the firmware genuinely cannot tell charging from discharging |
| Coulomb counting | State-of-charge remains a voltage guess |
| **Net-discharge detection while docked** | ⚠ **The robot has drained flat on the dock twice.** Charge-in (~75 W) was less than full-bringup draw, so it net-discharged while apparently charging. **Only total pack current answers this, and nothing now measures it** |
| Data to size the docked low-power profile | [[project_operational_modes]] stays unquantified |

**This goes back to the parking lot, it is not solved.** If the flat-on-dock problem recurs, the
INA226 (or any pack-line current sense) is the answer and the shunt belongs in the **pack main
line**, not on the motion board — a motion-board shunt would miss Thor's 40–60 W entirely, which
is the whole point. Size any such shunt for **~8–10 A**: Thor (~3.5 A at 16.8 V) + motors (3.3 A
ISEN-limited) + the rest.

`J-I2C` remains free for exactly this if it is ever revisited. It costs zero pins (§3.3).

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
                    +3.3 V
                      │
                   [4.7 k]        ⚠ SENSOR side, NOT the GPIO side
                      │
  SENSOR OUT ─────────┴──── 10 k ────┬──── GPIO6 / GPIO7
                                     │
                                    BAT54S  →  3V3 / GND
                                     │
                                    ─┴─  100 nF
                                    GND
```

⚠⚠ **CORRECTED 2026-08-25. The 4.7 k pull-up MUST sit on the SENSOR side of the 10 k series
resistor.** An earlier draft of this section, and of §3.3, put it "on the GPIO side" — that
configuration **does not work**:

| 4.7 k position | Sensor inactive | Sensor ACTIVE (sinks to 0 V) |
|---|---|---|
| **At the GPIO node** ❌ | 3.3 V — reads HIGH ✓ | `3.3 × 10/(4.7+10)` = **2.24 V** — above V_IL (0.83 V), below V_IH (2.48 V). **Indeterminate, reads HIGH.** The sensor never registers |
| **At the sensor node** ✅ | 3.3 V — reads HIGH ✓ | 0 V — reads LOW ✓. No current flows in the 10 k into a high-Z input |

**Consequence if built wrong: `/dock/contact` never reaches 3, and docking cannot complete.**
That is the single success criterion for the whole docking stack.

Protection is unaffected by the move: if the sensor output ever presents 12 V, the 10 k limits
current into the BAT54S to `(12 − 3.3)/10k ≈ 0.87 mA`.

**Pull up to 3.3 V — never to 12 V.** An NPN open-collector output only *sinks*; the pull-up rail
is decoupled from the sensor's supply and is entirely the designer's choice. It must be 3.3 V so the
HIGH level is what the GPIO tolerates. Pulled to 12 V, the pin would sit clamped by the BAT54S as a
*normal operating state* — which is not a design. Load is `3.3 V / 4.7 kΩ = 0.70 mA` against the
sensor's 300 mA sink rating: 0.2 %.

**Empirically confirmed on the working robot.** Today there is no external pull-up at all — the
firmware uses `INPUT_PULLUP`, the ESP32's internal **~45 kΩ** to 3.3 V — and docking still reaches
`contact=3` and charges. So the LJ18A3 already pulls a valid LOW through 45 kΩ. **4.7 kΩ is 10×
stiffer**, so residual voltage, leakage immunity and edge rate all improve. No measurement needed.

**Why fit it anyway, given the internal one works:** `INPUT_PULLUP` does not exist until firmware
runs `pinMode()`. Through the whole boot window the pin floats and dock contact state is undefined.
The external resistor defines it from the instant power is applied, and survives any firmware path
that sets the pin to plain `INPUT`.

**Supply: the 12 V rail, not VCC.** Both are inside the 6–36 V spec (a flat pack passing through at
~12 V still clears the 6 V floor), but on VCC the sensor supply swings 16.8 → 12 V across a
discharge and its leakage and residual voltage drift with it. ~10–15 mA each is invisible against
buck #1's 3.3 A budget.

Better still if there's room: an **optocoupler** (e.g. LTV-357T) per channel. Full galvanic
separation between the 12 V sensor loop and the logic, and it removes any question about ground
offsets between the sensor supply and the ESP32.

**Connector: AS BUILT one 6-pin `CN3`** (DB128V-5.08-6P-GN-S) for both sensors — see §3.3 for the
pinout. An earlier draft asked for two keyed 3-pin blocks so one sensor could be serviced without
disturbing the other; the single block trades that away. Acceptable **because** the protection
network above makes a miswire survivable rather than fatal — on a screw terminal that a person
rewires in the field, those six parts stop being optional.

⚠ **The 1 ms RC (10k + 100 nF) is not only protection.** The sensor cables are **103 cm**, unshielded,
running alongside motor wiring — a serviceable antenna for 8 kHz PWM. Firmware debounces over
5 × 20 ms cycles, so 1 ms is invisible to it and kills the pickup.

---

## 7. Dock IR emitter — currently under-driven

**Part:** TSAL6400 + 220 Ω, **GPIO8** on the new board (ver3_1 used GPIO4, which is now
BATT_SENSE — §5). 38 kHz carrier via LEDC ch 4, burst-gated
(600 µs on/off × 10, then 40 ms gap ≈ 19 packets/s) to keep the dock's TSOP AGC happy.
Fires only while seated **and** below `BATTERY_FULL_STOP` = 16.70 V.

⚠ **Driven directly from a GPIO through 220 Ω from 3.3 V.** That gives roughly
(3.3 − 1.35) / 220 ≈ **9 mA**. The TSAL6400 is rated to 100 mA continuous and its whole value is
optical range. We are running it at under a tenth of what it can do, and the ESP32's 40 mA
per-pin limit means it can never do better on this topology.

**Specify a low-side driver:**

```
  +12 V ─── R(set) ──── TSAL6400 ──── MOSFET drain
             100 Ω                     (2N7002 / BSS138)
                    GPIO8 ──── 100 Ω ──── gate
                                       source ──── GND
                                       10 k gate pull-down
```

⚠ **Fed from 12 V, not 5 V — updated 2026-08-26.** The 5 V rail is deleted (§11); this was its last
real load. `R(set)` = **100 Ω** gives `(12 − 1.35)/100 ≈ 106 mA`, well within the LED's pulsed
rating at this duty and roughly **12× the current optical output** of the old 220 Ω-from-a-GPIO
topology. That is margin for a dirtier dock face or a wider approach cone.

⚠ **`R(set)` must not be a single 0805.** Dropping 10.65 V at 106 mA is 1.13 W peak; at the ~11.5 %
burst duty (600 µs on/off × 10, then a 40 ms gap) that is **~129 mW average**, above an 0805's
125 mW rating. Use **2 × 0805 in series (2 × 50 Ω)** — ~65 mW each — or one 1206. This is the one
sanctioned departure from the 0805 house rule (§12).

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

### ToF part — DECIDED: stay with VL53L0X

**DECISION 2026-08-19: use the 8 × VL53L0X already in hand.** VL53L5CX multizone was raised and is
**not warranted** — the measured blind wedge is fixed by mount geometry, at no cost, in a reprint
that has to happen anyway.

#### The geometry fix

An object of height `h` is invisible closer than `(H − h) / tan(12.5° − tilt)`:

| Mount | 30 mm object | 50 mm object | Floor enters cone |
|---|---|---|---|
| **85 mm / 8°** (as built) | blind < **0.70 m** | blind < 0.44 m | 1.08 m |
| **50 mm / 8°** ◀ **adopt** | blind < **0.25 m** | **always visible** | **0.64 m** |
| 40 mm / 8° | blind < 0.13 m | always visible | 0.51 m |

**Adopt 50 mm / 8°, `max_range` ≈ 0.60 m.** That gives a usable **0.25–0.60 m reflex band** which
sees anything 30 mm or taller, with the floor never entering the cone inside it. The current
85 mm mount is blind to a shoe-box-height object inside 0.70 m — most of the zone the layer exists
to cover — which is what was measured on 2026-08-14.

This costs nothing: the mounts must be reprinted regardless for the crosstalk fix (black matte,
aperture ≥6–7 mm, module flush or proud — see the mechanical warning below).

#### What the geometry fix does NOT solve

The ~25° cone is narrow, so a single sensor still misses a chair leg well off its axis. **Coverage
comes from multiplication, not per-sensor FoV** — that is the point of the 7-sensor ring
(3 front / 2 rear / 2 side) behind the mux, which costs zero GPIO.

#### Contingency — revisit VL53L5CX only if

- the ring proves genuinely insufficient after the geometry fix, **and**
- inter-sensor coverage gaps are the demonstrated cause

Its 4×4/8×8 zones and ~63° FoV would resolve floor and obstacle simultaneously, removing the tilt
trade entirely. But it costs more, needs far more I²C bandwidth and host processing, and adds an
unproven component to a subsystem that has already cost a full day.

⚠ **Prove what you have first.** Only **one** VL53L0X has ever worked on this robot, and the mux
never enumerated at all. The 7-sensor ring is entirely unproven. Get the on-PCB mux and two or
three VL53L0X working before buying any alternative.

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

### 10.2 Battery divider resistor values — ⚠ UN-RETIRED, then SETTLED 2026-08-22

**This section was marked RETIRED on 2026-08-19 when the INA226 was adopted. That decision was
reversed on 2026-08-22 (§5), so the question is live again — and now answered.**

**DECIDED: 100 kΩ / 20 kΩ, 1 %, on GPIO4.** Ratio 0.1667 → **2.80 V** at a 16.8 V pack.

The old firmware constant `BATTERY_V_DIV = 0.16510` was hand-fitted against a multimeter to absorb
both resistor tolerance and the ESP32 ADC's non-linearity. Choosing 100 k/20 k means the nominal
ratio (0.1667) starts within 1 % of that fitted value, so recalibration begins close rather than
from scratch. `CLAUDE.md` records 100 k/**22 k** as fitted on the current hardware — the new board
standardises on **20 k**; expect a small recalibration and update the constant.

Drop to 50 k/10 k only if ADC readings prove noisy: source impedance halves, drain rises
140 µA → 280 µA, both trivial.

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

### 10.4 ⚠ WROOM-32 → ESP32-S3 firmware port — the pins that MOVED

**There is no GPIO22, 23, 24 or 25 on the ESP32-S3.** The map runs 0–21, then jumps to 26. Any
WROOM-32 pin number above 21 is invalid, and several *valid* numbers now mean something different.

| Firmware today | New board | Consequence if ported unchanged |
|---|---|---|
| `firmware.ino:952` `Wire.begin(21, 22)` | **`Wire.begin(16, 17)`** | ⚠⚠ **GPIO21 is now `M1_PWM`** (DRV8870 #1 IN2). Unchanged, this drives a motor driver input as SDA at bus rate. GPIO22 does not exist, so SCL is undefined |
| `i2c_scan` `SDA_PIN=21, SCL_PIN=22` | **16 / 17** | same |
| `IR_EMIT_PIN 4` | **8** | GPIO4 is now `BATT_SENSE` (§5). Unchanged, the IR carrier modulates the battery ADC node |
| `PROX_LEFT_PIN 13` | **6** | GPIO13 is now `IMU_MISO` |
| `PROX_RIGHT_PIN 15` | **7** | GPIO15 is now `IMU_RST` |
| battery ADC `GPIO34` | **4** | GPIO34 is not brought out on the DevKitC-1 |

⚠ **The 10 k pull-downs do not protect against this.** They hold a *floating* pin; an actively
driven pin overrides them. The wheel-spin defence in §8 is for the boot window, not for a firmware
pin-number mistake.

**Do the pin-number sweep as one deliberate pass against §3.1 — not opportunistically.** Every
`#define` in `jupiter_config.h` and every literal pin number in `.ino`/`.cpp` gets checked against
the §3.1 table. §3.1 is the only authority.

---

## 11. Power supply — bring the external buck onboard

**The change:** the board is currently fed 12 V by an **external 16.8 V → 12 V buck** sitting
elsewhere on the robot. The new board takes the **raw 4S pack** and generates all three rails
itself. One input, one fuse, one reverse-protection stage, one less box to mount and wire.

### Target architecture — TWO RAILS, 2026-08-26

```
  PACK 12–16.8 V ──► F1 S1206-FA-8.0A ──► DBT50 terminal ──► VCC
                                                              │
   ┌──────────────────────────────────────────────────────────┤
   │                                                          │
   │  BUCK #1   U5  TPS54560DDAR  ──► 12.02 V ──┬──► DRV8870 VM ×2
   ├──                                          ├──► prox sensors ×2 (12 V)
   │                                            └──► IR emitter driver (§7)
   │
   │  BUCK #2   U7  MP2338        ──►  3.30 V ──┬──► ESP32-S3-WROOM-1U
   └──                                          ├──► BNO086 (via ferrite U8)
                                                ├──► TCA9548A + 7 × ToF
                                                ├──► TLV9002 (via AGND, §3.3)
                                                └──► VREF
```

#### ▶ The 5 V rail is DELETED

**DECISION 2026-08-26.** The old buck #2 (5 V) had exactly three loads, and all three evaporated:

| Load | Fate |
|---|---|
| ESP32 DevKit via `D5` | **Gone.** The bare module runs on 3.3 V — no 5 V input exists |
| IR emitter driver (§7) | **Moved to 12 V** — see below |
| `J3` | **Phantom.** Listed twice as a 5 V load and never defined anywhere. A ver3_1 leftover |

Deleting it recovers an IC, an inductor, ~6 caps and ~6 resistors — **~14 parts and ~15 × 20 mm**
on a board where area is the binding constraint.

**`D5`, the SS34 ORing Schottky, is deleted with it.** So is the whole §4 USB-VBUS-versus-pack
contention analysis *as it applied to a socketed DevKit* — with your own USB-C receptacle you can
simply not connect VBUS, which §4 already lists as the zero-risk option.

**The IR emitter moves to 12 V.** §7 sets ~110 mA through the TSAL6400 with 33 Ω from 5 V; from
12 V use **100 Ω** for ~106 mA. ⚠ Dissipation in that resistor rises from ~46 mW to **~129 mW**
average at the ~11.5 % burst duty, which exceeds an 0805's 125 mW rating — use **2 × 0805 in series
(2 × 50 Ω)** or a single 1206 for that one part.

**No 5 V AUX LDO — not even as a DNP footprint.** DECIDED 2026-08-26. A DNP footprint was
considered and rejected: **unpopulated pads still consume board area**, and area is the binding
constraint on a 100×100. Reserving ~50 mm² for a capability with no identified user is a worse
trade than the "cannot add a footprint after fab" argument that favours it.

**5 V already exists on the robot, off this board.** A separate high-current buck powers the
**Raspberry Pi 5** (which hosts the ReSpeaker — see `project_pi5_audio_migration`). So the escape
hatch is not hypothetical: any future 5 V load taps that supply, or gets its own module on a 4-pin
header. There is no reason for this board to generate a rail the chassis already has. Note
that an on-board LDO would only ever have delivered ~150 mA anyway — `(12−5) × 0.15 = 1.05 W` in a
SOT-223 — so it would not have covered a servo, a fan or a WS2812 ring regardless.

### Rail loads

| Rail | Source | Feeds | Current |
|---|---|---|---|
| **12.02 V** | U5 TPS54560, from VCC | 2 × DRV8870 VM, prox ×2, IR driver | **≤ 3.3 A** motors + ~0.25 A |
| **3.30 V** | U7 MP2338, from VCC | ESP32 module, BNO086, mux + 7 ToF, TLV9002, VREF | **~300–350 mA** |
| **VCC** | pack, via F1 | both bucks | ~2.7 A at 16.8 V, ~3.8 A at a flat pack |

#### 3.3 V budget — recomputed for the module, WiFi off

| | |
|---|---|
| ESP32-S3-WROOM-1U (**radio never enabled**) | ~150 mA |
| TCA9548A + 7 × VL53L0X | ~140 mA |
| BNO086 | ~15 mA |
| TLV9002, VREF | negligible |
| **Total** | **~300–350 mA** |

▶ **The noise concern that moving the MCU onto 3.3 V would have created does not arise.** A
WiFi-enabled module pulses ~500 mA on TX, which would have sat alongside the TLV9002 and the ToF
array. With the radio off it is a steady ~150 mA. An MP2338 at 3 A is comfortably oversized.

#### The IMU supply filter — `U8` + `C28`/`C29`

The BNO086 shares the 3.3 V rail with a switcher and with the MCU. A **ferrite bead in series with
its supply**, `C28`/`C29` on the IMU side, makes a pi filter for the one part on that rail whose
analogue front end can rectify HF trash.

| | |
|---|---|
| **LCSC** | **`C21519`** — TDK **MPZ2012S601AT000** |
| Package | 0805 |
| Impedance | 600 Ω @ 100 MHz |
| DCR | **100 mΩ** → ~1.5 mV at the IMU's ~15 mA |
| Alternates | Sunlord `C1017` (0805, 300 mΩ) · `C1002` (0603, 450 mΩ) |

⚠ **Know what a bead does and does not do.** Ferrites are near-resistive only above ~10 MHz and
close to *nothing* at the MP2338's 450 kHz fundamental. `U8` attenuates **switching edges and EMI**,
which is what actually couples into an IMU. **`C29` handles the fundamental ripple, not the bead.**

If measurement ever shows the fundamental is the problem, the upgrade is a small **series inductor**
(≈10 µH) in place of the bead — with `C29` that is a real LC at `fc ≈ 16 kHz`, >30 dB at 450 kHz.
Do not fit that speculatively.

⚠ **Placement has two ways to go wrong — see §3.1.** The bead must sit between the rail and the
IMU's *whole* local domain with the caps downstream; and **breakout pin 15 must be NC**, or supply
current routes in through it and straight past the filter.

### Buck #2 — 3.3 V, MP2338 (replaces MP1584)

**Follow the datasheet Figure 8 circuit verbatim.** V_IN 6.5–28 V covers the pack; 3 A against a
~350 mA load.

| | MP1584 (was) | **MP2338 (now)** |
|---|---|---|
| Rectification | non-synchronous — needed an SS34 catch diode | **synchronous** — ⊖ 1 diode, better efficiency |
| Control | voltage mode + COMP network | **COT** — ⊖ the 100 k/1 nF COMP pair |
| Frequency | RT resistor | **450 kHz fixed** — ⊖ the 250 k RT resistor |
| Package | SOIC-8-EP | SOT583 |
| **V_ref** | 0.8 V | ⚠ **0.5 V** |

⚠⚠ **V_ref is 0.5 V, NOT 0.8 V.** Back-calculated from all three datasheet application circuits:
`3.3/(1+51/9.09) = 0.499` · `5.0/(1+90.9/10) = 0.496` · `12/(1+255/11) = 0.496`.
**The MP1584-era dividers do not carry over.** Keeping 118 k/22 k on a 0.5 V reference would give
**3.18 V**, not 5.09 V.

**As-built values (datasheet Figure 8):**

| Ref | Value | Note |
|---|---|---|
| R1 (FB top) | **51 kΩ** | `0.5 × (1 + 51/9.09) = 3.30 V` |
| R2 (FB bottom) | **9.09 kΩ** | |
| R6 | 10 kΩ | series into FB — ripple injection, COT needs it |
| C5 | 470 pF | across R1. Note 9: optional, improves transient. Fit it |
| **L1** | **6.8 µH** | ⚠ **not 22 µH** — the MP1584-era inductors do not carry over |
| C1A/C1B/C1C | 10 µF + 10 µF + 0.1 µF | input |
| C2A/C2B | 22 µF + 22 µF | output |
| C3 | 0.1 µF | BST |
| C4 | 22 nF | **SS — soft-start.** New part, MP1584 had none |
| R3/R4 | 191 k / 49.9 k | **EN divider = programmable UVLO.** Take it — a defined input-undervoltage lockout is genuinely useful on a battery robot |
| R5 | 100 kΩ | PG pull-up |

⚠ **Do not second-guess L = 6.8 µH.** COT control derives its feedback from output ripple; an
oversized inductor starves the loop. That is also what `R6`/`C5` are for.

⚠ **Buck #1 stays TPS54560.** Datasheet Figure 10 requires V_IN ≥ 17 V for a 12 V output, and
MP2338 is 3 A against a 3.3 A motor budget. Do not swap it.

### Sizing buck #1 — bounded by the existing ISEN design

The DRV8870s are already current-limited by the 200 mΩ sense resistors and the 3.3 V VREF:

```
I_TRIP = V_REF / (10 × R_ISEN) = 3.3 V / (10 × 0.2 Ω) = 1.65 A per channel
```

Two channels → **3.3 A absolute worst case, including stall**, plus ~0.2 A for the prox sensors.
Buck #1 no longer carries the 5 V or 3.3 V stages — both come off VCC directly — so its whole
budget is motors. A **5 A** part gives comfortable margin. *Confirm the factor of 10 against the DRV8870
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
| Topology | Non-synchronous is **fine here** — see the correction below |
| Duty | **High / 100 % pass-through capable** — the requirement most parts fail |

**AS BUILT: U5 = TPS54560DDAR** (5 A, 60 V, non-synchronous), with `D1` **B560C-13-F** catch diode,
`L1` 6.8 µH, 4 × 47 µF output, `R3`/`R4` 442 k/90.9 k EN-UVLO divider, `R8` 243 k RT, `C14` 100 nF
bootstrap, and `R12`/`C19`/`C20` (16.9 k / 4.7 nF / 47 pF) COMP compensation.

⚠ **Correcting an earlier claim.** This section previously demanded a *synchronous* part because
*"at 71–100 % duty a diode rectifier wastes real power."* **That reasoning is backwards.** In a
buck the catch diode conducts for `(1 − D)`, so high duty means the diode conducts *less*:

| Pack | Duty `D = 12/Vin` | Diode conducts | Diode loss @ 3.3 A |
|---|---|---|---|
| 16.8 V | 71 % | 29 % | ~0.67 W |
| 12.5 V | 96 % | 4 % | ~0.09 W |

High duty is exactly the condition under which non-synchronous is *most* efficient. The TPS54560 +
B560C is a sound choice, not a compromise. Give D1 copper for the ~0.7 W worst case.

⚠ **Still to verify: true 100 % duty.** The TPS54560 refreshes its bootstrap capacitor via a
minimum off-time, so it approaches but does not reach 100 % duty. At a flat 12.0 V pack expect the
rail to sit slightly *below* 12 V (inductor DCR + diode + min off-time) rather than passing through
cleanly. Harmless for 12 V-rated motors, but confirm the number rather than assuming 12.0 V.

### ~~Buck #2 — 5 V (U6, MP1584)~~ — DELETED 2026-08-26

**The 5 V rail no longer exists.** See "The 5 V rail is DELETED" above. `U6`, `L2`, `D2` (SS34
catch), `C9`/`C10`/`C21`/`C22`/`C23`, `R6`/`R13`/`R14`/`R15`/`R16` are all removed, as is `D5`, the
SS34 ORing Schottky at the DevKit's 5V pin.

### ~~Buck #3 — 3.3 V (U7, MP1584)~~ — SUPERSEDED by MP2338

Replaced by the MP2338 circuit above. The MP1584-era values (`R17` 75 k / `R19` 24 k on a 0.8 V
reference, `L3` 22 µH, `D3` SS34, `R10`/`C25` COMP, `R18` 250 k RT) **do not carry over** — the
reference is 0.5 V and the topology is COT. Retained here only so the old values are not
resurrected from an older revision.

⚠ The old minimum-on-time concern is **resolved**: at MP2338's fixed 450 kHz, 16.8 V → 3.3 V gives
an on-time of `(3.3/16.8)/450 kHz ≈ 436 ns`, comfortably clear of any minimum. No pulse-skipping.

### ⚠ Regenerative kickback — design this in

A buck **cannot sink current**. When the motors decelerate or reverse, energy returns up the rail
and pumps the 12 V node. Previously that energy went back toward the pack through the external
buck's path; with the regulator onboard and the motors behind it, it has nowhere to go.

**Rough energy check.** ~12 kg at 0.3 m/s ≈ 0.54 J. Brake mode dumps most of it in the windings,
but if only 0.1 J reaches the rail and lands in 470 µF: `0.1 = ½ × 470µF × (V² − 144)` → **~23.9 V**
on a 12 V rail from a single hard stop. Bulk capacitance alone does not contain it.

Add on the 12 V rail, **near the drivers** (motor-driver sheet, not the PSU sheet):

| Part | Spec | LCSC |
|---|---|---|
| **TVS clamp** | **SMBJ15A**, unidirectional, SMB (DO-214AA). Standoff 15 V, breakdown 16.7–18.5 V, clamp **24.4 V** @ 24.6 A, 600 W peak. **Cathode (banded) → +12 V, anode → GND** | **`C699013`** (YANGJIE) or `C1979382` (Vishay) |
| **Bulk electrolytic** | **470 µF**, **35 V** (not 220 µF — see energy check below) | confirm can size when ordering |

**DRAWN 2026-09-28** on `motor_controller.kicad_sch` as **`D8`** (TVS) and **`C39`** (bulk), both on
`+12V` / `GND`, verified by connectivity trace. Footprints `Diode_SMD:D_SMB_Handsoldering` and
`Capacitor_SMD:CP_Elec_10x10.5`.

⚠ **Not a Schottky.** An SS34 (or any rectifier) is reverse-biased at 12 V and does nothing until
its 40 V breakdown, where it is not avalanche-rated and simply fails. Different function entirely
from D5 on the 5 V rail.

⚠ **Get the `A` suffix.** `SMBJ15CA` is **bi**directional and wrong for a DC rail.

⚠ **Schematic symbol: use `Device:D_Zener`, NOT `Device:D_TVS`.** *Every* `D_TVS*` symbol in the
KiCad Device library is **bidirectional** — its pins are named `A1`/`A2` (anode/anode), which is the
`SMBJ15CA` the line above warns against. A unidirectional TVS is drawn as a zener: `Device:D_Zener`,
**pin 1 = K (cathode) → +12 V, pin 2 = A (anode) → GND**. Picking `D_TVS` because the name matches
draws the wrong part and the silkscreen band then means nothing.

⚠ **Choose 470 µF, not 220 µF.** The energy check below assumes 470 µF and lands at 23.9 V — just
under the 24.4 V clamp, so the TVS never conducts on a normal hard stop and stays true insurance.
At 220 µF the same 0.1 J reaches **32.5 V**, so the TVS conducts on *every* hard stop. A 600 W SMB
part survives that, but it becomes a working component instead of a backstop.

⚠ **35 V, not 25 V, on the electrolytics.** With the clamp at 24.4 V a 25 V part sits at its
rating. This applies to the bulk cap **and** to `C2`/`C4`, the 10 µF at each DRV8870 VM. An earlier
version of this section specified 25 V — that predates choosing the clamp voltage. DRV8870 VM is
rated to 45 V, so nothing else on the rail cares.

DRV8870 brake mode shorts the winding and dissipates most of it in the motor, so this is insurance
rather than the primary path — but it is cheap insurance on the rail the whole board hangs off.

#### ⚠ Placement reality — the bulk cap could NOT go near the drivers

This section says "near the drivers". As built, **only the TVS is**:

| Part | Distance to the DRV8870 VM pins | Why |
|---|---|---|
| `D8` TVS | **10 mm** | Must clamp at the source of the di/dt. Non-negotiable, and it fits. |
| `C39` bulk | **~46 mm** | No 10 mm square exists anywhere nearer. Measured, not assumed: with all 141 footprints placed, the closest free spot for a 10×10.5 mm can is (61.5, 40.5), and even an 8 mm can gets no closer than 44.5 mm. |

**Why ~46 mm is acceptable here.** The two jobs are on different timescales and different parts do
them:

- The **fast switching loop** is closed by `C23`/`C25` (100 n) and `C24`/`C26` (10 µF) sitting
  2–6 mm from each VM pin. Those are the ones that must be short, and they are.
- The **bulk electrolytic** absorbs *decel energy* over a millisecond-scale event. 46 mm of 12 V
  copper is ≈ 40–50 nH; at ~1 kHz that is sub-milliohm. It is irrelevant at that timescale.

So the split is deliberate: clamp at the source, energy store wherever it fits. ⚠ **Do not "fix"
this by moving `C23`–`C26` out to make room for `C39`** — that would trade a loop that matters for
one that does not. If the bulk cap must come closer, the thing to move is a connector, not the
VM decoupling.

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


### ⚠ Assembly rule — passives are 0805

**Every passive is 0805** (resistors, capacitors, ferrite beads) — changed from 1206 on
2026-08-26. Still comfortable under a trinocular scope, and it recovers meaningful area on a
crowded 100×100. Specify 0805 by default; depart only where dissipation demands it (see the IR
emitter's R_set in §11 — 129 mW exceeds an 0805).

**This does not extend to the actives, and two of them are the real assembly problem:**

| Part | Package | Hand-solderable? |
|---|---|---|
| D5 SS34 ORing, D1 B560C | SMA / SMC | ✅ Large, easy |
| BAT54S, 2N7002 | SOT-23 | ✅ Fine |
| **U7 MP2338** | **SOT583** (1.6 × 2.1 mm, leadless) | ⚠ Stencil/reflow or hot air only |
| **U5 TPS54560DDAR** | SO PowerPAD-8 | ⚠ Centre exposed pad |
| **ESP32-S3-WROOM-1U** | castellated + **EPAD** | ⚠ Castellations solder by hand; the EPAD does not. Reflow, or put a hole in the PCB under it |
| U-AMP MCP6002 | SOIC-8 | ✅ Fine |
| TCA9548A | TSSOP-24 | ⚠ Fine pitch, doable with flux and drag-soldering |

An exposed-pad regulator cannot be soldered with an iron alone — the thermal pad *is* the ground
connection and the primary heat path, and a buck without it will run hot and regulate badly.

**Hot air (Atten ST-862) and a trinocular scope are available, and the SMT side will likely be
machine-assembled.** That is what makes both the WROOM module and the MP2338's SOT583 viable
choices — neither would be sensible for iron-only assembly.


## 13. Bill of materials — changes from ver3_1

**Remove:**

- 2 × DRV8870DDAR (U3, U4), 2 × 200 mΩ (R3, R4), decoupling C5–C8, 2 × 6-pin connectors
  (CN3, CN4) — the 4WD→2WD cut
- **SSD1306 OLED (OLED1)** — purpose gone with BNO055 calibration, invisible in service, and a
  `setup()` hard-hang risk (§9)
- **BNO055** — superseded by BNO086 (§2.5)
- **CP2102** — not on the board either way; the DevKit carries its own (§3.2)
- **Battery divider (R1/R2 + filter cap)** — RETAINED, but moved **on-board** and re-valued
  100 k/20 k on GPIO4 (§5). The INA226 that briefly replaced it is **not fitted**

**Add:**

| Item | Qty | Note |
|---|---|---|
| **BNO086 IMU** | 1 | **§2.5.** SPI preferred; bring out INT + RST. Place AWAY from motors/power stage |
| ~~INA226 + shunt~~ | 0 | **NOT FITTED** — decision reversed 2026-08-22 (§5). Pack current is unmeasured; parked |
| **Current-sense amplifier** | 2 | **§4.** One per DRV8870 ISEN — turns stall detection from a heuristic into a measurement |
| TCA9548A / PCA9548A, TSSOP-24 | 1 | **TI or NXP part — reject PW548A clones** |
| 4-pin JST connectors (ToF) | 8 | one per mux channel, for the **VL53L0X already in hand** (§9) |
| Keyed I²C bench header (J-I2C) | 1 | doubles as the temporary-OLED port (§9) |
| J-EXP header, 2-pin | 0–1 | §3.3 — **3V3 + GND only, no I/O.** No spare GPIO exists. Consider deleting |
| **J-CONSOLE header, 3-pin** | 1 | §3.3 — UART0 fallback console |
| **ESP32-S3-DevKitC-1** | 1 | §3.2. Socketed. Any flash size; **N16R8 is fine** — the §3.1 map avoids GPIO33–37, so its PSRAM simply goes unused |
| **ESP32-S3-WROOM-1U-N16** | 1 | §3.2. Soldered. `U` = ext. antenna (no keep-out), `N16` = no PSRAM → GPIO35/36/37 free |
| USB-C receptacle, 16-pin | 1 | §3.2b Group 3. ⚠ **VBUS unconnected** — data only |
| **5.1 kΩ CC resistors** | 2 | ⚠ CC1→GND, CC2→GND. Miss these and the host never enumerates |
| USBLC6-2SC6 | 1 | ESD on D+/D− |
| Tactile switch | 2 | EN→GND (reset), GPIO0→GND (boot) |
| EN RC: 10 kΩ + 1 µF | 1 set | ⚠ Espressif: *"do not leave EN floating"* |
| STATUS_LED — plain LED + series R | 1 + 1 | GPIO38. ⚠ **Not addressable** — SK6812/WS2812 need ≥3.5 V, no 5 V rail exists (§3.2) |
| 3-pin connectors (prox) | 2 | 12 V, keyed |
| BAT54S clamp diodes | 3 | 2 × prox, 1 × battery ADC |
| 2N7002 / BSS138 | 1 | IR emitter driver |
| **TLV9002IDR** dual op-amp, LCSC `C398360` | 1 | §3.3 — motor current sense, both channels. ⚠ **NOT LM358** — cannot reach the rail |
| Buck #1 IC — **TPS54560DDAR**, 12 V @ 5 A | 1 | **replaces the external 16.8→12 V buck.** From VCC |
| Buck #2 IC — **MP1584EN-LF-Z-JSM**, 5.09 V | 1 | From **VCC**, not 12 V |
| Buck #3 IC — **MP1584EN-LF-Z-JSM**, 3.30 V | 1 | From **VCC**. Replaces the SSP1117-3.3, which is **DELETED** (§11) |
| Catch diodes — **B560C-13-F** ×1, **SS34** ×2 | 3 | D1 on buck #1, D2/D3 on the MP1584s |
| **SS34 ORing Schottky (D5)** — LCSC `C8678` | 1 | §4. DevKit 5V pin. **Not** a catch diode |
| **0 Ω 1206 (R48)** — LCSC `C17888` | 1 | §3.3 — the single AGND↔GND tie. ⚠ Never DNP |
| **NTC 10 kΩ B3950 0805** + 10 kΩ + 100 nF | 1 set | §3.1 `BOARD_TEMP` on IO5. Place beside the TPS54560 |
| Inductors, shielded | 3 | 6.8 µH (#1, sat ≥ 1.5× peak motor current), 22 µH ×2 |
| Buck passives (caps, FB dividers, BST, COMP) | ~28 | see §11 |
| **Ferrite bead** — TDK MPZ2012S601AT000, LCSC `C21519` | 1 | 0805, 600 Ω@100 MHz, 100 mΩ. Pi-filter on the BNO086 supply (§11) |
| Bulk electrolytic **470 µF** / **35 V** — `C39` | 1 | 12 V rail, motor regen (§11). ~46 mm from the drivers by necessity — §11 explains why that is fine |
| **TVS SMBJ15A** — LCSC `C699013` — `D8` | 1 | 12 V rail regen clamp, 10 mm from the VM pins. ⚠ `A` not `CA`; **not** a Schottky; symbol is `Device:D_Zener` **not** `D_TVS` (§11) |
| **MP2338** (SOT583) | 1 | §11 — buck #2, 3.3 V. ⚠ **V_ref 0.5 V**, not 0.8 V. Synchronous + COT: deletes a catch diode, a COMP network and an RT resistor |
| ~~MP1584EN~~ | 0 | Superseded by MP2338 (§11) |
| ~~5 V buck (U6) + L2 + D2 + passives~~ | 0 | **5 V RAIL DELETED** (§11) — ~14 parts recovered |
| ~~D5 SS34 ORing Schottky~~ | 0 | **DELETED** — no 5 V input exists (§11) |
| ~~5 V AUX LDO~~ | 0 | **Not fitted, no DNP footprint** — DNP pads still cost area (§11) |
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
