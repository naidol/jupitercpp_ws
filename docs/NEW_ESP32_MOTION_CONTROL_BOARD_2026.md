# New ESP32 Motion Control Board — 2026

**Purpose:** design brief for the successor to `Jupiter ESP32 Drv8870 12vDC ver3_1`
(EasyEDA, JLCPCB-002, drawn 2024-06-29). Captures every change made to the robot since that
board was fabricated, so the schematic can be redrawn and sent to JLCPCB in one pass.

**Status:** specification only. Nothing here has been drawn yet.
**Source of truth for pin assignments:** `firmware/esp32/include/jupiter_config.h`.
**Predecessor schematic:** `~/Documents/Jupiter_ESP32_Board_easyeda.pdf`.

> ### ⚠ Resolve these three before drawing anything
> They are contradictions between the old schematic, the firmware and the docs. Each one is
> cheap to settle with a multimeter and expensive to get wrong on a fabricated board.
> They are detailed in §9.
>
> 1. What voltage actually feeds DRV8870 `VM` — a regulated 12 V, or the raw 4S pack?
> 2. What are the two real resistor values in the battery divider — 100 k/20 k or 100 k/22 k?
> 3. Three GPIOs are defined twice in firmware (13, 15, 4). Confirm which function is physically wired.

---

## 1. What changed since ver3_1

| # | Change | Consequence for the new board |
|---|---|---|
| 1 | **4WD → 2WD.** Two driven wheels + one rear caster (180 mm behind the drive axle) | Two DRV8870 channels are now dead weight |
| 2 | **Battery monitoring added.** Resistive divider → GPIO34 | Currently off-board / flying. Needs to be a designed circuit |
| 3 | **Dock proximity sensors added.** 2 × LJ18A3-8-Z/BX inductive, **12 V**, NPN open-collector | 12 V logic arriving at a 3.3 V pin with no protection |
| 4 | **Dock IR emitter added.** TSAL6400 + 220 Ω on GPIO4, 38 kHz | Driven straight from a GPIO. Under-driven, see §6 |
| 5 | **Wheels changed** 65 mm rubber → 100 mm AGV | Firmware constants only, no PCB impact |
| 6 | **Strapping-pin pull-downs needed** | Not on ver3_1 at all. See §7 — this is the most likely cause of intermittent boot failures |
| 7 | **I²C expansion:** TCA9548A mux + up to 7 × VL53L0X ToF | New. Currently hand-wired and **not working** — see §8 |
| 8 | Position-control mode (`/wheel_move`) added to firmware | Software only, no PCB impact |

---

## 2. Pin map — current firmware, authoritative

From `jupiter_config.h`. **Bold = in active use on the 2WD robot.**

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
| MOTOR3_ENC_B | 12 | **free** — ⚠ strapping pin, see §7 |
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
Ample headroom. GPIO 12 should be spent last, or given a pull-down (§7).

---

## 3. Motor drive — 2 channels

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

---

## 4. Battery monitoring

Currently a flying divider. Make it a designed block.

```
   PACK+ (16.8 V max)
      |
     R1  100 k
      |
      +----+----> GPIO34  (ADC1_CH6)
      |    |
     R2   C  100 nF
      |    |
     GND  GND
```

- Firmware constant: `BATTERY_V_DIV = 0.16510` — **calibrated against a multimeter**, not nominal.
  The comment says "nominal 20/120", i.e. R1 = 100 k, R2 = 20 k → 0.1667. See §9.2.
- Add the **100 nF** to ground at the ADC node. The ESP32 SAR ADC wants a low-impedance source and
  a 100 k/20 k divider is nowhere near it; this is likely a real contributor to ADC noise today.
- Consider a **BAT54S clamp** to 3.3 V/GND at the pin. Cheap insurance on a node tied to the pack.
- GPIO34 is **input-only** — no internal pull-up/down exists, which is exactly why it's a good ADC pin.
- Full scale check: 16.8 V × 0.1667 = **2.80 V**, inside the 11 dB attenuation range (~3.1 V). Good,
  with headroom. Do not raise the divider ratio.

---

## 5. Dock proximity sensors — needs protection

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

## 6. Dock IR emitter — currently under-driven

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

## 7. Strapping-pin pull-downs — do not skip

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

## 8. I²C bus, mux and ToF fan-out

### Current bus

| Device | Address | Notes |
|---|---|---|
| BNO055 IMU | `0x28` | on-board, works |
| SSD1306 OLED | `0x3C` | on-board, works |
| TCA9548A mux | `0x70` | **hand-wired, never enumerated — treated as dead** |
| VL53L0X ToF | `0x29` | all identical, hence the mux |

`Wire.begin(21, 22)` at **400 kHz** (BNO055 fast mode).

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
| Upstream (ESP32 ↔ mux, IMU, OLED) | **2.2 k** to 3.3 V. 4.7 k is marginal at 400 kHz once several devices and cable capacitance are on the bus |
| Each downstream channel | **4.7 k** to 3.3 V, **on the board**, one pair per active channel |

The TCA9548A does **not** pass pull-ups through — every downstream segment needs its own pair.
Relying on the pull-ups inside each GY-530 breakout works, but the value then depends on which
module is plugged in.

### ToF power budget

7 × VL53L0X at ~20 mA ≈ **140 mA** on 3.3 V, plus IMU, OLED and the mux. That is a material load —
see §10, it is the main argument for replacing the linear regulators.

Keep the ToFs on **3.3 V**. Measured: 22.9 MCPS of return signal on the 3.3 V rail is a perfectly
healthy VCSEL. An earlier theory that the GY-530's onboard LDO was browning out at 3.3 V was
tested and is **wrong**.

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

## 9. The three contradictions to resolve first

### 9.1 What feeds DRV8870 `VM`?

ver3_1 shows **+12 V**. But the firmware compensates motor duty against *measured pack voltage*:

```c
#define MOTOR_V_NOMINAL   14.4f   // reference pack voltage
#define MOTOR_V_COMP_MIN  0.80f   // clamp (16.8 V -> 0.857)
#define MOTOR_V_COMP_MAX  1.25f   // clamp (12.0 V -> 1.200)
```

Those clamps span **12.0–16.8 V**, which is the 4S pack range, not a regulated 12 V rail. If VM
were truly a fixed 12 V, that entire compensation would be meaningless — and it is load-bearing
for docking, which was tuned at 14.4 V.

**Measure VM at U1 pin, on the bench, at a known pack voltage.** If it tracks the pack, draw the
new board with **VM = pack**, fused and reverse-protected, and delete the 12 V motor rail.

### 9.2 Battery divider resistor values

- `CLAUDE.md` says **R1 = 100 k, R2 = 22 k** → ratio 0.1803
- `jupiter_config.h` says *"nominal 20/120"* → **R1 = 100 k, R2 = 20 k** → ratio 0.1667
- Calibrated in firmware: **0.16510**

0.16510 is far closer to the 20 k figure. **Read the actual resistors before drawing the block.**
Getting this wrong shifts every battery reading and the `BATTERY_FULL_STOP = 16.70 V` charge cutoff.

### 9.3 Triple-defined GPIOs

`13`, `15` and `4` are each defined **twice** in `jupiter_config.h` — once as a motor-3/4 encoder,
once as prox/IR — and the firmware still constructs `motor3_encoder` and `motor4_encoder` on those
same pins. On the physical 2WD robot the prox/IR meaning is the live one, but the stale encoder
objects are still attaching to them.

Harmless today only because motors 3 and 4 drive nothing. **On the new board, delete the motor-3/4
encoder definitions entirely** so the mapping is unambiguous, and clean up the firmware to match.

---

## 10. Power supply — one buck, then keep the linear

ver3_1 uses **SSP1117-3.3 V** and **SSP1117-5.0 V** linear regulators, both from the 12 V rail.
Both are now stressed. Note the ESP32 DevKit does **not** sit on the 3.3 V rail — ver3_1 feeds it
5 V into VIN and it regulates its own 3.3 V onboard.

| Rail | Feeds | Est. current | Dissipation, linear from 12 V |
|---|---|---|---|
| 5 V | ESP32 DevKit, IR driver, J3 header | ~250–350 mA | **1.8–2.4 W** — already marginal |
| 3.3 V | BNO055, OLED, mux, 7 × ToF, DRV8870 VREF | ~200–400 mA | **1.7–3.5 W** — will not work |

### Architecture

```
  PACK/12V ──► BUCK ──► 5 V (2 A) ──┬──► ESP32 DevKit VIN
                                     ├──► IR emitter driver (§6)
                                     └──► SSP1117-3.3 ──► 3.3 V ──► IMU, OLED, mux, ToFs
```

**One switcher, not two.** The 3.3 V rail stays linear, which matters more than usual here: it
feeds a photon-counting ToF array and sits beside a 12-bit ADC reading pack voltage, and switching
ripple is the last thing either wants. Dissipation in the retained LDO drops to
(5 − 3.3) × 0.4 A ≈ **0.7 W** — comfortable in SOT-223. **The existing SSP1117-3.3 is kept**,
simply re-sourced from 5 V instead of 12 V.

### Added components

| Item | Value / part | Notes |
|---|---|---|
| Buck IC | **MP1584EN**, SOIC-8-EP | 3 A, 4.5–28 V in. Cheap, well stocked. 28 V covers 12 V **or** pack-fed (§9.1) |
| *alternative* | AP63205WU-7, TSOT-23-6 | 2 A, 3.8–32 V, integrated bootstrap, low EMI, fewer externals |
| *avoid* | TPS563201 | 17 V max — no margin against a 16.8 V pack |
| Inductor | 10 µH, ≥3 A sat, **shielded** | Unshielded next to a 400 kHz I²C bus is asking for trouble |
| Input caps | 2 × 10 µF / 50 V X7R (1206) + 100 nF | Rated for pack plus transients |
| Output caps | 2 × 22 µF / 16 V X7R | |
| Bootstrap cap | 100 nF / 50 V | |
| Feedback divider | 52.3 k / 10 k, 1 % | 0.8 V ref → 0.8 × (1 + 52.3/10) = 4.98 V |
| Enable pull-up | 100 k to VIN | |
| Freq-set resistor | per datasheet | Depends on chosen switching frequency |

⚠ The **inductor value and freq-set resistor above are a starting point, not a verified design.**
Confirm against the MP1584 datasheet's typical-application table for the actual input range.
Also check LCSC stock and whether the part is JLC **Basic or Extended** — Extended carries a
per-feeder setup fee, which is material on a one-off run.

### Switcher layout

- Keep the **switch node** (IC → inductor) as small as physically possible — it is the main radiator.
- Input cap **directly** across the IC's VIN/GND, shortest possible loop.
- Route the **feedback trace away from the inductor**; reference it to output-cap ground.
- **Physically separate the switcher from the I²C fan-out, ToF connectors and the battery ADC node.**
  Put it at the power-input end, sensors at the far end.

### Lower-risk alternative

Footprint a **ready-made MP1584 buck module** on a 4-pin header instead. Zero layout risk,
hand-fitted after assembly, not JLC-assemblable. For a one-off build that is a rational trade.

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

## 11. Layout notes

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

## 12. Bill of materials — changes from ver3_1

**Remove:** 2 × DRV8870DDAR (U3, U4), 2 × 200 mΩ (R3, R4), associated decoupling
(C5–C8), 2 × 6-pin connectors (CN3, CN4).

**Add:**

| Item | Qty | Note |
|---|---|---|
| TCA9548A / PCA9548A, TSSOP-24 | 1 | **TI or NXP part — reject PW548A clones** |
| 4-pin JST connectors (ToF) | 8 | one per mux channel |
| 3-pin connectors (prox) | 2 | 12 V, keyed |
| BAT54S clamp diodes | 3 | 2 × prox, 1 × battery ADC |
| 2N7002 / BSS138 | 1 | IR emitter driver |
| Buck regulator (MP1584EN or AP63205) | 1 | new 5 V rail; SSP1117-3.3 is RETAINED, re-sourced from 5 V |
| Inductor 10 µH, ≥3 A, shielded | 1 | buck |
| Buck passives (caps, FB divider, BST) | ~8 | see §10 |
| Resistors: 10 k pull-down/up (strapping) | ~8 | §7 |
| Resistors: 2.2 k (upstream I²C pull-up) | 2 | |
| Resistors: 4.7 k (per-channel I²C pull-up) | 16 | 2 per active mux channel |
| 100 nF decoupling | ~6 | mux, ADC node, prox inputs |
| Ideal-diode reverse protection FET | 1 | |

---

## 13. Open questions for Logan

1. **7 ToFs or 8?** The mux has 8 channels. Are all 7 front-facing, or are some cliff/rear?
   Placement drives connector positions on the board edge.
2. **Is the 12 V rail still needed at all** if VM turns out to be pack-fed (§9.1)? The prox sensors
   need 12 V, so probably yes — but it may become a small dedicated supply rather than the main rail.
3. **Board outline / mounting** — unchanged from ver3_1?
4. **JLC assembly scope** — full assembly, or hand-solder the connectors? Affects whether the mux
   must be a JLC "basic" part.

---

*Written 2026-08-12. Pin data from `firmware/esp32/include/jupiter_config.h` @ `ba0a7fe`.
Predecessor: `Jupiter ESP32 Drv8870 12vDC ver3_1`, EasyEDA, JLCPCB-002, 2024-06-29.*
