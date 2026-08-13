// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// TEMPORARY DIAGNOSTIC firmware — VL53L0X ranging bench for the Jupiter front ToF.
//
// PURPOSE: verify the 8 deg upward mount tilt against the REAL floor. The angle was sized
// analytically (floor-hit distance ~= mount_height / tan(12.5 deg - tilt), predicting the floor
// leaves the cone beyond ~0.8-0.9 m at 80 mm height) but has never been measured. Cone half-angle
// and floor reflectivity both vary from the datasheet in practice, so this is the check that
// decides whether 8 deg is right before three more mounts get printed.
//
// Reads continuously and prints distance plus the sensor's own range status, so an out-of-range
// or low-signal reading is distinguishable from a genuine long measurement -- the VL53L0X returns
// a plausible-looking number in several failure modes, which is exactly how a floor-detection test
// gets fooled.
//
// ===========================================================================================
// RESULT, 2026-08-12 -- THE 3D PRINTED CASING WAS REFLECTING THE LASER INTO THE RECEIVER.
//
//   in the printed mount:   ~4% valid readings   signal  0.50 MCPS   background 14.20 MCPS
//   same sensor, free air:  175/175 valid        signal 22.94 MCPS   background  0.04 MCPS
//
// Optical crosstalk: the bore caught the VCSEL's own emission and returned it to the receiver a
// few mm away. It is CONSTANT by construction -- it moved with the sensor, so no change of room,
// target or even a finger on the glass shifted it off ~14 MCPS. That constant buried the real
// return (under 2 MCPS at 0.6 m), so the histogram never formed a peak worth trusting and the
// sensor reported range-phase-check / phase-consistency / MSRC-no-target, NOT "no signal".
//
// Diagnosing this took a full day, mostly down blind alleys. What misled us, in order:
//   - Ambient IR / daylight. Killed by an A/B: the default profile is far more ambient-immune
//     and failed identically. Ambient would not do that.
//   - VCSEL brownout on the 3.3 V rail (the GY-530 has an onboard LDO and is spec'd 2.8-5 V).
//     Killed by the free-air run: 22.9 MCPS of signal on the SAME 3.3 V rail is a healthy laser.
//   - The wire-wrap and the sensor itself. Both were fine the whole time.
// The one test that settled it was pulling the module OUT of its mount and changing nothing else.
//
// If a mount is ever reprinted: black matte filament (light PLA reflects strongly at 850 nm),
// aperture >= 6-7 mm chamfered outward (cone is +/-12.5 deg, emitter and receiver windows sit
// ~2.8 mm apart), and the module face FLUSH or proud -- never recessed.
//
// Also measured, and still uncorrected: a large per-profile range bias. Against a box at a
// tape-measured 600 mm, DEFAULT read 680 mm (+80) and LONG-RANGE read 528 mm (-72), each
// internally tight. Whatever profile ships must be fixed at build time and offset-calibrated
// (ALGO_PART_TO_PART_RANGE_OFFSET_MM, signed, 1/4 mm units). NEVER switch profile at runtime --
// the alternation below is a diagnostic only. Two known distances are needed to tell a constant
// offset from a scale error; that was not done.
//
// Separately: the TCA9548A mux never enumerated at any address despite correct 3.3 V (the board
// is 1.65-5.5 V, no regulator, so 3.3 V is right), RST high and A0-A2 grounded. Chip reads as a
// PW548A clone where a TI part was advertised. Treated as dead.
// ===========================================================================================
//
// NOT the real firmware. No micro-ROS, no motors, no odometry. Re-flash firmware/esp32 after:
//   ~/tools/esptool-linux-aarch64/esptool --port /dev/jupiter_esp32 --baud 460800 --chip esp32 \
//       write-flash -z 0x10000 ~/firmware_new/firmware.bin

#include <Arduino.h>
#include <Wire.h>

static const uint8_t SDA_PIN = 21;
static const uint8_t SCL_PIN = 22;
static const uint8_t TOF_ADDR = 0x29;

#include <VL53L0X.h>

VL53L0X tof;
static bool tof_ready = false;

// The library reports timeouts itself; map its state to something readable at the bench.
static const char* reading_note(uint16_t mm, bool timeout) {
  if (timeout)     return "TIMEOUT (no return — nothing in range?)";
  if (mm >= 8190)  return "OUT OF RANGE (nothing detected)";
  if (mm < 30)     return "TOO CLOSE / spurious";
  return "OK";
}

// Raw register read, bypassing the driver entirely. If the driver is unhappy this says whether
// the BUS is the problem or the driver is: model ID must read 0xEE on a live VL53L0X.
static int raw_reg(uint8_t reg) {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(TOF_ADDR, (uint8_t)1) != 1) return -2;
  return Wire.read();
}

// ---------------------------------------------------------------------------------------------
// The Pololu driver returns a bare distance and discards WHY a reading failed, which makes an
// 8190 ("no signal") indistinguishable from an 8190 ("swamped by sunlight") -- the exact question
// at hand. These read the sensor's own result block directly:
//   0x14 status | 0x1A signal count rate | 0x1C ambient count rate | 0x1E range mm
// Rates are 9.7 fixed-point MCPS. Ambient >> signal is the fingerprint of IR from daylight.
static int raw_reg16(uint8_t reg, uint16_t* out) {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(TOF_ADDR, (uint8_t)2) != 2) return -2;
  *out = (uint16_t)(Wire.read() << 8) | Wire.read();
  return 0;
}

static const char* status_name(uint8_t s) {
  switch (s) {
    case 0:  return "data-not-ready";
    case 1:  return "VCSEL-continuity-FAIL";   // laser itself is not driving
    case 2:  return "VCSEL-watchdog-FAIL";
    case 3:  return "no-VHV-value";
    case 4:  return "MSRC-NO-TARGET";          // nothing came back: blocked lens, aim, or too far
    case 5:  return "SNR-check";
    case 6:  return "range-phase-check";
    case 7:  return "sigma-threshold";         // return too noisy to trust -- classic ambient swamp
    case 8:  return "TCC-crosstalk";
    case 9:  return "phase-consistency";
    case 10: return "min-clip";
    case 11: return "RANGE-VALID";
    case 12: return "algo-underflow";
    case 13: return "algo-overflow";
    case 14: return "range-ignore-threshold";
    default: return "unknown";
  }
}

static bool bring_up() {
  const int model = raw_reg(0xC0);            // IDENTIFICATION_MODEL_ID
  const int rev   = raw_reg(0xC2);            // IDENTIFICATION_REVISION_ID
  Serial.printf("  bring-up: model_id=0x%02X (want 0xEE)  rev=0x%02X  ", model, rev);
  if (model < 0) { Serial.println("-> BUS READ FAILED (no ACK / no data)"); return false; }
  if (model != 0xEE) { Serial.println("-> not a VL53L0X at this address"); return false; }

  tof.setTimeout(500);
  if (!tof.init()) { Serial.println("-> init() FAILED (SPAD/VHV/phase calibration did not complete)");
                     return false; }
  Serial.println("-> OK");
  return true;
}

// Two profiles, alternated so the ambient-immunity trade is measured rather than assumed.
static void apply_profile(bool long_range) {
  tof.stopContinuous();
  delay(50);
  if (long_range) {
    // Reach, at the cost of ambient-light immunity: a lower signal-rate limit accepts weaker
    // returns and longer VCSEL periods put more energy downrange.
    tof.setSignalRateLimit(0.1);
    tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tof.setMeasurementTimingBudget(50000);
  } else {
    // Stock profile. Shorter reach but far harder to swamp -- if this one works and long-range
    // does not, ambient IR is the answer.
    tof.setSignalRateLimit(0.25);
    tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
    tof.setMeasurementTimingBudget(33000);
  }
  tof.startContinuous();
  Serial.printf("\n  ===== PROFILE: %s =====\n", long_range ? "LONG-RANGE (0.10 MCPS, 18/14, 50ms)"
                                                            : "DEFAULT (0.25 MCPS, 14/10, 33ms)");
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.println("\n========== VL53L0X RANGING BENCH ==========");
  Serial.printf("addr 0x%02X on SDA=GPIO%u SCL=GPIO%u\n", TOF_ADDR, SDA_PIN, SCL_PIN);
  Serial.println("WHAT TO CHECK:");
  Serial.println("  1. CLEAR FLOOR AHEAD -> readings should be LONG or out-of-range.");
  Serial.println("     A short stable reading here means the cone is clipping the floor and");
  Serial.println("     the tilt is too shallow -- that is a false obstacle in the reflex zone.");
  Serial.println("  2. Object at 0.3 / 0.5 / 1.0 m -> should read close to true distance.");
  Serial.println("  3. Watch the SPREAD, not just the value: jitter matters more than offset.\n");
  tof_ready = bring_up();
  if (tof_ready) apply_profile(false);
}

void loop() {
  // Re-attempt bring-up rather than printing timeouts forever: this way the banner explaining
  // WHY it is not ranging reappears every second, whenever the bench operator attaches.
  if (!tof_ready) { delay(1000); tof_ready = bring_up(); if (tof_ready) apply_profile(false); return; }

  // PROFILE IS LOCKED to DEFAULT. The earlier 10 s alternation was a diagnostic and it must not
  // run during characterisation: the two profiles disagreed by 152 mm on the same target, so
  // switching mid-capture corrupts every statistic taken across the boundary.

  const uint16_t mm = tof.readRangeContinuousMillimeters();
  const bool to = tof.timeoutOccurred();

  uint16_t sig = 0, amb = 0;
  const int st_raw = raw_reg(0x14);
  raw_reg16(0x1A, &sig);
  raw_reg16(0x1C, &amb);
  const uint8_t st = (st_raw < 0) ? 0xFF : (uint8_t)((st_raw & 0x78) >> 3);

  Serial.printf("  %5u mm  %-38s  status=%-22s sig=%6.2f amb=%6.2f MCPS\n",
                mm, reading_note(mm, to), status_name(st), sig / 128.0, amb / 128.0);
  delay(250);
}
