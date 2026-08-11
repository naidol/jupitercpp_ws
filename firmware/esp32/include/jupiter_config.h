//#######################################################################################################
// Name:             jupiter_config.h
// Purpose:          Main configuration header file for Jupiter Robot
// Description:      ESP32 pins and Robot properties can be user defined here 
// Related Files:         
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

// Motor Pins
#define MOTOR1_PWM  32
#define MOTOR1_DIR  33
#define MOTOR1_ENC_B  25
#define MOTOR1_ENC_A  26

#define MOTOR2_PWM  23 //19
#define MOTOR2_DIR  19 //23
#define MOTOR2_ENC_A  18
#define MOTOR2_ENC_B  5

#define MOTOR3_PWM  27
#define MOTOR3_DIR  14
#define MOTOR3_ENC_B  12
#define MOTOR3_ENC_A  13

#define MOTOR4_PWM  17 //16
#define MOTOR4_DIR  16 //17
#define MOTOR4_ENC_A  4
#define MOTOR4_ENC_B  15 

// Define Onboard LED
#define ESP32_LED 2

// Dock PROXIMITY sensors (LJ18A3-8-Z/BX, NPN open-collector, 12V supply) on the freed
// rear-encoder pins (the dead IR receivers that lived here were removed 2026-07).
// INPUT_PULLUP: HIGH = clear, LOW = metal detected (= physical contact with the dock).
#define PROX_LEFT_PIN   13      // rear-left  (was MOTOR3_ENC_A / IR_RECV_LEFT)
#define PROX_RIGHT_PIN  15      // rear-right (was MOTOR4_ENC_B / IR_RECV_RIGHT)
#define PROX_DEBOUNCE_CYCLES 5  // timer cycles (20ms each) both-LOW before "seated" latches (and clear before it drops)
// Seat grace: after the FIRST sensor makes contact, keep easing in for this long so the guide
// rails can square the robot and the SECOND sensor can engage. Stop immediately if both engage.
// Bounded so a permanently-missing second sensor can never grind the robot into the dock.
#define CONTACT_SEAT_GRACE_MS 1500

// Dock charge-enable IR EMITTER: TSAL6400 + 220R on GPIO4 (freed MOTOR4_ENC_A pin).
// 38kHz carrier (LEDC) gated in short-burst packets (TSOP-AGC-friendly — same envelope
// as the proven dock_beacon: 600us on/off x10 bursts, then 40ms gap ≈ 19 packets/s).
// Fires ONLY while seated (both prox) AND battery below full — the dock's Nano needs
// sustained packets to hold the SSR closed (dead-man's keepalive).
#define IR_EMIT_PIN        4
#define IR_EMIT_LEDC_CH    4          // LEDC channels 0-3 are the motors
#define IR_BURST_ON_US     600UL
#define IR_BURST_OFF_US    600UL
#define IR_BURSTS_PER_PKT  10
#define IR_PACKET_GAP_MS   40
#define BATTERY_FULL_STOP  16.70f     // stop requesting charge above this (4S full = 16.8V)

// cmd_vel WATCHDOG: no cmd_vel for this long -> motors stop (brake). Without it the last
// command LATCHES FOREVER if every publisher dies (proven: runaway spin into the doorway).
#define CMD_VEL_TIMEOUT_MS 400

// Battery ADC (GPIO34 = ADC1_CHANNEL_6, input-only pin)
#define BATTERY_ADC_CHANNEL  ADC1_CHANNEL_6
#define BATTERY_ADC_ATTEN    ADC_ATTEN_DB_11
#define BATTERY_V_DIV        (0.16510f)           // R2/(R1+R2) calibrated: nominal 20/120, scaled to match multimeter 16.152V @ firmware 16.0V
#define BATTERY_V_MIN        12.0f               // 4S Li-Ion depleted
#define BATTERY_V_MAX        16.8f               // 4S Li-Ion full charge
#define BATTERY_MA_SIZE      10                  // moving-average window (samples at 1 Hz = 10 s)


// FOR PID
// ⚠️ KNOWN ISSUE, DELIBERATELY NOT FIXED HERE (2026-08-08) — K_P IS MIS-SCALED FOR LOW SPEED.
// Measured (docs/bench_wheel_tracking_long.csv): the loop takes 4-9 s to reach a commanded wheel
// speed. K_P=5 is scaled for the FULL 0-214 RPM range (PWM_MAX=1023), so at docking speeds a
// ~5 RPM error asks for only ~25 counts (~2.4 % duty) against a wheel needing ~15-20 % to break
// friction — leaving the slow integral to do all the work. A raise to ~15 was drafted and
// REVERTED: this loop is shared with Nav2, and docking has moved to POSITION control
// (docs/DOCK_POSITION_CONTROL_SPEC.md) which takes the velocity lag off docking's critical path.
// Fix this when navigation performance is the subject, not as a side effect of docking work.
#define K_P 5.0                            // P constant — see mis-scaling note above
#define K_I 5.0                            // I constant
#define K_D 0.0                            // D constant

// Static-friction FEED-FORWARD (PWM units, 0..PWM_MAX). Added to the PID output in the
// COMMANDED direction whenever a wheel is told to move, so it breaks away immediately
// instead of waiting for the integral to crawl up to the stiction threshold — the root of
// the multi-second reverse-breakaway lag/skew and the "one wheel arcs" in-place rotation.
// Tune WHEELS-UP: raise until both wheels start promptly (<~0.2s) and together; lower if a
// stop->start visibly lurches. 0 = disabled (old behaviour).
#define MOTOR_FF_STATIC 200.0f
// BREAKAWAY-ONLY gating: apply the FF kick ONLY while a wheel is essentially STALLED
// (|rpm| < RELEASE) AND actually commanded to move meaningfully (|req_rpm| > CMD_MIN).
// Once the wheel is rolling, FF drops out so the PID has clean fine control (a continuous
// FF slams tiny heading corrections and made the reverse over-rotate & miss the dock).
//
// ⚠️ KNOWN ISSUE (2026-08-08) — THIS GATE CAUSES UNCOMMANDED YAW AT CRAWL SPEED.
// The hard on/off step sits at 4 RPM, right inside the old docking band. At v=0.04 m/s the
// wheels ran 3.4-7.1 RPM and crossed it repeatedly: FF on -> wheel accelerates -> FF drops
// 200 counts -> wheel decelerates -> FF on. Both wheels chatter INDEPENDENTLY across that step,
// and a difference between wheels IS a turn — measured 133 % L/R asymmetry at 0.04 m/s vs 3 %
// at 0.12 m/s. This is very likely what was long blamed on "leading caster drift".
// A tapered continuous FF (STATIC at rest -> ~140 rolling) was drafted as the fix and REVERTED
// for the same reason as K_P above: shared with Nav2, and position-control docking sidesteps it.
// INTERIM MITIGATION: do not operate below ~0.10 m/s — asymmetry collapses to ~3 % there.
#define MOTOR_FF_RELEASE_RPM 4.0f    // above this the wheel is "rolling" -> no FF
#define MOTOR_FF_CMD_MIN     3.0f    // don't kick a wheel commanded to ~zero (e.g. pivot-idle side)

// ===== POSITION-CONTROL (SEGMENT) MODE — /wheel_move =====================================
// Docking commands DISTANCES (encoder counts), not speeds. Steering is a differential
// distance: dS = theta * WHEEL_SEPARATION — pure geometry, independent of HOW FAST it
// happens. So a slow/ugly velocity profile still lands the wheels on their target counts and
// the turn comes out right. This takes the velocity loop's 4-9 s settling time (see K_P note
// above) off docking's critical path instead of fighting it.
// Full design + rationale: docs/DOCK_POSITION_CONTROL_SPEC.md
//
// ⚠️ Position mode SUSPENDS the cmd_vel watchdog (nothing publishes cmd_vel during a move, and
// the watchdog would brake the wheels 400 ms in). The stall + timeout guards below are its
// safety equivalent and are therefore NOT optional.
#define MOVE_K_POS            0.060f   // RPM commanded per count of remaining error
                                       // proportional band = MAX_RPM / K_POS ~= 420 counts (~10 cm) of deceleration
// MEASURED 2026-08-10: at 18 RPM a wheel overshot its target by 161 counts (~39 mm) and then
// STALLED trying to correct back. Cause is the velocity loop's slow settling (4-9 s, see the K_P
// note above) -- it cannot follow a fast deceleration ramp, so it sails past. Overshoot scales
// with segment speed, so precision segments must be SLOW. 12 RPM roughly halves it; raise only
// if a segment is long and accuracy at its end does not matter.
#define MOVE_DEFAULT_MAX_RPM  12.0f    // segment speed cap when the command doesn't specify one
#define MOVE_ACCEL_RPM_S      60.0f    // slew limit on the RPM command (trapezoidal profile, no jerk)
// MINIMUM SPEED FLOOR — measured 2026-08-10, first live position-mode test.
// A pure proportional law decelerates into a crawl the drivetrain cannot execute: at 27 counts
// remaining it asked for 1.6 RPM, the wheels stopped dead, and the stall guard (correctly) fired
// at 73/100 counts. Same failure the in-place SQUARE state hit years earlier, and the same fix:
// hold a floor speed while outside the deadband, then hard-stop inside it. Must sit ABOVE
// MOTOR_FF_RELEASE_RPM (4.0) so the breakaway kick is not chattering across its gate.
// 6.0 -> 10.0 (2026-08-10, live floor test): SHORT segments stall. A 76-count rotation sits
// entirely inside the deceleration band (MAX_RPM/K_POS = 200 counts), so it commands exactly
// MOVE_MIN_RPM for its whole length -- and 6 RPM cannot break stiction under the robot's weight.
// Two consecutive AIM segments (76 and 38 counts) died this way. Wheels-up they were fine; load
// is what changes it.
// 10 -> 20 (2026-08-11). The floor MUST sit above the breakaway threshold or it does nothing.
// Measured this session: arcs stall at 16 rpm and complete at 25. But speed = K_POS * remaining
// FLOORED at MOVE_MIN_RPM and only THEN capped by the commanded max -- so a SHORT segment never
// reaches its cap. A 100-count seat nudge asked for 0.06*100 = 6 rpm, was floored to 10, and its
// "30 rpm" cap never applied: three consecutive nudges stalled without moving the robot at all.
// 20 keeps short segments above the threshold; the overshoot it costs is absorbed by the
// arrived-on-overshoot latch and corrected by the outer loop.
#define MOVE_MIN_RPM          20.0f    // never command slower than this while outside tolerance
#define MOVE_DONE_TOL_COUNTS  20       // |error| within this = arrived (20 counts ~= 5 mm ~= 1.6 deg
                                       // of rotation). Widened from 8: with a 6 RPM floor the wheel
                                       // carries ~20 counts of stopping lag, and chasing a tighter
                                       // band just makes it hunt. Residual is corrected by the
                                       // OUTER loop (reflector re-measure), which is the whole point
                                       // of the segmented design.
#define MOVE_DONE_HOLD_MS     100      // must stay in tolerance this long before DONE
// 700 -> 1200 (2026-08-10, live dock): the final push stalled 10 mm short of the seat. When a
// wheel is blocked the duty it pushes with is K_P * commanded_rpm plus whatever the integral has
// built -- and at ~20 PWM/s the integral needs SECONDS. A 700 ms window killed the push before
// the force could develop. 1200 ms lets it build while still bounded well short of anything that
// would cook a motor.
#define MOVE_STALL_MS         1200     // no progress for this long -> ABORT_STALL
#define MOVE_STALL_MIN_COUNTS 3        // progress smaller than this counts as "no progress"
#define MOVE_MAX_SEGMENT_CNT  6200     // reject any segment longer than this (~1.5 m) — guards
                                       // against a bad computation driving across the room
// 3.0 -> 5.0 (2026-08-11). Driving FORWARD out of the dock makes the single rear caster flip
// 180 deg from its reverse-trailing position -- the slowest move this chassis can make. A 205
// count back-off got a 2.1 s budget and timed out mid-flip. The stall and divergence guards are
// the real protection; the timeout only needs to be generous enough not to fire on legitimate
// slow motion.
#define MOVE_TIMEOUT_FACTOR   5.0f     // abort after expected_time * this (+1 s floor)
#define MOVE_DIVERGE_COUNTS   150      // abort if remaining error GROWS this far beyond its start
                                       // value (~3.6 cm). Catches an inverted encoder sign or a
                                       // wheel driven the wrong way: without it the loop reads a
                                       // growing error, holds full speed, and runs until timeout
                                       // (~3.6 m on a long segment) instead of stopping at once.

// BATTERY-VOLTAGE COMPENSATION (2026-08-08). Motor torque per PWM count scales with pack
// voltage, so gains tuned at one state of charge run ~17 % hotter on a full pack. The robot
// docks when it is LOW (that is the whole point of docking), so the tune is done there —
// this normalises the duty to MOTOR_V_NOMINAL so the SAME tune behaves identically from
// 16.8 V down to the docking range, and navigation at full charge does not get quietly
// stronger. Guarded: an unread/implausible battery reading leaves the duty untouched.
#define MOTOR_V_NOMINAL   14.4f      // reference pack voltage — the docking-condition tuning point
#define MOTOR_V_COMP_MIN  0.80f      // clamp on the scale factor (16.8 V -> 0.857)
#define MOTOR_V_COMP_MAX  1.25f      // clamp on the scale factor (12.0 V -> 1.200)

// Per-motor trim to compensate physical motor mismatch.
// Tune: if robot drifts right, reduce MOTOR1_TRIM. If drifts left, reduce MOTOR2_TRIM.
// Levelled chassis reading: motor1~115 RPM, motor2~137 RPM → motor2 trim = 115/137 = 0.84
#define MOTOR1_TRIM 1.00f                   // reset to neutral 2026-07-11: old 0.92/0.93 were tuned for the 65mm soft wheels
#define MOTOR2_TRIM 1.00f                   // and caused a forward LEFT-bank on the 100mm AGV wheels; re-tune on new wheels


// MOTOR AND ROBOT SPECS
#define MOTOR_MAX_RPM 330                   // motor's max RPM          
#define MAX_RPM_RATIO 0.65                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 1290                // ticks per WHEEL rev — re-calibrated 2026-07-27 for the 100mm AGV wheels (was 1372, for old 65mm rubber). Tape-measured: 4267 counts / (1.05m / 0.3142m-circ) ~= 1290.
#define COUNTS_PER_REV2 1290                // (odom was ~6% low with 1372; the big 2.6x error was the getRPM 0.0-return bug, fixed separately)
#define COUNTS_PER_REV3 1290                // motors 3&4 unused (rear caster), kept consistent
#define COUNTS_PER_REV4 1290
// REMOVED 2026-08-08: WHEEL_DIAMETER (0.065) and LR_WHEELS_DISTANCE (0.346) were stale values
// from the old 65 mm rubber wheels / earlier chassis measurement. Both were referenced NOWHERE
// in the firmware (verified), but sat next to the live values as a trap — anyone reaching for
// "wheel diameter" would have got 65 mm instead of 100 mm, a 35 % error. The live geometry is
// WHEEL_RADIUS and WHEEL_SEPARATION below; use only those.
#define PWM_BITS 10                         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 8000                  // PWM Frequency
#define PWM_MAX pow(2, PWM_BITS) - 1        // e.g. for 8-bit PWM_MAX = 2^8 - 1 = 256 - 1 = 255
#define PWM_MIN -PWM_MAX
#define USE_PID true                        // true = PID controller ON. Switch OFF (false). See Note (1) below
#define WHEEL_RADIUS 0.050                  // in meters (100mm AGV wheels, installed 2026-07-11; was 0.0325/65mm rubber)
// CALIBRATED 2026-08-10 (was 0.355, a geometric estimate: 0.385 out-out minus one 30mm wheel
// width). Measured by rotating in place and comparing DELIVERED differential encoder counts
// against BNO055 yaw -- not commanded vs actual, which is polluted by overshoot and stalls.
// Three segments gave 0.3583 / 0.3590 / 0.3585 m, spread 0.21 %. The old value made every
// commanded angle 1.02 % small. theta = (sR - sL)/W, so this scales ALL rotation: position-mode
// turns, Nav2's angular commands, and the odometry's reported angular_z (which was
// under-reporting rotation by the same 1 %). 25.70 differential counts per degree.
#define WHEEL_SEPARATION 0.3586             // centre-to-centre, meters — CALIBRATED, do not "tidy" back to a round number
#define WHEEL_BASE 0.180                    // <-- ADD THIS: distance from front axle to rear axle in meters
#define PWM_FWD_MIN 1                       // these two PWM_FWD & PWM_REV are needed to balance the startup power needed
#define PWM_REV_MIN 1                       // for each direction (helps to drive straight)  [REVERTED 2026-07-11: floor/kick experiments made drive worse — see project memory]

// NOTES
// (1)  Without PID control, the robot motors may only spin at higher speed commands from cmd_vel as there is no feedback from the wheel encoders
//      about the current speed. Essentially, there is no 'error' that can be reduced auto-matically by the open loop system.
//      So the operator (human) will need to increase speeds manually until the robot moves.  The operator will be the 'closed-loop" control system.
//      The PID controller continuously adjusts the PWM signals to the motors to get the current speeds to the target speeds using the wheel encoder
//      data as feedback measure of the current speed.
//
// (2)  The Kp, Ki and Kd settings for PID control is different for each motor and robot kinematics.  This will need to be tuned manually.
//      Start with Kp first, with Ki and Kd = 0.  Then adjust each until there is a smooth rotation of the wheels.
//      It may be necessary to watch some YouTube examples about PID tuning.  Also make sure that the PID control loop (dt) has sufficient time to 
//      execute the PID calculations needed to control the system.