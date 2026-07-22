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
#define K_P 5.0                            // P constant — raised from 2.0: larger wheels give lower req_rpm so startup PWM was too low to overcome stiction
#define K_I 5.0                            // I constant
#define K_D 0.0                            // D constant

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
#define COUNTS_PER_REV1 1372                // wheel1 encoder's no of ticks per rev (calibrated for 65mm rubber wheels)
#define COUNTS_PER_REV2 1372                // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 1372                // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 1372                // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.065                // wheel's diameter in meters (65mm rubber wheels)
#define LR_WHEELS_DISTANCE 0.346            // distance between left and right wheels
#define PWM_BITS 10                         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 8000                  // PWM Frequency
#define PWM_MAX pow(2, PWM_BITS) - 1        // e.g. for 8-bit PWM_MAX = 2^8 - 1 = 256 - 1 = 255
#define PWM_MIN -PWM_MAX
#define USE_PID true                        // true = PID controller ON. Switch OFF (false). See Note (1) below
#define WHEEL_RADIUS 0.050                  // in meters (100mm AGV wheels, installed 2026-07-11; was 0.0325/65mm rubber)
#define WHEEL_SEPARATION 0.355              // centre-to-centre, meters (100mm AGV wheels: 0.385 out-out minus one 30mm wheel width; was 0.346)
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