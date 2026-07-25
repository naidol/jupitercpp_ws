#include "motor.h"
#include "esp32-hal-ledc.h"

// Constructor to initialize the motor with PWM and DIR pins.
// Both bridge inputs are driven by their own LEDC channel (see setSpeed for why).
Motor::Motor(uint8_t pwm_pin, uint8_t dir_pin, uint8_t pwm_channel, uint8_t dir_channel,
             uint32_t pwm_frequency, uint8_t pwm_resolution)
    : pwm_pin_(pwm_pin), dir_pin_(dir_pin), pwm_channel_(pwm_channel), dir_channel_(dir_channel),
      pwm_frequency_(pwm_frequency), pwm_resolution_(pwm_resolution) {

    // Attach a LEDC channel to EACH bridge input. dir_pin is no longer a plain GPIO —
    // it is PWM'd during reverse, so it must be LEDC-driven (and held HIGH via full duty
    // when it needs to be a constant level).
    ledcSetup(pwm_channel_, pwm_frequency_, pwm_resolution_);
    ledcAttachPin(pwm_pin_, pwm_channel_);
    ledcSetup(dir_channel_, pwm_frequency_, pwm_resolution_);
    ledcAttachPin(dir_pin_, dir_channel_);
}

// Set motor speed + direction. IN/IN H-bridge truth table:
//   (dir=H, pwm=L)=forward drive   (dir=L, pwm=H)=reverse drive
//   (H,H)=BRAKE (fast decay)       (L,L)=COAST (freewheel)
//
// FAST-DECAY (brake-mode) in a direction = PWM between that drive state and BRAKE,
// holding the OTHER input HIGH. Forward holds dir HIGH and PWMs pwm; reverse holds pwm
// HIGH and PWMs dir. Previously reverse held dir LOW and PWM'd pwm -> it alternated
// drive with COAST (slow decay) = electrically weak, which caused the multi-second
// reverse-breakaway stall and the lurch. Now reverse mirrors forward. (2026-07-25)
void Motor::setSpeed(int speed) {
    if (speed == 0) {
        // BRAKE (instant stop): both inputs HIGH.
        ledcWrite(dir_channel_, PWM_MAX);
        ledcWrite(pwm_channel_, PWM_MAX);
    }
    else if (speed > 0) {
        // FORWARD, fast decay: dir HIGH, PWM pwm_pin. Low fraction = drive, so a larger
        // magnitude writes a smaller duty (more time in the drive state).
        int mag = constrain(speed, PWM_FWD_MIN, PWM_MAX);
        ledcWrite(dir_channel_, PWM_MAX);        // constant HIGH
        ledcWrite(pwm_channel_, PWM_MAX - mag);  // toggles LOW(drive) <-> HIGH(brake)
    }
    else {
        // REVERSE, fast decay (brake-mode): pwm HIGH, PWM dir_pin. Mirror of forward.
        int mag = constrain(-speed, PWM_REV_MIN, PWM_MAX);
        ledcWrite(pwm_channel_, PWM_MAX);        // constant HIGH
        ledcWrite(dir_channel_, PWM_MAX - mag);  // toggles LOW(drive) <-> HIGH(brake)
    }
}


// Helper function to get the maximum PWM value based on resolution
// int Motor::max_pwm_value() const {
//     return (1 << pwm_resolution_) - 1;  // e.g., 8-bit resolution -> 255
// }
