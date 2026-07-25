#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "esp32-hal-ledc.h"
#include "jupiter_config.h"

class Motor {
public:
    // Constructor to initialize the motor with PWM and DIR pins.
    // BOTH bridge inputs get their own LEDC channel so EITHER leg can be the PWM'd one:
    // forward PWMs the pwm_pin (dir held high), reverse PWMs the dir_pin (pwm held high).
    // That makes reverse fast-decay (brake-mode) like forward, instead of weak slow-decay.
    Motor(uint8_t pwm_pin, uint8_t dir_pin, uint8_t pwm_channel, uint8_t dir_channel,
          uint32_t pwm_frequency, uint8_t pwm_resolution);

    // Function to set motor speed and direction
    void setSpeed(int speed);

private:
    uint8_t pwm_pin_;
    uint8_t dir_pin_;
    uint8_t pwm_channel_;
    uint8_t dir_channel_;
    uint32_t pwm_frequency_;
    uint8_t pwm_resolution_;

    // Helper function to get the maximum PWM value based on resolution
    int max_pwm_value() const;
};

#endif // MOTOR_H
