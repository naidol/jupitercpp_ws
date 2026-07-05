#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "esp32-hal-ledc.h"
#include "jupiter_config.h"

class Motor {
public:
    // Constructor to initialize the motor with PWM and DIR pins
    Motor(uint8_t pwm_pin, uint8_t dir_pin, uint8_t pwm_channel, uint32_t pwm_frequency, uint8_t pwm_resolution);

    // Function to set motor speed and direction
    void setSpeed(int speed);

private:
    uint8_t pwm_pin_;
    uint8_t dir_pin_;
    uint8_t pwm_channel_;
    uint32_t pwm_frequency_;
    uint8_t pwm_resolution_;

    // Helper function to get the maximum PWM value based on resolution
    int max_pwm_value() const;
};

#endif // MOTOR_H
