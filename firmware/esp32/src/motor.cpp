#include "motor.h"
#include "esp32-hal-ledc.h"

// Constructor to initialize the motor with PWM and DIR pins
Motor::Motor(uint8_t pwm_pin, uint8_t dir_pin, uint8_t pwm_channel, uint32_t pwm_frequency, uint8_t pwm_resolution)
    : pwm_pin_(pwm_pin), dir_pin_(dir_pin), pwm_channel_(pwm_channel),
      pwm_frequency_(pwm_frequency), pwm_resolution_(pwm_resolution) {
    
    // Initialize the DIR pin as output
    pinMode(dir_pin_, OUTPUT);

    // Initialize PWM using the specified parameters
    ledcSetup(pwm_channel_, pwm_frequency_, pwm_resolution_);

    // Attach the PWM channel to the specified PWM pin
    ledcAttachPin(pwm_pin_, pwm_channel_);
}

// Function to set motor speed and direction
void Motor::setSpeed(int speed) {
    if (speed == 0) {
        // Full stop: Disable the PWM signal by writing 0 duty cycle
        ledcWrite(pwm_channel_, 0);
        
        // Optionally set both motor pins to LOW for braking
        digitalWrite(dir_pin_, LOW);
    } 
    else if (speed > 0) {
        // Set direction to forward
        digitalWrite(dir_pin_, HIGH);
        // Set the PWM duty cycle
        int duty_cycle = constrain(speed, PWM_FWD_MIN, PWM_MAX);  // min power needed for forward motion adjust PWM_FWD_MIN in jupiter_config
        // Invert the pwm duty cycle
        duty_cycle = PWM_MAX - duty_cycle;      // 0 - 255 means 255 - 0 that is HIGH PWM duty means lower speed
        ledcWrite(pwm_channel_, duty_cycle);  
        // ledcWrite(pwm_channel_, PWM_MAX - speed);
    } 
    else {
        // Set direction to reverse
        digitalWrite(dir_pin_, LOW);
        int duty_cycle = constrain(-speed, PWM_REV_MIN, PWM_MAX);  // min power needed for reverse motion adjust PWM_REV_MIN in jupiter_config
        // Dont Invert the pwm duty cycle
        ledcWrite(pwm_channel_, duty_cycle);
        // ledcWrite(pwm_channel_, -speed); 
    }
}


// Helper function to get the maximum PWM value based on resolution
// int Motor::max_pwm_value() const {
//     return (1 << pwm_resolution_) - 1;  // e.g., 8-bit resolution -> 255
// }
