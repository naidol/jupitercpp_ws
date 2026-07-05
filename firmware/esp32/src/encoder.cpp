#include "encoder.h"

// Initialize the static array of pointers to Encoder instances
Encoder* Encoder::instances_[4] = {nullptr, nullptr, nullptr, nullptr};

Encoder::Encoder(int enc_pinA, int enc_pinB, int ticks_per_revolution)
    : enc_pinA_(enc_pinA), enc_pinB_(enc_pinB), ticks_per_revolution_(ticks_per_revolution),
      count_(0), last_count_(0), last_time_(0) {
    // Assign the correct instance to the array based on available slots
    for (int i = 0; i < 4; ++i) {
        if (instances_[i] == nullptr) {
            instances_[i] = this;
            break;
        }
    }
}

void Encoder::begin() {
    pinMode(enc_pinA_, INPUT);
    pinMode(enc_pinB_, INPUT);

    // Attach interrupts to both encoder pins based on the encoder's instance index
    if (instances_[0] == this) {
        attachInterrupt(digitalPinToInterrupt(enc_pinA_), handleInterruptA_0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(enc_pinB_), handleInterruptB_0, CHANGE);
    } else if (instances_[1] == this) {
        attachInterrupt(digitalPinToInterrupt(enc_pinA_), handleInterruptA_1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(enc_pinB_), handleInterruptB_1, CHANGE);
    } else if (instances_[2] == this) {
        attachInterrupt(digitalPinToInterrupt(enc_pinA_), handleInterruptA_2, CHANGE);
        attachInterrupt(digitalPinToInterrupt(enc_pinB_), handleInterruptB_2, CHANGE);
    } else if (instances_[3] == this) {
        attachInterrupt(digitalPinToInterrupt(enc_pinA_), handleInterruptA_3, CHANGE);
        attachInterrupt(digitalPinToInterrupt(enc_pinB_), handleInterruptB_3, CHANGE);
    }

    // Initialize timing for RPM calculation
    last_time_ = millis();
}

long Encoder::getCount() {
    return count_;
}

float Encoder::getRPM() {
    unsigned long current_time = millis();
    unsigned long time_elapsed = current_time - last_time_;

    // Calculate RPM if time has passed
    if (time_elapsed > 100) {  // At least 100ms has passed
        float rpm = calculateRPM();
        last_time_ = current_time;
        last_count_ = count_;  // Update the last count for the next RPM calculation
        return rpm;
    }
    return 0.0;  // No significant time passed to calculate RPM
}

void Encoder::reset() {
    count_ = 0;
    last_count_ = 0;
}

// Static ISR functions for the four encoders
void IRAM_ATTR Encoder::handleInterruptA_0() {
    if (instances_[0]) {
        instances_[0]->handleA();
    }
}

void IRAM_ATTR Encoder::handleInterruptB_0() {
    if (instances_[0]) {
        instances_[0]->handleB();
    }
}

void IRAM_ATTR Encoder::handleInterruptA_1() {
    if (instances_[1]) {
        instances_[1]->handleA();
    }
}

void IRAM_ATTR Encoder::handleInterruptB_1() {
    if (instances_[1]) {
        instances_[1]->handleB();
    }
}

void IRAM_ATTR Encoder::handleInterruptA_2() {
    if (instances_[2]) {
        instances_[2]->handleA();
    }
}

void IRAM_ATTR Encoder::handleInterruptB_2() {
    if (instances_[2]) {
        instances_[2]->handleB();
    }
}

void IRAM_ATTR Encoder::handleInterruptA_3() {
    if (instances_[3]) {
        instances_[3]->handleA();
    }
}

void IRAM_ATTR Encoder::handleInterruptB_3() {
    if (instances_[3]) {
        instances_[3]->handleB();
    }
}

void Encoder::handleA() {
    // A has changed, check B to determine the direction
    if (digitalRead(enc_pinA_) == digitalRead(enc_pinB_)) {
        count_++;  // Forward direction
    } else {
        count_--;  // Reverse direction
    }
}

void Encoder::handleB() {
    // B has changed, check A to determine the direction
    if (digitalRead(enc_pinA_) != digitalRead(enc_pinB_)) {
        count_++;  // Forward direction
    } else {
        count_--;  // Reverse direction
    }
}

float Encoder::calculateRPM() {
    // Calculate the change in counts
    long delta_count = count_ - last_count_;

    // Calculate RPM: (delta_count / ticks_per_revolution) * (60000 / time_elapsed)
    unsigned long time_elapsed = millis() - last_time_;
    return (delta_count / static_cast<float>(ticks_per_revolution_)) * (60000.0 / time_elapsed);
}


// Notes from GPT for the encoders
// My Question: the Encoder::handleA() and Encoder::handleB() routines appear to be cancelling each other out. can you check if these routines are correct?
// ChatGPT said:
// ChatGPT

// The issue where Encoder::handleA() and Encoder::handleB() seem to be canceling each other out could be due to incorrect logic for 
// determining the direction of rotation based on the encoder signals.

// For a quadrature encoder, the direction of rotation is typically determined by the relative timing of the signals 
// on the enc_pinA and enc_pinB lines. The logic in the previous implementation might not be correctly handling the quadrature signals. 
// Let’s clarify the logic and correct it.
// Correct Logic for Quadrature Encoders:

//     Quadrature Encoding: In a quadrature encoder, two channels (A and B) generate square wave signals that are 90 degrees out of phase with each other. 
//     The direction of rotation is determined by the order in which the signals change.
//         Clockwise Rotation: When channel A leads channel B (A changes state before B).
//         Counterclockwise Rotation: When channel B leads channel A (B changes state before A).

// We can update the handleA() and handleB() routines to correctly account for this by checking the state of both enc_pinA and enc_pinB at every transition.
// Updated handleA() and handleB() Routines:

//     handleA(): This ISR is triggered when enc_pinA changes state. To determine the direction, check the current state of enc_pinB.
//     handleB(): This ISR is triggered when enc_pinB changes state. To determine the direction, check the current state of enc_pinA.

// Here’s the corrected implementation:
// Updated encoder.cpp