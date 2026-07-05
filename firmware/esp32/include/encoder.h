#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int enc_pinA, int enc_pinB, int ticks_per_revolution);

    // Initialize the encoder
    void begin();

    // Returns the current count (ticks)
    long getCount();

    // Returns the wheel RPM based on encoder counts
    float getRPM();

    // Reset the encoder count
    void reset();

    // ISR functions for handling encoder pulses (one ISR per encoder pin)
    static void IRAM_ATTR handleInterruptA_0();
    static void IRAM_ATTR handleInterruptB_0();
    static void IRAM_ATTR handleInterruptA_1();
    static void IRAM_ATTR handleInterruptB_1();
    static void IRAM_ATTR handleInterruptA_2();
    static void IRAM_ATTR handleInterruptB_2();
    static void IRAM_ATTR handleInterruptA_3();
    static void IRAM_ATTR handleInterruptB_3();

private:
    int enc_pinA_;
    int enc_pinB_;
    int ticks_per_revolution_;
    
    volatile long count_;  // Encoder count
    long last_count_;      // Last count for calculating RPM
    unsigned long last_time_;  // Last time in milliseconds

    static Encoder* instances_[4];  // Array to store pointers to Encoder instances

    float calculateRPM();  // Helper function to calculate RPM

    // Internal methods to handle the interrupts for this instance
    void handleA();
    void handleB();
};

#endif // ENCODER_H
