//#######################################################################################################
// Name:             oled_ssd1306.h
// Purpose:          Module to setup the Adafruit SSD1306 128x64 display
// Description:      to be used with esp32 firmware and micro-ros
// Related Files:         
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

// OLED 
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C // Define the I2C address for your display


// Routine to initialise & setup the Oled Display
void setup_oled_display();

// Routine called by imu_bno055.cpp to populate the Oled display with IMU data
void display_oled_imu_data(int8_t temp, float_t heading, float_t pitch, float_t roll, 
                            float_t gx, float_t gy, float_t gz,
                            uint8_t system, uint8_t gyro, uint8_t accel, uint8_t mag);

// Routine to provide IMU calibration save/restore status
void display_oled_alert(const char* message);

