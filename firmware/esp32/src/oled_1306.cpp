//#######################################################################################################
// Name:             oled_ssd1306.cpp
// Purpose:          Module to setup the Adafruit SSD1306 128x64 display
// Description:      to be used with esp32 firmware and micro-ros
// Related Files:         
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include "oled_ssd1306.h"

// Instantiate the Oled object
Adafruit_SSD1306 oled_ssd1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup_oled_display() {
    // Initialize OLED display
    if (!oled_ssd1306.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    oled_ssd1306.display();
    delay(2000);
    oled_ssd1306.clearDisplay();
    oled_ssd1306.display();
}


void display_oled_imu_data(int8_t temp, float_t heading, float_t pitch, float_t roll, 
                            float_t gx, float_t gy, float_t gz,
                            uint8_t system, uint8_t gyro, uint8_t accel, uint8_t mag) {
    // Clear the Oled and set cursor to top left
    oled_ssd1306.clearDisplay();
    oled_ssd1306.setTextSize(1);
    oled_ssd1306.setTextColor(SSD1306_WHITE);
    oled_ssd1306.setCursor(0, 0);
    // print the temp data
    oled_ssd1306.print("Temperature C: "); oled_ssd1306.println(temp);
    // print the orientation and gyro data
    oled_ssd1306.print("oX: "); oled_ssd1306.print(heading, 1); oled_ssd1306.print(" | ");
    oled_ssd1306.print("oY: "); oled_ssd1306.println(pitch, 1);
    oled_ssd1306.print("oZ: "); oled_ssd1306.println(roll, 1);
    oled_ssd1306.print("gX: "); oled_ssd1306.print(gx, 1); oled_ssd1306.print(" | ");
    oled_ssd1306.print("gY: "); oled_ssd1306.println(gy, 1);
    oled_ssd1306.print("gZ: "); oled_ssd1306.println(gz, 1);
    // oled_ssd1306 the calibration status (Sys | Gyro | Accel | Magno)
    oled_ssd1306.println();
    oled_ssd1306.print("C: S="); oled_ssd1306.print(system); oled_ssd1306.print("|"); 
    oled_ssd1306.print("G="); oled_ssd1306.print(gyro); oled_ssd1306.print("|");
    oled_ssd1306.print("A="); oled_ssd1306.print(accel); oled_ssd1306.print("|");
    oled_ssd1306.print("M="); oled_ssd1306.println(mag);
    // Populate the Oled Display
    oled_ssd1306.display();
}

// Used to display IMU calib save/restore status
void display_oled_alert(const char* message) {
    oled_ssd1306.clearDisplay();
    oled_ssd1306.setTextSize(2); // Larger text for alerts
    oled_ssd1306.setTextColor(SSD1306_WHITE);
    oled_ssd1306.setCursor(0, 0);
    oled_ssd1306.println(message);
    oled_ssd1306.display();
}