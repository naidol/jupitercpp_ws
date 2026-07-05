//#######################################################################################################
// Name:             imu_bno055.h
// Purpose:          Bosch IMU bno055 module setup and read sensor data
// Description:      to be used with esp32 firmware and micro-ros
// Related Files:    uses the onboard oled (oled_ssd1306.h) to display imu data     
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include <sensor_msgs/msg/imu.h>

// Oled display header
#include "oled_ssd1306.h"


// Initial routine to Set Up the IMU sensor.
void setup_imu(sensor_msgs__msg__Imu* imu_msg);

// Routine called by Timer Callback in Main.cpp to provide imu data
void get_imu_data(sensor_msgs__msg__Imu* imu_msg);