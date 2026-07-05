//#######################################################################################################
// Name:             imu_bno055.cpp
// Purpose:          Bosch IMU bno055 module setup and read sensor data
// Description:      to be used with esp32 firmware and micro-ros
// Related Files:    uses the onboard oled (oled_ssd1306.h) to display imu data     
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include "imu_bno055.h"
// BNO055 IMU pre-built OEM files
#include <Adafruit_BNO055.h>
#include <Adafruit_BusIO_Register.h>
#include <sensor_msgs/msg/imu.h>

// Added for setting up calibration save & restore
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

extern void display_oled_alert(const char* message);
// 1. The Safety Flag
bool trigger_imu_save = false;

// Define IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup_imu(sensor_msgs__msg__Imu* imu_msg) {
    // Initialize I2C communication
    // Wire.begin(21, 22);  <--- COMMENT THIS OUT (Moved to firmware.ino)

    // Initialize the BNO055 sensor
    if (!bno.begin()) {
        display_oled_alert("NO BNO055!");
        while (1);
    }

    // Set up the BNO055 sensor
    bno.setExtCrystalUse(true);

    // 2. RESTORE LOGIC (Happens safely before ROS connects)
    Preferences prefs; 
    prefs.begin("bno_cal", true); // Open in Read-Only mode
    adafruit_bno055_offsets_t calibData;
    
    if (prefs.isKey("offsets")) {
        prefs.getBytes("offsets", &calibData, sizeof(calibData));
        
        bno.setMode(OPERATION_MODE_CONFIG);
        delay(50);
        bno.setSensorOffsets(calibData);
        delay(50);
        bno.setMode(OPERATION_MODE_NDOF);
        delay(50);
        
        display_oled_alert("RESTORE:\nSUCCESS");
        delay(2000);
    } 
    else {
        display_oled_alert("RESTORE:\nNO DATA");
        delay(2000);
    }
    prefs.end(); // Close immediately

    // Setup imu_msg initial header and covariances
    imu_msg->header.frame_id.data = (char*)("imu_link");
    imu_msg->orientation_covariance[0] = -1; // Indicates orientation covariance is not provided
    imu_msg->angular_velocity_covariance[0] = -1; // Indicates angular velocity covariance is not provided
    imu_msg->linear_acceleration_covariance[0] = -1; // Indicates linear acceleration covariance is not provided
}

// 3. THE SAVE EXECUTION (Called by the main loop, not the callback)
// Retries silently every loop until G==3 && A==3 && M==3, then verifies the NVS write.
void perform_imu_save() {
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if (gyro != 3 || accel != 3 || mag != 3) {
        // Calibration not ready — keep flag set, retry next loop silently
        return;
    }

    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);

    Preferences prefs;
    prefs.begin("bno_cal", false); // Read/Write mode
    size_t written = prefs.putBytes("offsets", &newCalib, sizeof(newCalib));
    prefs.end();

    if (written == sizeof(newCalib)) {
        trigger_imu_save = false; // Clear only on verified success
        display_oled_alert("SAVE:\nSUCCESS");
        delay(2000);
    } else {
        // NVS write failed — keep flag set so it retries next loop
        display_oled_alert("SAVE:\nNVS ERR");
        delay(2000);
    }
}

// Get IMU data and update the &imu_msg and refresh the Oled display with latest IMU data 
void get_imu_data(sensor_msgs__msg__Imu* imu_msg) {
    // Get the BNO055 calibration status
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    // Read the 9DOF sensor data
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    /* Read the current temperature from BNO055 */
    int8_t temp = bno.getTemp();

    // Read the BNO055 Quaternion
    imu::Quaternion quat = bno.getQuat();

    // Fill the ROS2 message for IMU data
    // imu_msg->header.stamp.nanosec = (uint32_t)(millis() * 1e6);
    // imu_msg->header.stamp.sec = (uint32_t)(millis() / 1000);
    

    imu_msg->orientation.x = quat.x();
    imu_msg->orientation.y = quat.y();
    imu_msg->orientation.z = quat.z();
    imu_msg->orientation.w = quat.w(); 

    imu_msg->angular_velocity.x = angVelocityData.gyro.x;
    imu_msg->angular_velocity.y = angVelocityData.gyro.y;
    imu_msg->angular_velocity.z = angVelocityData.gyro.z;

    imu_msg->linear_acceleration.x = linearAccelData.acceleration.x;
    imu_msg->linear_acceleration.y = linearAccelData.acceleration.y;
    imu_msg->linear_acceleration.z = linearAccelData.acceleration.z;

    // Display IMU data on OLED
    display_oled_imu_data(temp, orientationData.acceleration.heading,
                          orientationData.acceleration.pitch,
                          orientationData.acceleration.roll,
                          gravityData.gyro.x,
                          gravityData.gyro.y,
                          gravityData.gyro.z,
                          system, gyro, accel, mag);
    
}