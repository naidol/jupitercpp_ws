#include "kinematics.h"

// Constructor updated to initialize wheel base
Kinematics::Kinematics(float wheel_radius, float wheel_separation, float wheel_base)
    : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation), wheel_base_(wheel_base) {}

// Full Mecanum Inverse Kinematics (Cmd_Vel -> Wheel Speeds)
Kinematics::WheelSpeeds Kinematics::computeWheelSpeeds(float linear_velocity_x, float linear_velocity_y, float angular_velocity_z) {
    WheelSpeeds speeds;

    // L is half the wheelbase, W is half the track width
    float L = wheel_base_ / 2.0;
    float W = wheel_separation_ / 2.0;

    // Mecanum wheel speed mixing
    speeds.motor1 = linear_velocity_x - linear_velocity_y - (angular_velocity_z * (L + W)); // Front-Left
    speeds.motor2 = linear_velocity_x + linear_velocity_y + (angular_velocity_z * (L + W)); // Front-Right
    speeds.motor3 = linear_velocity_x + linear_velocity_y - (angular_velocity_z * (L + W)); // Rear-Left
    speeds.motor4 = linear_velocity_x - linear_velocity_y + (angular_velocity_z * (L + W)); // Rear-Right

    return speeds;
}

Kinematics::MotorRPM Kinematics::calculateRPM(float linear_velocity_x, float linear_velocity_y, float angular_velocity_z) {
    MotorRPM rpm;

    WheelSpeeds speeds = computeWheelSpeeds(linear_velocity_x, linear_velocity_y, angular_velocity_z);

    // Convert the wheel speeds (m/s) to RPM
    rpm.motor1 = (speeds.motor1 / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.motor2 = (speeds.motor2 / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.motor3 = (speeds.motor3 / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.motor4 = (speeds.motor4 / (2.0 * M_PI * wheel_radius_)) * 60.0;

    return rpm;
}

// Full Mecanum Forward Kinematics (Wheel RPM -> Odom Velocities)
Kinematics::Velocities Kinematics::getVelocities(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm) {
    
    float front_left_velocity = (front_left_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float front_right_velocity = (front_right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float back_left_velocity = (back_left_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float back_right_velocity = (back_right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;

    float L = wheel_base_ / 2.0;
    float W = wheel_separation_ / 2.0;

    // Mecanum inverse matrices for Odom
    float linear_velocity_x = (front_left_velocity + front_right_velocity + back_left_velocity + back_right_velocity) / 4.0f;
    float linear_velocity_y = (-front_left_velocity + front_right_velocity + back_left_velocity - back_right_velocity) / 4.0f;
    float angular_velocity_z = (-front_left_velocity + front_right_velocity - back_left_velocity + back_right_velocity) / (4.0f * (L + W));

    Velocities velocities;
    velocities.linear_x = linear_velocity_x;
    velocities.linear_y = linear_velocity_y; 
    velocities.angular_z = angular_velocity_z;
    
    return velocities;
}

Kinematics::Velocities Kinematics::getDiffVelocities(float left_rpm, float right_rpm) {
    float left_vel  = (left_rpm  * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float right_vel = (right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;

    Velocities velocities;
    velocities.linear_x  = (left_vel + right_vel) / 2.0f;
    velocities.linear_y  = 0.0f;
    velocities.angular_z = (right_vel - left_vel) / wheel_separation_;
    return velocities;
}