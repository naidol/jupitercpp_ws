#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>

class Kinematics {
public:
    struct Velocities {
        float linear_x;  // Linear velocity in the x direction (forward)
        float linear_y;  // Linear velocity in the y direction (sideways/strafing)
        float angular_z; // Angular velocity around the z axis (yaw)
    };

    struct WheelSpeeds {
        float motor1;   // Linear velocity of the front-left wheel (m/s)
        float motor2;   // Linear velocity of the front-right wheel (m/s)
        float motor3;   // Linear velocity of the back-left wheel (m/s)
        float motor4;   // Linear velocity of the back-right wheel (m/s)
    };

    struct MotorRPM {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    // Added wheel_base to the constructor for Mecanum geometry
    Kinematics(float wheel_radius, float wheel_separation, float wheel_base);

    // Added linear_velocity_y to the calculation signatures
    WheelSpeeds computeWheelSpeeds(float linear_velocity_x, float linear_velocity_y, float angular_velocity_z);
    MotorRPM calculateRPM(float linear_velocity_x, float linear_velocity_y, float angular_velocity_z);

    Velocities getVelocities(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm);
    // Diff-drive (2-wheel) odometry: vx = (L+R)/2, wz = (R-L)/wheel_separation
    Velocities getDiffVelocities(float left_rpm, float right_rpm);

private:
    float wheel_radius_;      // Radius of the wheel (meters)
    float wheel_separation_;  // Distance between the left and right wheels (meters)
    float wheel_base_;        // Distance between the front and rear axles (meters)
};

#endif // KINEMATICS_H