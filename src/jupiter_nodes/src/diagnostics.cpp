#include <iostream>  // Like "import sys" - lets us print to the screen
#include <fstream>   // Lets us read files (to get battery/temp data)
#include <string>    // For handling text
#include <unistd.h>  // For usleep (making the robot wait)

int main() {
    std::cout << "--- Jupiter Hardware Monitor Starting ---" << std::endl;

    // This is our "While True" loop
    while (true) {
        // On Jetson, this file contains the CPU temperature in millidegrees
        std::ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
        std::string rawTemp;

        if (tempFile >> rawTemp) {
            // Convert millidegrees to Celsius (e.g., 45000 -> 45.0)
            float celsius = std::stof(rawTemp) / 1000.0;
            std::cout << "Thor Core Temp: " << celsius << "°C" << std::endl;
        }

        tempFile.close();

        // Wait for 2 seconds so we don't spam the terminal
        // 2,000,000 microseconds = 2 seconds
        usleep(2000000); 
    }

    return 0;
}