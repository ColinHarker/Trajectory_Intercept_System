#ifndef PORT_H
#define PORT_H
#include "Linker.hpp"
#include <sstream>

[[nodiscard]] static bool checkConnection() {
    while (1) {
        std::cout << "\n\nSearching...\n";


        const std::unique_ptr<SerialPort> arduino = std::make_unique<SerialPort>(constants::kPortName);

        while (!arduino->isConnected()) {
            Sleep(300);
            std::cout << ".";
        }

        //Checking if arduino is connected or not
        if (arduino->isConnected()) {
            std::cout << std::endl << "Connection established at port " << constants::kPortName << std::endl;
            return true;
        }
        return false;
    }
}

[[nodiscard]] static std::string convertPointData(const cv::Point& p) {
    
    const int x = p.x;
    const int y = p.y;
    std::ostringstream oss;
    oss << x << ", " << y;
    const std::string output_string = oss.str();

    return output_string;
}

static void sendData(const cv::Point& p) {

    const std::string point_string = convertPointData(p);

    std::unique_ptr<SerialPort> arduino = std::make_unique<SerialPort>(constants::kPortName);

    if (!arduino->writeSerialPort(point_string.c_str(), constants::kDataLength)) {
        std::cout << "Error writing to port: " << constants::kPortName << std::endl;
    }
}

#endif // !PORT_H

