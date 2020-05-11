#include "Port.hpp"
#include "Library/SerialPort.hpp"
#include "constants.hpp"
#include <sstream>

void sendData(cv::Point p) {



    char* point_string = convertPointData(p);

    std::unique_ptr<SerialPort> arduino = std::make_unique<SerialPort>(constants::kPortName);

    if (!arduino->writeSerialPort(point_string, constants::kDataLength)) {
        std::cout << "Error writing to port: " << constants::kPortName << std::endl;
    }
    delete[] point_string;
}

bool checkConnection() {
    while (1) {
        std::cout << "\n\nSearching...\n";

        
        std::unique_ptr<SerialPort> arduino = std::make_unique<SerialPort>(constants::kPortName);

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

char* convertPointData(cv::Point p) {
    int x = p.x;
    int y = p.y;
    std::ostringstream oss;
    oss << x << ", " << y;
    std::string output_string = oss.str();
    
    size_t length = output_string.length();

    char* output_char_arr = new char[length + 1];

    strcpy_s(output_char_arr, length + 1, output_string.c_str());

    return output_char_arr;
}
