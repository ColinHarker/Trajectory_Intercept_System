#include "Port.hpp"
#include "Library/SerialPort.hpp"
#include "constants.hpp"

void sendData(cv::Point p) {

    char point_string[] = "test";


    //convert point to degree data that can be sent to arduino



    SerialPort* arduino = new SerialPort(constants::kPortName);

    if (!arduino->writeSerialPort(point_string, constants::kDataLength)) {
        std::cout << "Error writing to port: " << constants::kPortName << std::endl;
    }

}

bool checkConnection() {
    while (1) {
        std::cout << "\n\nSearching...\n";

        SerialPort* arduino = new SerialPort(constants::kPortName);

        while (!arduino->isConnected()) {
            Sleep(300);
            std::cout << ".";
            arduino = new SerialPort(constants::kPortName);
        }

        //Checking if arduino is connected or not
        if (arduino->isConnected()) {
            std::cout << std::endl << "Connection established at port " << constants::kPortName << std::endl;
            return true;
        }
        return false;
    }
}
