/*
* Author: Colin Harker
* Computer vision program to track and calculate trajectory of object in order to predict future position.
* Designed to interface with Arduino and control servos.
* License: MIT
* _STATUS_: WIP
* 
* Citations:
*   1. Trajectory Algorithm Based on research paper
*       - "Real-time Trajectory Calculation and Prediction Using Neighborhood-Level Parallel Processing"
*       -  Written by Mahir Kabeer Gharzai, Dingyi Hong, Joseph A. Schmitz, Michael W. Hoffman, Sina Balkır
*       -  Published by: University of Nebraska–Lincoln Department of Electrical & Computer Engineering 
*
*   2. C++ Serial Port library
*       - Author: Manash Kumar Mandal
*       - Link: github.com/manashmandal/SerialPort
*/


#include "Linker.hpp"

#define FILENAME "test3.mov" 



int main(int argc, char** argv)
{
    cv::VideoCapture capture(FILENAME);
    //--for webcam--
    //VideoCapture Capture;
    //capture.open(0);

    if (!capture.isOpened())
        throw "Error when reading mov";
  /*
    if (!checkConnection()) {
        std::cout << "Error: Cannot establish connection to port: " << constants::kPortName << std::endl;
        return 1;
    }
    */
    run(capture);

    return 0;
}





