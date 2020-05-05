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

#include <thread>
#include <future>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "Library/SerialPort.hpp"

constexpr int kDataLength = 20;
constexpr int kNumCalcFrames = 20;
constexpr int kNumPredictedFrames = 15;
constexpr char kPortName[] = "\\\\.\\COM20";


bool checkConnection();
void sendData(cv::Point p);
int calculateTrajectory(int p[]);

int main(int argc, char** argv)
{
    std::string filename = "test3.mov";
    cv::VideoCapture capture(filename);
    //--for webcam--
    //VideoCapture Capture;
    //capture.open(0);
    cv::Mat frame, frame_HSV;
    int frame_data_x[kNumCalcFrames];
    int frame_data_y[kNumCalcFrames];
    int predicted_x;
    int predicted_y;
    int frame_count = 0;
    cv::Point predicted_point;

    if (!capture.isOpened())
        throw "Error when reading mov";
    if (!checkConnection) {
        std::cout << "Error: Cannot establish connection to port: " << kPortName << std::endl;
        return 1;
    }
    cv::namedWindow("w", 1);
    for (;;){
        capture >> frame;
        if (frame.empty())
            break;

        //convert to gray
        cv::cvtColor(frame, frame_HSV, cv::COLOR_RGB2HSV);

        //GaussianBlur(frame_grey, frame_grey, Size(9, 9), 1, 1);
        cv::blur(frame_HSV, frame_HSV, cv::Size(1, 1));

        //define threshhold around desired color
        cv::Scalar lower_bound = cv::Scalar(25, 52, 30);
        cv::Scalar upper_bound = cv::Scalar(80, 210, 150);
        cv::Mat thresh_frame;

        //Convert desired object to white, everything else black
        cv::inRange(frame_HSV, lower_bound, upper_bound, thresh_frame);

        //calculate center of body
        cv::Moments m = moments(thresh_frame, false);
        cv::Point com(m.m10 / m.m00, m.m01 / m.m00);

        
        

        //if the coords are on screen, display red cross
        if (!(com.x < 0 || com.y < 0)) {
            cv::Scalar color = cv::Scalar(0, 0, 255);
            cv::drawMarker(frame, com, color, cv::MARKER_CROSS, 25, 2);

            while (frame_count < 20) {          
                frame_data_x[frame_count] = com.x;
                frame_data_y[frame_count] = com.y;
                frame_count++;
            }
            if (frame_count == 20) {
                auto future_x = std::async(calculateTrajectory, frame_data_x);
                auto future_y = std::async(calculateTrajectory, frame_data_y);
                predicted_x = future_x.get;
                predicted_y = future_y.get;

                predicted_point = { predicted_x, predicted_y };
                sendData(predicted_point);
                
                frame_count++;
            }
            cv::drawMarker(frame, predicted_point, color, cv::MARKER_DIAMOND, 20, 2);
        }

        //display coord to terminal
        std::cout << com << std::endl;


        //display image
        cv::imshow("thresh", thresh_frame);
        cv::imshow("hsv", frame_HSV);
        cv::imshow("w", frame);


        cv::waitKey(15);

    }



    cv::waitKey(0); // key press to close window
    return 0;
}

void sendData(cv::Point p) {
    
    char point_string[] = "test";
    

    //convert point to degree data that can be sent to arduino
    
    
    
    SerialPort* arduino = new SerialPort(kPortName);

    if (!arduino->writeSerialPort(point_string, kDataLength)) {
        std::cout << "Error writing to port: " << kPortName << std::endl;
    }

}

bool checkConnection() {
    while (1) {
        std::cout << "Searching...";

        SerialPort* arduino = new SerialPort(kPortName);

        while (!arduino->isConnected()) {
            Sleep(100);
            std::cout << ".";
            arduino = new SerialPort(kPortName);
        }

        //Checking if arduino is connected or not
        if (arduino->isConnected()) {
            std::cout << std::endl << "Connection established at port " << kPortName << std::endl;
            return true;
        }
        return false;
    }
}

int calculateTrajectory(int p[]){

    
    int p_position = p[1], p_velocity = 0, p_s_velocity = 0, p_acceleration = 0, p_s_acceleration;
    int c_position = p[0], c_velocity = 0, c_s_velocity = 0, c_acceleration = 0, c_s_acceleration;
    int f_position[20];
    

    for (int i = 0; i < kNumCalcFrames; i++) {

        p_velocity = c_velocity;
        p_acceleration = c_acceleration;
        p_s_velocity = c_s_velocity;

        // +Velocity
        if (c_position >= p_position) {
            c_velocity = c_position - p_position;
            c_s_velocity = 1;
        }
        // -Velocity
        if (c_position < p_position) {
            c_velocity = p_position - c_position;
            c_s_velocity = 0;
        }
        // Accelerating
        if (c_velocity >= p_velocity) {
            c_acceleration = c_velocity - p_velocity;
            c_s_acceleration = c_s_velocity;
        }
        // Decelerating
        if (c_velocity < p_velocity) {
            c_acceleration = p_velocity - c_velocity;
            c_s_acceleration = c_s_velocity ^ 1;
        }
        // Inflection Point
        if ((c_s_velocity ^ p_s_velocity) == 1) {
            c_acceleration = c_position + p_position;
            c_s_acceleration = c_s_velocity;
        }
    }
    //Next, the path is predicted_x by incrementally adding the acceleration to the velocity
    //then this sum to the position

    f_position[0] = p_position;
    f_position[1] = c_position;

    for (int i = 1; i < kNumPredictedFrames; i++) {
        if (c_s_velocity != 0) {
            if (c_s_acceleration != 0) {
                f_position[i] = f_position[i - 1] + c_velocity + c_acceleration;
            }
            if (c_s_acceleration == 0) {
                f_position[i] = f_position[i - 1] + c_velocity - c_acceleration;
            }
        }
        if (c_s_velocity == 0) {
            if (c_s_acceleration != 0) {
                f_position[i] = f_position[i - 1] - c_velocity + c_acceleration;
            }
            if (c_s_acceleration == 0) {
                f_position[i] = f_position[i - 1] - c_velocity - c_acceleration;
            }
        }
    }

    return f_position[kNumPredictedFrames - 1];
}

























