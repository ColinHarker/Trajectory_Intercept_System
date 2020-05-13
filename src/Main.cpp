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

#include "Linker.hpp"


int main(int argc, char** argv)
{

    std::string filename = "test3.mov";
    cv::VideoCapture capture(filename);
    //--for webcam--
    //VideoCapture Capture;
    //capture.open(0);


    cv::Mat frame, frame_HSV;

    int frame_data_x[constants::kNumCalcFrames];
    int frame_data_y[constants::kNumCalcFrames];
    int predicted_x;
    int predicted_y;
    int frame_count = 0;
    int index = 0;

    cv::Point predicted_point;

    if (!capture.isOpened())
        throw "Error when reading mov";
    if (!checkConnection()) {
        std::cout << "Error: Cannot establish connection to port: " << constants::kPortName << std::endl;
        return 1;
    }


    bool p = false;

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
            
           
            if(frame_count <= constants::kStartFrame + constants::kNumCalcFrames && frame_count > constants::kStartFrame) {
               
                frame_data_x[index] = com.x;
                frame_data_y[index] = com.y;
                
                index++;
                frame_count++;

            }else if(frame_count == constants::kStartFrame + constants::kNumCalcFrames + 1) {
                
                auto future_x = std::async(calculateTrajectory, frame_data_x);
                auto future_y = std::async(calculateTrajectory, frame_data_y);
                predicted_x = future_x.get();
                predicted_y = future_y.get();

                predicted_point = { predicted_x, predicted_y };
                sendData(predicted_point);
               
                p = true;
                frame_count++;
            }else {
                frame_count++;
            }

            if (frame_count == constants::kStartFrame + constants::kNumCalcFrames + constants::kNumPredictedFrames + 1) p = false; //turn marker off after frame has passed the predicted frame
            if(p)cv::drawMarker(frame, predicted_point, color, cv::MARKER_DIAMOND, 20, 2);  //display the predicted point
        }

        //display coord to terminal
        std::cout << com << std::endl;


        //display image
        //cv::imshow("thresh", thresh_frame);
        //cv::imshow("hsv", frame_HSV);
        cv::imshow("w", frame);


        cv::waitKey(0);      
    }



    cv::waitKey(0); // key press to close window
    return 0;
}




