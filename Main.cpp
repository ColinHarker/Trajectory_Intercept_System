/*
* Author: Colin Harker
* Computer vision program to track and calculate trajectory of object in order to predict future position.
* Designed to interface with Arduino and control servos.
* _STATUS_: WIP
*/



#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "Library/SerialPort.hpp"


int main(int argc, char** argv)
{
    std::string filename = "test3.mov";
    cv::VideoCapture capture(filename);
    //--for webcam--
    //VideoCapture Capture;
    //capture.open(0);
    cv::Mat frame, frame_HSV;

    if (!capture.isOpened())
        throw "Error when reading mov";

    cv::namedWindow("w", 1);
    for (;;)
    {
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
            drawMarker(frame, com, color, cv::MARKER_CROSS, 25, 2);
        }
        
        //display coord to terminal
        std::cout << com << std::endl;

    
        //display image
        imshow("thresh", thresh_frame);
        imshow("hsv", frame_HSV);
        imshow("w", frame);



        cv::waitKey(15);


        //after n number of frames, use marker locations to calculate predicted trajectory k frames in the future


        //once trajectory is predicted, save coordinates to a Point


        //convert point to radian data that can be sent to arduino


        //send data to arduino




    }
    




    cv::waitKey(0); // key press to close window
    return 0;
}

