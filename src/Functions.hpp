#ifndef FUNCTIONS_H
#define FUNCTIONS_H 

#include <thread>
#include <future>
#include <iostream>
#include <conio.h>

#include "Linker.hpp"

    template<int T>
    static void calculateFrames(int&, cv::Point&, std::array<int, T>&, std::array<int, T>&);
    static cv::Mat filterFrame();
    static bool coordinatesAreVisible(cv::Point&);
    static void display(cv::Point&);

    cv::Mat frame, frame_HSV;
    cv::Point predicted_point;
    size_t frame_count = 0;
    bool show_marker = false, running = true;
    int index = 0;

static void run(cv::VideoCapture& capture) {

    cv::namedWindow("w", 1);
    std::array<int, constants::kNumCalcFrames> frame_data_x;
    std::array<int, constants::kNumCalcFrames> frame_data_y;

    while (running) {

        capture >> frame;
        if (frame.empty())
            break;

        cv::Mat thresh_frame = filterFrame();

        //calculate center of body
        cv::Moments m = moments(thresh_frame, false);
        cv::Point com(m.m10 / m.m00, m.m01 / m.m00);


        //if the coords are on screen, display red cross
        if (coordinatesAreVisible(com)) {

            cv::Scalar color = cv::Scalar(0, 0, 255);
            cv::drawMarker(frame, com, color, cv::MARKER_CROSS, 25, 2);

            calculateFrames(index, com, frame_data_x, frame_data_y);

            //display the predicted point
            if (show_marker) cv::drawMarker(frame, predicted_point, color, cv::MARKER_DIAMOND, 20, 2);

            //turn marker off after frame has passed the predicted frame
            if (frame_count == constants::kStartFrame + constants::kNumCalcFrames + constants::kNumPredictedFrames + 1) show_marker = false;
        }

        display(com);

        cv::waitKey(0);

        //if escape is pressed, exit
        if (_getch() == 27)
            running = false;
    }
}

static cv::Mat filterFrame() {
        
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
    
        return thresh_frame;
}

static bool coordinatesAreVisible(cv::Point& com) {
    return !(com.x < 0 || com.y < 0);
}

static void display(cv::Point& com) {

        std::cout << com << std::endl;

        //display image
        //cv::imshow("thresh", thresh_frame);
        //cv::imshow("hsv", frame_HSV);
        cv::imshow("w", frame);
}

template<int T>
static void calculateFrames(int& index, cv::Point& com, std::array<int, T>& frame_data_x, std::array<int, T>& frame_data_y) {

     //if the current frame is within the window for aquiring trajectory data 
            if (frame_count <= constants::kStartFrame + constants::kNumCalcFrames && frame_count > constants::kStartFrame) {

                frame_data_x[index] = com.x;
                frame_data_y[index] = com.y;

                index++;
                frame_count++;

                //once we leave the window to calculate data
            }
            else if (frame_count == constants::kStartFrame + constants::kNumCalcFrames + 1) {

                //send x and y coord data to the calculate trajectory function
                auto future_x = std::async([&frame_data_x]() { return calculateTrajectory(frame_data_x); });
                auto future_y = std::async([&frame_data_y]() { return calculateTrajectory(frame_data_y); });
                const int predicted_x = future_x.get();
                const int predicted_y = future_y.get();

                predicted_point = { predicted_x, predicted_y };

                //sending the predicted coord of trajectory
                sendData(predicted_point);

                show_marker = true;
                frame_count++;
            }
            else {
                frame_count++;
            }
}

#endif // !FUNCTIONS_H
