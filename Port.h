#ifndef PORT_H
#define PORT_H
#pragma once
#include <opencv2\core\types.hpp>

bool checkConnection();
void sendData(cv::Point p);

#endif // !PORT_H

