#ifndef PORT_H
#define PORT_H
#include <opencv2\core\types.hpp>

bool checkConnection();
void sendData(cv::Point p);
char* convertPointData(cv::Point p);

#endif // !PORT_H

