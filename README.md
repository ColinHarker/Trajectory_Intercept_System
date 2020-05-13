# Trajectory_Intercept_System
C++ and Arduino project using OpenCV


### Install

This project files requires the installation of **OpenCV**:

- [OpenCV](https://opencv.org/)

Following are some links to help install OpenCV on mac, windows and linux:


[OpenCV](https://github.com/opencv/opencv) - [Mac](https://www.learnopencv.com/install-opencv3-on-macos/) | [Windows](https://www.learnopencv.com/install-opencv3-on-windows/) | [Ubuntu](https://www.learnopencv.com/install-opencv3-on-ubuntu/)



### Run

```c++
g++ {file_name}
```  

### Quick Start
Include the following header

```c++
#include "Linker.hpp"
```

In the 'constants.hpp' file this is where you specify; 

* How many frames of data will be calculated

  ```c++
  constexpr int kNumCalcFrames = ;
  ```


* At what point does the frame calculation start

  ```c++
  constexpr int kStartFrame = ;
  ```
* How many frames in the future do you want to predict
  
  ```c++
  constexpr int kNumPredictedFrames = ;
  ```

* If you are sending data out of a port, you include the port name

  ```c++
  constexpr char kPortName[] = "";
  ```

### Calculate Future Trajectory

1. Create two arrays, for "X" and "Y" coordinates in all frames up until a given point.

```c++
int frame_data_x[constants::kNumCalcFrames];
int frame_data_y[constants::kNumCalcFrames];
```

2. Send data to the calculate trajectory function asynchronously, the predicted point needs to be calculated as fast as possible.

```c++
auto future_x = std::async(calculateTrajectory, frame_data_x);
auto future_y = std::async(calculateTrajectory, frame_data_y);
```




## About
