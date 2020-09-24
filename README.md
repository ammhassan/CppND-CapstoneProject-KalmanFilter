# CPPND: Capstone Project Repo - Kalman Filter

This is the project repository for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213).


## Dependencies for Running Locally
* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Eigen >= 3.2.10
  * All OSes: [click here for installation instructions](http://eigen.tuxfamily.org/dox/GettingStarted.html)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./kalmanTest`.

## Project Description

### Background
The objective of this project is to implement Kalman filter in C++ and test it on an aircraft simulation dataset.
Kalman filter is used to estimate a dynamic system's states from noisy measurments. More details on Kalman filter can be found [here](https://books.google.com/books?hl=en&lr=&id=UiMVoP_7TZkC&oi=fnd&pg=PR3&dq=+Dan+Simon+-+Optimal+State+Estimation_+Kalman,+H+Infinity,+and+Nonlinear+Approaches+&ots=L1Gf1DLgtg&sig=SzaCp18NVabM3_y-_tNOuYEz2Bc#v=onepage&q=Dan%20Simon%20-%20Optimal%20State%20Estimation_%20Kalman%2C%20H%20Infinity%2C%20and%20Nonlinear%20Approaches&f=false) and [here](http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

The input to this filter is a discrete-time linear system on this form <br />
x(k+1) = A * x(k) + B * u(k) + w(k) <br />
y(k+1) = C * x(k+1) + v(k+1) <br />
where x is the state vector, A is the dynamics matrix, B is control input matrix, w is the process noise vector, y is the output vector, C is the output matrix, v is the measurment noise vector, and k is the time step index.

### Project Structure
This project is composed of the follwing elements:
1. The kalman filter class which is defined in `KalmanFilter.h` and `KalmanFilter.cpp`.
2. A few helper functions defined in `helperFunc.h` and `helperFunc.cpp`.
3. A test file (`KalmanFilter_test.cpp`) which has the main function and test the filter on an aircaft simulation dataset.

### Expected Behavior
The expected output from this program looks like this: <br />
`Dataset has been loaded` <br />
`Kalman filter object has been constructed.` <br />
`Kalman filter object has been initialized.` <br />
`Processing dataset...` <br />
`Finished` <br />
`Mean square error is: 8.06016e-07` <br />

## Targeted Rubric Points
1. Loops, Functions, I/O: The project demonstrates an understanding of C++ functions and control structures. <br />
See `helperFunc.h`,&nbsp; `helperFunc.cpp` and `KalmanFilter_test.cpp`. <br />
2. Loops, Functions, I/O: The project reads data from a file and process the data, or the program writes data to a file. <br />
See `loadDataSet` function in `helperFunc.cpp`. <br />
3. Object Oriented Programming: The project uses Object Oriented Programming techniques. <br />
See class `KalmanFilter` in `KalmanFilter.h` and `KalmanFilter.cpp`. <br />
4. Object Oriented Programming: Classes use appropriate access specifiers for class members. <br />
See `KalmanFilter.cpp` line 16--29. <br />
5. Object Oriented Programming: Class constructors utilize member initialization lists. <br />
See `KalmanFilter.cpp` line 4--12. <br />
6. Object Oriented Programming: Classes abstract implementation details from their interfaces. <br />
See `KalmanFilter.cpp` line 31--54. <br />
7. Memory Management: The project makes use of references in function declarations. <br />
See `KalmanFilter.h` line 10--18.
