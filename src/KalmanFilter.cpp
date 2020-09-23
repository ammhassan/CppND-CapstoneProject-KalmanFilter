#include <iostream>
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                           const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q,
                           const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
                           double dt) : A(A), B(B), C(C), Q(Q), R(R),
                           P0(P), sampleTime(dt), nStates(A.rows()),
                           nInputs(B.cols()), nOutputs(C.rows()),
                           xHat(nStates), xhatNext(nStates),
                           I(nStates, nStates)
{
    std::cout << "Kalman filter object has been constructed." << std::endl;
}

Eigen::VectorXd KalmanFilter::getState()
{
    return xHat;
}

Eigen::VectorXd KalmanFilter::getEstimatedOutput()
{
    return C * xHat;
}

double KalmanFilter::getTime()
{
    return time;
}

void KalmanFilter::init(const Eigen::VectorXd &x0)
{
    I.setIdentity();
    xHat = x0;
    P = P0;
    time = 0.0;
    std::cout << "Kalman filter object has been initialized." << std::endl;
}

void KalmanFilter::update(const Eigen::VectorXd &y, const Eigen::VectorXd &u)
{
    // prediction step
    xhatNext = A * xHat + B * u;
    P = A * P * A.transpose() + Q;
    std::cout << "P now is: " << P << std::endl;

    // correction step
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    std::cout << "K now is: " << K << std::endl;
    xhatNext += K * (y - C * xhatNext);
    P =  (I - K * C) * P;

    // prepare for next time step
    xHat = xhatNext;
    time += sampleTime;
}

