#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter
{
    public:
    // constructor
    KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q,
                const Eigen::MatrixXd &R, const Eigen::MatrixXd &P, double dt);

    // filter initializer
    void init(const Eigen::VectorXd &x0);

    // filter update function, y: output measurement vector, u: control input vector 
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u);

    // getters
    Eigen::VectorXd getState();
    Eigen::VectorXd getEstimatedOutput();
    double getTime();           

    private:
    // time and sampleTime
    double time;
    double sampleTime;

    // dimensions of the dynamic system
    int nStates;
    int nInputs;
    int nOutputs;

    // dynamic system matrices
    Eigen::MatrixXd A; // dynamics matrix
    Eigen::MatrixXd B; // control input matrix
    Eigen::MatrixXd C; // output matrix
    Eigen::MatrixXd Q; // process noise covariance matrix
    Eigen::MatrixXd R; // measurment noise covariance matrix

    // filter matrices
    Eigen::MatrixXd P; // estimate error covariance matrix
    Eigen::MatrixXd P0; // initial value for P
    Eigen::MatrixXd K; // Kalman gain matrix
    Eigen::MatrixXd I; // identity matrix

    // state estimate at current and next time steps
    Eigen::VectorXd xHat;
    Eigen::VectorXd xhatNext;
};


#endif