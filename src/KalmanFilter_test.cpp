#include <iostream>
#include "KalmanFilter.h"
#include "helperFunc.h"

int main()
{
    // define the subject dynamic system
    const std::string dataSetPath{"../data/aircraft_pitch_dynamics_data.txt"};
    const int nStates = 2;
    const int nInputs = 1;
    const int nOutputs = 1;
    const double sampleTime = 0.01;

    Eigen::MatrixXd A(nStates, nStates); // dynamics matrix
    A << 0.9937, 0.75,
         -0.00005, 0.9939;
    
    Eigen::MatrixXd B(nStates, nInputs); // control input matrix
    B << -0.029,
         -0.0064;

    Eigen::MatrixXd C(nOutputs, nStates); // output matrix
    C << 0, 1;

    Eigen::MatrixXd Q(nStates, nStates); // process noise covariance matrix
    Q << 0.0001, 0.0,
         0.0, 0.000001;

    Eigen::MatrixXd R(nInputs, nInputs); // measurment noise covariance matrix
    R << 0.000003;

    Eigen::MatrixXd P0(nStates, nStates); // initial estimate error covariance matrix
    P0 << 0.0001, 0.0,
         0.0, 0.000001;

    // load data set
    int dataSetSize = 1001;
    Eigen::VectorXd input(dataSetSize);
    Eigen::VectorXd trueOutput(dataSetSize);
    Eigen::VectorXd measuredOutput(dataSetSize);
    loadDataSet(dataSetPath, input, trueOutput, measuredOutput);
    
    // construct KalmanFilter object
    KalmanFilter kf(A, B, C, Q, R, P0, sampleTime);

    // initialize the filter
    Eigen::VectorXd x0(nStates);
    x0 << 0.0,
          0.0;
    kf.init(x0);

    // define a vector to hold the estimated output
    Eigen::VectorXd estimatedOutput(dataSetSize);

    // run the filter on the data set of I/O
    Eigen::VectorXd y(nOutputs);
    Eigen::VectorXd u(nInputs);
    std::cout << "Processing dataset..." << std::endl;

    for (int i = 0; i < input.size(); i++)
    {
        y << measuredOutput(i);
        u << input(i);
        kf.update(y, u);
        estimatedOutput(i) = kf.getEstimatedOutput()(0);
    }

    std::cout << "Finished" << std::endl;
    
    // compute mean square error for the estimated output
    std::cout << "Mean square error is: " << computeMSE(trueOutput, estimatedOutput) << std::endl;
}