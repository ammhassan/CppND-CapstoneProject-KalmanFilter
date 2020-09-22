#include <iostream>
#include <string>
#include <fstream>
#include <regex>
#include "KalmanFilter.h"

const std::string dataSetPath{"../data/aircraft_pitch_dynamics_data.txt"};

void loadDataSet(std::string path, Eigen::VectorXd &input, Eigen::VectorXd &trueOutput, Eigen::VectorXd &measuredOutput) {
  std::string line;
  std::string time;
  double u, yTrue, yMeasured;
  std::ifstream filestream(path);
  if (filestream.is_open()) 
  {
    int i = 0;
    while (std::getline(filestream, line)) 
    {
      std::istringstream linestream(line);
      while (linestream >> time >> u >> yTrue >> yMeasured) {
        if (time == "time") 
        {
          // first line that has the columns legends - do not parse data
        }
        else 
        {
            input(i) = u;
            trueOutput(i) = yTrue;
            measuredOutput(i) = yMeasured;
            ++i;
        }
      }
    }
    std::cout << "Dataset has been loaded" << std::endl;
  }
}

int main()
{
    // define the subject dynamic system
    int nStates = 2;
    int nInputs = 1;
    int nOutputs = 1;
    double sampleTime = 0.01;

    Eigen::MatrixXd A(nStates, nStates);
    A << 0.9937, 0.75,
         -0.00005, 0.9939;
    std::cout << "A: " << A << std::endl;
    
    Eigen::MatrixXd B(nStates, nInputs);
    B << -0.029,
         -0.0064;
    std::cout << "B: " << B << std::endl;

    Eigen::MatrixXd C(nOutputs, nStates);
    C << 0, 1;
    std::cout << "C: " << C << std::endl;

    Eigen::MatrixXd Q(nStates, nStates);
    Q << 0.0001, 0.0,
         0.0, 0.000001;
    std::cout << "Q: " << Q << std::endl;

    Eigen::MatrixXd R(nInputs, nInputs);
    R << 0.000003;
    std::cout << "R: " << R << std::endl;

    Eigen::MatrixXd P(nStates, nStates);
    P << 0.0, 0.0,
         0.0, 0.0;
    std::cout << "P: " << P << std::endl;

    // load data set
    int dataSetSize = 1001;
    Eigen::VectorXd input(dataSetSize);
    Eigen::VectorXd trueOutput(dataSetSize);
    Eigen::VectorXd measuredOutput(dataSetSize);
    loadDataSet(dataSetPath, input, trueOutput, measuredOutput);
    
    // construct KalmanFilter object
    KalmanFilter kf(A, B, C, Q, R, P, sampleTime);

    // initialize the filter
    Eigen::VectorXd x0(nStates);
    x0 << 0.0,
          0.0;
    std::cout << "x0: " << x0 << std::endl;
    kf.init(x0);

    // run the filter on the data set of I/O
    Eigen::VectorXd y(nOutputs);
    Eigen::VectorXd u(nInputs);
    for (int i = 0; i < input.size(); i++)
    {
        y << measuredOutput(i);
        u << input(i);
        // print I/O at current step
        std::cout << "Input: " << u << ", Output: " << y << std::endl;

        kf.update(y, u);

        // print state estimate at the current time step
        std::cout << "State estimate at step " << i << " is: " << kf.getState() << std::endl;
    }

    // print state estimate and estimation error at the end time step
    std::cout << "State estimate at the end time step is: " << kf.getState() << std::endl;
    std::cout << "Estimation error percentage at the end time step is " << 
    (trueOutput(trueOutput.size()-1) - kf.getState()(1))/ trueOutput(trueOutput.size()-1) * 100.0 << std::endl;


}