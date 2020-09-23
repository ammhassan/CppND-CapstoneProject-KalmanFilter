#include <Eigen/Dense>

void loadDataSet(std::string path, Eigen::VectorXd &input, Eigen::VectorXd &trueOutput, Eigen::VectorXd &measuredOutput);

float computeMSE(Eigen::VectorXd &trueOutput, Eigen::VectorXd &estimatedOutput);
