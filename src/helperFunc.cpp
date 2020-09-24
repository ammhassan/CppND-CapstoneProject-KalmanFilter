#include "helperFunc.h"
#include <fstream>
#include <iostream>

// This function loads a data set I/O from a text file.
void loadDataSet(std::string path, Eigen::VectorXd &input, Eigen::VectorXd &trueOutput, Eigen::VectorXd &measuredOutput) 
{
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
          // first line that has the columns titles - do not parse data
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

// This function computes the mean square error between the true (clean) output and the estimated one.
float computeMSE(Eigen::VectorXd &trueOutput, Eigen::VectorXd &estimatedOutput)
{
    float meanSquareError = 0.0;
    for (int i = 0; i < trueOutput.size(); ++i)
    {
        double error = trueOutput(i) - estimatedOutput(i);
        meanSquareError += error * error / trueOutput.size();
    }
return meanSquareError;
}