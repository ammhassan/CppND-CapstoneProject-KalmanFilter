#include "helperFunc.h"
#include <fstream>
#include <iostream>

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