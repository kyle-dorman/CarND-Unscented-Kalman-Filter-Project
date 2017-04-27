#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::string;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse = VectorXd(estimations[0].size());
  rmse.fill(0.0);

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0)
  {
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}


void Tools::NISPercentage(const std::vector<double> &nis, int deg_freedom, string name) {
  double max_val;
  if (deg_freedom == 2) {
    max_val = 5.991;
  } else if (deg_freedom == 3) {
    max_val = 7.815;
  } else {
    return;
  }

  int count = 0;
  for(int i = 0; i < nis.size(); i++) {
    if (nis[i] < max_val) {
      count++;
    }
  }

  double percent_below = 100.0 * count / nis.size();

  cout << name << " has " << percent_below << "% below max val." << endl;
}
