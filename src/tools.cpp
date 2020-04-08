#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd err(4);
  /* error length checking */
  if (   (estimations.size() == 0)
       ||(estimations.size() != ground_truth.size())){
    return rmse;       
  }
      
  for (int i=0; i<estimations.size(); i++){
    err = estimations[i] - ground_truth[i];
    err = err.array()*err.array();
    rmse += err;
  }
      
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
      
  return rmse;  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float pxy = px * px + py * py;
  
  /* pxy is the distantce to origin; close to 0 means sensor blocked or collision; assign a small value to handle div by 0 */
  if (pxy <0.01){
    pxy = 0.01;
  }
  
  Hj(0,0) = px / sqrt(pxy);
  Hj(0,1) = py / sqrt(pxy);
  Hj(0,2) = 0;
  Hj(0,3) = 0;

  Hj(1,0) = -py / pxy;
  Hj(1,1) = px / pxy;
  Hj(1,2) = 0;
  Hj(1,3) = 0;

  Hj(2,0) = py * (vx*py-vy*px) / pow(pxy,1.5);
  Hj(2,1) = px * (vy*px-vx*py) / pow(pxy,1.5);
  Hj(2,2) = px / sqrt(pxy);
  Hj(2,3) = py / sqrt(pxy);
  
  return Hj;
}
