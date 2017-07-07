#include "kalman_filter.h"
#include "measurement_package.h"
#include "FusionEKF.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using std::vector;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, VectorXd &hx_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in, MatrixXd &Hj_in) {
  x_ = x_in;
  hx_ = hx_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  Hj_ = Hj_in;
  //z_ = z_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  cout << "x_" << endl << x_ << endl << endl;
  cout << "F_" << endl << x_ << endl << endl;
 
  x_ = F_ * x_;
  
  
  MatrixXd Ft = F_.transpose();
  
  cout << "P_" << endl << x_ << endl << endl;
  cout << "Q_" << endl << x_ << endl << endl;
  
P_ = (F_ * P_ * Ft) + Q_;}
  


void KalmanFilter::Update(const VectorXd &z_) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  
  //VectorXd z_ = VectorXd::Zero(2);
  
  
  //ekf_z_.Update(meas_package.raw_measurements_);
  //float px = meas_package.raw_measurements_[0];
  //float py = meas_package.raw_measurements_[1];
  //z_ << px, py;
  //VectorXd y = meas_package.raw_measurements_ - H_ * x_;
  
  VectorXd y = z_ - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = (H_ * P_ * Ht) + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z_) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float p_x = x_(0);
  float p_y = x_(1);
  float v1 = x_(2);
  float v2 = x_(3);
  float c1 = p_x*p_x + p_y*p_y;
  float c2 = sqrt(c1);
  
  hx_ = VectorXd(3);
  hx_ << 1,1,1; //initialize
  hx_ << c2, atan2(p_y,p_x), (p_x*v1 + p_y*v2)/c2; 
  //account for 2pi on angle
  
  //VectorXd zext_ = VectorXd::Zero(3);
  
  //ekf_zext_.UpdateEKF(meas_package.raw_measurements_);
  //float ro = meas_package.raw_measurements_[0];
  //float theta = meas_package.raw_measurements_[1];
  //float ro_dot = meas_package.raw_measurements_[2];
  //zext_ << ro, theta, ro_dot;
  //VectorXd y = meas_package.raw_measurements_ - hx_;
  VectorXd y = z_ - hx_;
  while (y[1] > M_PI) {
      y[1] -= 2.* M_PI;
  }
  while (y[1] < -M_PI) {
    y[1] += 2.* M_PI;
  }

  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = (Hj_ * P_ * Hjt) + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Hjt * Si;
  
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}