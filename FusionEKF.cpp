#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//Kalman Filter variables
VectorXd x_;// object state
MatrixXd P_;// object covariance matrix
//VectorXd u;// external motion
MatrixXd F_; // state transition matrix
MatrixXd H_;// laser measurement matrix
MatrixXd Hj_;//Jacobian matrix
MatrixXd R_laser_;// measurement covariance matrix
MatrixXd R_radar_;
//MatrixXd I; // Identity matrix
MatrixXd Q_;// process covariance matrix
VectorXd hx_;  //extended kalman
//VectorXd z_; //kalman and extended kalman
//VectorXd zext_; // extended kalman


/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices and vectors
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.F_ = MatrixXd(4,4);
  P_ = MatrixXd(4,4);
  ekf_.Q_ = MatrixXd(4,4);
  hx_ = VectorXd(3);
  //x_ = VectorXd(4);
  //z_ = VectorXd(2);
  //zext_ = VectorXd(3);
  

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

//initialize H_laser_
  H_ << 1,0,0,0,
        0,1,0,0;

//state initial transition matrix
  ekf_.F_ << 1,0,1,0,
             0,1,0,1,
             0,0,1,0,
             0,0,0,1;

//covariance matrix
  ekf_.Q_ << 1,0,1,0,
             0,1,0,1,
             1,0,1,0,
             0,1,0,1;

//initialize P_
  P_ << 1,0,0,0,
        0,1,0,0,
        0,0,1000,0,
        0,0,0,1000;

//initialize Hj_
  Hj_<< 1,1,0,0,
        1,1,0,0,
        1,1,1,1;

//initialize hx_
  hx_<<1,1,1;
   
//initialize x_
  //x_ << 1,1,0,0;
  
//initialize z
  //z_ << 0,0;
  
//initialize zext
  //zext_ << 0,0,0;


  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &meas_package) {

//was &measurement_pack
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;
    float p_x = x_(0);
    float p_y = x_(1);
    float v1 = x_(2);
    float v2 = x_(3);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /** sign? on py
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //set the state with the initial location and zero velocity
      ekf_.x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0;
      float ro = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];
      float ro_dot = meas_package.raw_measurements_[2];
      float px = ro*cos(theta);
      float py = ro*sin(theta);
      float p_x = px;
      float p_y = py;
      ekf_.x_<< p_x, p_y, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      ekf_.x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0;
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      float p_x = px;
      float p_y = py;
      ekf_.x_<< p_x, p_y, 0, 0;
      }
      previous_timestamp_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (previous_timestamp_ == 0) {
	  //initialize P_
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_<< 1,0,0,0,
              0,1,0,0,
              0,0,1000,0,
              0,0,0,1000;
  }
  else {
    ekf_.P_ = MatrixXd(4,4);

  }


  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    float dt = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    previous_timestamp_ = meas_package.timestamp_;
    //update state matrix 
    ekf_.F_(0,3) = dt;
    ekf_.F_(1,4) = dt;

    //update covariance matrix
    ekf_.Q_ << (dt4/4)* 9, 0, (dt3/2)*9, 0,
         0, (dt4/4)*9, 0, (dt3/2)*9,
         (dt3/2)*9, 0, dt2*9, 0,
         0, (dt3/2)*9, 0, dt2*9;
  }
  
  else {
    // Laser updates
    float dt = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    previous_timestamp_ = meas_package.timestamp_;

    // update state matrix
    ekf_.F_(0,3) = dt;
    ekf_.F_(1,4) = dt;

    //update covariance matrix
    ekf_.Q_ << (dt4/4)* 9, 0, (dt3/2)*9, 0,
          0, (dt4/4)*9, 0, (dt3/2)*9,
          (dt3/2)*9, 0, dt2*9, 0,
          0, (dt3/2)*9, 0, dt2*9;
  }
  


  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(meas_package.raw_measurements_);
  } else {
    // Laser updates
    //ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(meas_package.raw_measurements_);
  }  //added
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
