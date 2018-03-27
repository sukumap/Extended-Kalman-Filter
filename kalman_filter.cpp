#include "kalman_filter.h"
#include "tools.h"
#include <iostream> 

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.
const float PI = 3.14159265;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in, MatrixXd &R_rad) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_laser = R_in;
  R_radar = R_rad;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //cout << "F_: " << F_<<endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  //cout<<"Predicted ouput "<< x_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_laser;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd h_ = tools.CalculateJacobian(x_);
//cout << x_;


    double ro_pred = pow(pow(x_[0], 2) + pow(x_[1], 2), 0.5);
    double theta_pred = 0.0;
/* Make sure no division by zero */
    if (fabs(x_[0]) > 0.0001) {
        theta_pred = atan2(x_[1], x_[0]);
    }

    double rodot_pred = 0.0;
    if (fabs(ro_pred) > 0.0001) {
        rodot_pred = (x_[0] * x_[2] + x_[1] * x_[3]) / ro_pred;
    }


    VectorXd z_pred(3);
    z_pred << ro_pred, theta_pred, rodot_pred;

    VectorXd y = z - z_pred;
    if(y[1] > PI)
    {
      /* subtract -2 PI to normalize the angle */
      y[1] = y[1] - (2 *PI);
    }
    else if (y[1] <  -PI)
    {
      /* add 2 PI to normalize the angle */      
      y[1] = y[1] + (2*PI);
    }
    MatrixXd ht = h_.transpose();
    MatrixXd S = h_ * P_ * ht + R_radar;
    MatrixXd Si = S.inverse();
    MatrixXd Pht = P_ * ht;
    MatrixXd K = Pht * Si;

    // new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * h_) * P_;

}
