#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.2;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  /**
   * TODO (done): Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Lidar measurement matrix
  H_ = MatrixXd(2, n_x_);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  // Lidar measurement noise covariance matrix
  R_ = MatrixXd(2, 2);
  R_ << std_laspx_ * std_laspx_, 0.0,
        0.0, std_laspy_ * std_laspy_;

  // set vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  double weight = 0.5 / (lambda_ + n_aug_);
  weights_(0) = weight_0;

  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  
    weights_(i) = weight;
  }

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO (done): Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
 if (!is_initialized_) {

    const auto& raw = meas_package.raw_measurements_;
    const auto sensor = meas_package.sensor_type_;

    if (sensor == MeasurementPackage::LASER) {
        x_ << raw[0], raw[1], 0.0, 0.0, 0.0;

        P_ = MatrixXd::Zero(n_x_, n_x_);
        P_(0, 0) = std_laspx_ * std_laspx_;
        P_(1, 1) = std_laspy_ * std_laspy_;
        P_(2, 2) = 5.0;
        P_(3, 3) = 1.0;
        P_(4, 4) = 1.0;
    } 
    else if (sensor == MeasurementPackage::RADAR) {
        const double rho = raw[0];
        const double phi = raw[1];
        const double rhodot = raw[2];

        x_ << rho * cos(phi), rho * sin(phi), rhodot, phi, 0.0;

        P_ = MatrixXd::Zero(n_x_, n_x_);
        P_(0, 0) = 10.0;
        P_(1, 1) = 10.0;
        P_(2, 2) = 0.1;
        P_(3, 3) = 0.01;
        P_(4, 4) = 0.0001;
    }


    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
}

// Compute the time elapsed between measurements (in seconds)
double dt = (meas_package.timestamp_ - time_us_) / 1e6;
time_us_ = meas_package.timestamp_;

// Predict
Prediction(dt);

// Update based on sensor type
if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
} else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
}

}
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

     // LIDAR measures px and py only
    const int n_z = 2;

    // Extract measurement
    const VectorXd& z = meas_package.raw_measurements_;
    const VectorXd z_pred = H_ * x_;
    const VectorXd y = z - z_pred;

    // Measurement update
    const MatrixXd Ht = H_.transpose();
    const MatrixXd S = H_ * P_ * Ht + R_;
    const MatrixXd K = P_ * Ht * S.inverse();

    // Update state estimate and covariance
    x_ += K * y;
    P_ = (MatrixXd::Identity(n_x_, n_x_) - K * H_) * P_;

    // Normalize angle in state vector
    normalizeAngle(x_);



}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);

  // call the function to assign the sigma point matrix
  AugmentedSigmaPoints(&Xsig_aug);

  // update sigma point matrix Xsig_pred_
  SigmaPointPrediction(&Xsig_aug, delta_t);

  // predict mean and covariance
  PredictMeanAndCovariance();
}

// void UKF::UpdateLidar(MeasurementPackage meas_package) {
//   /**
//    * TODO: Complete this function! Use lidar data to update the belief 
//    * about the object's position. Modify the state vector, x_, and 
//    * covariance, P_.
//    * You can also calculate the lidar NIS, if desired.
//    */

//      // LIDAR measures px and py only
//     const int n_z = 2;

//     // Extract measurement
//     const VectorXd& z = meas_package.raw_measurements_;
//     const VectorXd z_pred = H_ * x_;
//     const VectorXd y = z - z_pred;

//     // Measurement update
//     const MatrixXd Ht = H_.transpose();
//     const MatrixXd S = H_ * P_ * Ht + R_;
//     const MatrixXd K = P_ * Ht * S.inverse();

//     // Update state estimate and covariance
    

//     // Normalize angle in state vector
//     normalizeAngle(x_);



// }

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO (done): Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

    const int n_z = 3;

    // Extract predicted sigma points and state
    const MatrixXd& Xsig_pred = Xsig_pred_;
    const VectorXd& x = x_;
    const MatrixXd& P = P_;

    // Measurement prediction containers
    MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);
    VectorXd z_pred = VectorXd::Zero(n_z);
    MatrixXd S = MatrixXd::Zero(n_z, n_z);

    // Predict radar measurement
    PredictRadarMeasurement(&z_pred, &S, &Zsig);

    // Incoming radar measurement
    const VectorXd& z = meas_package.raw_measurements_;

    // Cross correlation matrix
    MatrixXd T = MatrixXd::Zero(n_x_, n_z);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        const VectorXd x_diff = Xsig_pred.col(i) - x;
        const VectorXd z_diff = Zsig.col(i) - z_pred;
        T += weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain
    const MatrixXd K = T * S.inverse();

    // Update state mean and covariance
    x_ += K * (z - z_pred);
    P_ -= K * S * K.transpose();

    // Normalize yaw angle in state vector
    normalizeAngle(x_);




 
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

// Create augmented mean vector
VectorXd x_aug = VectorXd::Zero(n_aug_);
x_aug.head(n_x_) = x_;

// Create augmented covariance matrix
MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
P_aug.topLeftCorner(n_x_, n_x_) = P_;
P_aug(n_x_, n_x_)     = std_a_ * std_a_;
P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

// Calculate square root of P_aug
const MatrixXd L = P_aug.llt().matrixL();

// Create sigma point matrix
MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
Xsig_aug.col(0) = x_aug;

const double scaling_factor = sqrt(lambda_ + n_aug_);
for (int i = 0; i < n_aug_; ++i) {
    const VectorXd offset = scaling_factor * L.col(i);
    Xsig_aug.col(i + 1)        = x_aug + offset;
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - offset;
}

// Output the result
*Xsig_out = Xsig_aug;




}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_in, double delta_t) {

    const MatrixXd& Xsig_aug = *Xsig_in;
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

    const double half_dt2 = 0.5 * delta_t * delta_t;

    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        const double px     = Xsig_aug(0, i);
        const double py     = Xsig_aug(1, i);
        const double v      = Xsig_aug(2, i);
        const double phi    = Xsig_aug(3, i);
        const double phi_dot = Xsig_aug(4, i);
        const double nu_a   = Xsig_aug(5, i);
        const double nu_yawdd = Xsig_aug(6, i);

        VectorXd x_pred = VectorXd(n_x_);

        if (fabs(phi_dot) < 1e-3) {
            x_pred(0) = px + v * cos(phi) * delta_t + half_dt2 * cos(phi) * nu_a;
            x_pred(1) = py + v * sin(phi) * delta_t + half_dt2 * sin(phi) * nu_a;
        } else {
            const double phi_new = phi + phi_dot * delta_t;
            x_pred(0) = px + v / phi_dot * (sin(phi_new) - sin(phi)) + half_dt2 * cos(phi) * nu_a;
            x_pred(1) = py + v / phi_dot * (-cos(phi_new) + cos(phi)) + half_dt2 * sin(phi) * nu_a;
        }

        x_pred(2) = v + delta_t * nu_a;
        x_pred(3) = phi + delta_t * phi_dot + half_dt2 * nu_yawdd;
        x_pred(4) = phi_dot + delta_t * nu_yawdd;

        normalizeAngle(x_pred);
        Xsig_pred.col(i) = x_pred;
    }

    Xsig_pred_ = Xsig_pred;

}

void UKF::normalizeAngle(VectorXd& x) {
  while (x(3)> M_PI) x(3) -= 2.0 * M_PI;
  while (x(3)<-M_PI) x(3) += 2.0 * M_PI;
}

void UKF::PredictMeanAndCovariance() {

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred = Xsig_pred_;

  // create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  x.fill(0.0);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  P.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // predict state mean
    x += weights_(i) * Xsig_pred.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // Useful check: if the phi stays in [-pi, pi]
    // angle normalization
    VectorXd x_diff = Xsig_pred.col(i) - x;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    // predict state covariance matrix
    P += weights_(i) * x_diff * x_diff.transpose();
  }

  // write result
  x_ = x;
  P_ = P;

  // normalize angle for state x_
  normalizeAngle(x_);
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Z_sig) {

  const int n_z = 3;  // Radar measures r, phi, and r_dot
  const int n_sig = 2 * n_aug_ + 1;

  const MatrixXd& Xsig_pred = Xsig_pred_;  // reference to avoid copying

  MatrixXd Zsig = MatrixXd(n_z, n_sig);
  VectorXd z_pred = VectorXd::Zero(n_z);
  MatrixXd S = MatrixXd::Zero(n_z, n_z);

  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig; ++i) {
      const double px  = Xsig_pred(0, i);
      const double py  = Xsig_pred(1, i);
      const double v   = Xsig_pred(2, i);
      const double psi = Xsig_pred(3, i);

      const double rho = sqrt(px * px + py * py);
      const double phi = atan2(py, px);
      const double rho_dot = (px * cos(psi) * v + py * sin(psi) * v) / std::max(rho, 1e-6);  // avoid divide-by-zero

      VectorXd z_tmp(n_z);
      z_tmp << rho, phi, rho_dot;

      Zsig.col(i) = z_tmp;
      z_pred += weights_(i) * z_tmp;
  }

  // Add measurement noise covariance matrix R
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  // Compute innovation covariance matrix S
  for (int i = 0; i < n_sig; ++i) {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));  // Normalize angle
      S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R;

  // Output results
  *z_out = z_pred;
  *S_out = S;
  *Z_sig = Zsig;

}

