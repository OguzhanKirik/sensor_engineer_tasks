#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cos;
using std::sin;
using std::sqrt;
using std::atan2;

inline double NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2. * M_PI;
    while (angle < -M_PI) angle += 2. * M_PI;
    return angle;
}

UKF::UKF() {
  use_laser_ = true;
  use_radar_ = true;

  x_ = VectorXd(5);
  P_ = MatrixXd(5, 5);

  std_a_ = 3.5;
  std_yawdd_ = 1.2;

  std_laspx_ = 0.15;
  std_laspy_ = 0.15;
  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;

  is_initialized_ = false;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  H_ = MatrixXd(2, n_x_);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  R_ = MatrixXd(2, 2);
  R_ << std_laspx_ * std_laspx_, 0.0,
        0.0, std_laspy_ * std_laspy_;

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    const auto& raw = meas_package.raw_measurements_;

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << raw[0], raw[1], 0.0, 0.0, 0.0;
      P_ = MatrixXd::Identity(n_x_, n_x_);
      P_(0, 0) = std_laspx_ * std_laspx_;
      P_(1, 1) = std_laspy_ * std_laspy_;
      P_(2, 2) = 5.0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = raw[0], phi = raw[1], rhodot = raw[2];
      x_ << rho * cos(phi), rho * sin(phi), rhodot, phi, 0.0;
      P_ = MatrixXd::Identity(n_x_, n_x_);
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

  double dt = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;
  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    UpdateLidar(meas_package);
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    UpdateRadar(meas_package);
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ += K * y;
  P_ = (MatrixXd::Identity(n_x_, n_x_) - K * H_) * P_;
  x_(3) = NormalizeAngle(x_(3));
}

void UKF::Prediction(double delta_t) {
  MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);
  SigmaPointPrediction(&Xsig_aug, delta_t);
  PredictMeanAndCovariance();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  const int n_z = 3;
  MatrixXd Zsig(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred(n_z);
  MatrixXd S(n_z, n_z);

  PredictRadarMeasurement(&z_pred, &S, &Zsig);
  VectorXd z = meas_package.raw_measurements_;

  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));

    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  z_diff(1) = NormalizeAngle(z_diff(1));

  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
  x_(3) = NormalizeAngle(x_(3));
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(n_x_) = x_;

  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug.llt().matrixL();
  Xsig_out->col(0) = x_aug;
  double sqrt_lambda_n_aug = sqrt(lambda_ + n_aug_);

  for (int i = 0; i < n_aug_; ++i) {
    VectorXd offset = sqrt_lambda_n_aug * L.col(i);
    Xsig_out->col(i + 1) = x_aug + offset;
    Xsig_out->col(i + 1 + n_aug_) = x_aug - offset;
  }
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_in, double delta_t) {
  const MatrixXd& Xsig_aug = *Xsig_in;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double px = Xsig_aug(0, i), py = Xsig_aug(1, i), v = Xsig_aug(2, i);
    double phi = Xsig_aug(3, i), phi_dot = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i), nu_yawdd = Xsig_aug(6, i);

    VectorXd x_pred(n_x_);

    if (fabs(phi_dot) > 1e-3) {
      double phi_new = phi + phi_dot * delta_t;
      x_pred(0) = px + v / phi_dot * (sin(phi_new) - sin(phi)) + 0.5 * delta_t * delta_t * cos(phi) * nu_a;
      x_pred(1) = py + v / phi_dot * (-cos(phi_new) + cos(phi)) + 0.5 * delta_t * delta_t * sin(phi) * nu_a;
    } else {
      x_pred(0) = px + v * cos(phi) * delta_t + 0.5 * delta_t * delta_t * cos(phi) * nu_a;
      x_pred(1) = py + v * sin(phi) * delta_t + 0.5 * delta_t * delta_t * sin(phi) * nu_a;
    }

    x_pred(2) = v + delta_t * nu_a;
    x_pred(3) = phi + delta_t * phi_dot + 0.5 * delta_t * delta_t * nu_yawdd;
    x_pred(4) = phi_dot + delta_t * nu_yawdd;

    x_pred(3) = NormalizeAngle(x_pred(3));
    Xsig_pred_.col(i) = x_pred;
  }
}

void UKF::PredictMeanAndCovariance() {
  x_.setZero();
  P_.setZero();

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

  x_(3) = NormalizeAngle(x_(3));
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Z_sig) {
  const int n_z = 3;
  MatrixXd& Zsig = *Z_sig;
  VectorXd& z_pred = *z_out;
  MatrixXd& S = *S_out;

  Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  z_pred.setZero();
  S.setZero();

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double px = Xsig_pred_(0, i), py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i), psi = Xsig_pred_(3, i);

    double rho = sqrt(px * px + py * py);
    double phi = atan2(py, px);
    double rho_dot = (px * cos(psi) * v + py * sin(psi) * v) / std::max(rho, 1e-6);

    Zsig.col(i) << rho, phi, rho_dot;
    z_pred += weights_(i) * Zsig.col(i);
  }

  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R;
}
