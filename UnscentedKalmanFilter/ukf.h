#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Augement sigma points to include prediction error
   * @param Xsig_out The output augmented sigma point x signal matrix
   */
  void AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out);

  /**
   * Predict sigma points using the motion model
   * @param Xsig_in The input augmened sigma point x signal matrix
   * @param delta_t The time difference in seconds
   */
  void SigmaPointPrediction(Eigen::MatrixXd* Xsig_in, double delta_t);

  /**
   * Predict mean and covariance
   */
  void PredictMeanAndCovariance();

  /**
   * Predict radar mesurement
   * @param z_out The output predicted measurement vector z signal vector
   * @param S_out The output innovation covariance matrix S signal matrix
   * @param Z_sig The output matrix with sigma points in measurement space
   */
  void PredictRadarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Z_sig);

  /**
   * Normalize state(2): phi to [-pi, pi]
   * @param x The input/output vector to be modified for angle normalization
   */
  void normalizeAngle(Eigen::VectorXd& x);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // Lidar measurement matrix
  Eigen::MatrixXd H_;

  // Lidar measurement noise covariance matrix
  Eigen::MatrixXd R_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
};

#endif  // UKF_H
