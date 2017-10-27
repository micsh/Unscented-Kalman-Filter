#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF
{
public:
  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

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

private:
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* agumented sigma points matrix
  MatrixXd Xsig_aug_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long last_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Noise for lidar
  MatrixXd RL_;

  ///* Noise for radar
  MatrixXd RR_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  /**
   * Update Radar or Lidar measurement
   */
  void UpdateMeasurement(VectorXd z, MatrixXd Zsig, MatrixXd R);

  /**
   * Calculate the agumented sigma points by sampling the P_ with sigma points,
   * and agument the state vectors with the mean of the process noise values (have zero mean).
   * This function update Xsig_aug_
   */
  void CalculateAugmentedSigmaPoints();

  /**
   * Calculate the predicted sigma points.
   * Run all the sigma points through the nonlinear process model,
   * to get the sigma points at k+1|k.
   * These sigma points will be used later in PredictMeanAndCovariance to construct the state x_ and covariance P_ at k+1|k.
   * This function update Xsig_pred_
   */
  void PredictSigmaPoints(double delta_t);

  /**
   * Update the state and covariance using the predicted sigma points.
   * Update the state and covariance to k+1|k.
   * This function update x_ and P_
   */
  void PredictMeanAndCovariance();

  /**
   * Initialize the state and covariance and time using the first measurement.
   * This function update last_timestamp_ and initialize x_ and P_
   */
  void Initialization(MeasurementPackage meas_package);
};

#endif /* UKF_H */