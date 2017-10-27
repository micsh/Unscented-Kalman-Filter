#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

double NormalizeAngle(double phi);

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // spreading parameter for sigma points
  lambda_ = 3 - n_aug_;

  // previous measurement time
  last_timestamp_ = 0;

  // set weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  weights_.tail(2 * n_aug_).fill(0.5 / (n_aug_ + lambda_));

  // initial state vector
  x_ = VectorXd::Constant(n_x_, 1.);

  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // augmented sigma points
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_ = 0.15;
  // Laser measurement noise standard deviation position2 in m
  double std_laspy_ = 0.15;

  RL_ = MatrixXd(2, 2);
  RL_ << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_ = 0.3;
  // Radar measurement noise standard deviation angle in rad
  double std_radphi_ = 0.03;
  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ = 0.3;

  RR_ = MatrixXd(3, 3);
  RR_ << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  is_initialized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  //*************** initialization *********************
  if (!is_initialized_)
  {
    Initialization(meas_package);
    return;
  }

  double delta_t = (meas_package.timestamp_ - last_timestamp_) / 1000000.0; //dt - expressed in seconds
  last_timestamp_ = meas_package.timestamp_;

  //*************** Prediction ************************
  Prediction(delta_t);

  //**************** Update **************************
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
  /**
     Use previous state and covariance and delta_t to predit
     the state x_ and covariance, P_ at current time (from k|k to k+1|k).
  */

  // calculate augmented sigma points, populate Xsig_aug_
  CalculateAugmentedSigmaPoints();

  // calculate prediction on sigma points, populate Xsig_pred_
  PredictSigmaPoints(delta_t);

  // calculate predicted mean and covariance, update x_ and P_ from k|k to k+1|k.
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * update x_ and P_ from predictions at current time based on previous time measurment k+1|k
 * to current time based on current time measurement k+1|k+1
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  // transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    Zsig(0, i) = Xsig_pred_(0, i); //r
    Zsig(1, i) = Xsig_pred_(1, i); //phi
  }

  UpdateMeasurement(meas_package.raw_measurements_, Zsig, RL_);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * update x_ and P_ from predictions at current time based on previous time measurment k+1|k
 * to current time based on current time measurement k+1|k+1
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  // transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1); 
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);        //r
    Zsig(1, i) = atan2(p_y, p_x);                    //phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / Zsig(0, i); //r_dot
  }

  UpdateMeasurement(meas_package.raw_measurements_, Zsig, RR_);
}

void UKF::UpdateMeasurement(VectorXd z, MatrixXd Zsig, MatrixXd R)
{
  int n = Zsig.rows();

  //mean predicted measurement z_pred
  VectorXd z_pred = VectorXd::Zero(n);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n, n);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  S = S + R;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd::Zero(n_x_, n);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;
  z_diff(1) = NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

void UKF::CalculateAugmentedSigmaPoints()
{
  // create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_ + 1) = 0;

  // create augmented covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
};

void UKF::PredictSigmaPoints(double delta_t)
{
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001)
    {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else
    {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // avoid zero division
    if (fabs(px_p) < 0.0001 && fabs(py_p) < 0.0001)
    {
      px_p = 0.0001;
    }

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
};

void UKF::PredictMeanAndCovariance()
{
  // state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    if (x_diff(3) > 1000000)
    {
      std::cout << "error: predicted angle too large: " << x_diff(3) << std::endl;
      std::exit(0);
    }

    x_diff(3) = NormalizeAngle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
};

void UKF::Initialization(MeasurementPackage meas_package)
{
  last_timestamp_ = meas_package.timestamp_;

  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    // initialize state vector using laser data
    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    // initialize state vector using rader data
    double rho = meas_package.raw_measurements_[0];
    double phi = meas_package.raw_measurements_[1];
    double px = rho * cos(phi);
    double py = rho * sin(phi);

    x_ << px, py, fabs(meas_package.raw_measurements_[2]), 0, 0;
  }

  is_initialized_ = true;
};

double NormalizeAngle(double phi)
{
  const double Max = M_PI;
  const double Min = -M_PI;

  return phi < Min ? Max + std::fmod(phi - Min, Max - Min) : std::fmod(phi - Min, Max - Min) + Min;
}