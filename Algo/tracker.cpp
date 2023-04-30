//
// Created by Andrei on 30-Apr-23.
//

#include "tracker.hpp"
#include <cmath>

Tracker::Tracker() {
  x_ = Eigen::VectorXd(5);  // initial state vector
  P_ = Eigen::MatrixXd(5, 5);  // initial covariance matrix
  std_a_ = 2.0;  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_yawdd_ = 1.0;  // Process noise standard deviation yaw acceleration in rad/s^2
  std_laspx_ = 0.15;  // Laser measurement noise standard deviation position1 in m
  std_laspy_ = 0.15;  // Laser measurement noise standard deviation position2 in m
  std_radr_ = 0.3;  // Radar measurement noise standard deviation radius in m
  std_radphi_ = 0.03;  // Radar measurement noise standard deviation angle in rad
  std_radrd_ = 0.3;  // Radar measurement noise standard deviation radius change in m/s
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    if (i == 0) {
      weights_(i) = lambda_ / (lambda_ + n_aug_);
    } else {
      weights_(i) = 1 / (2 * (lambda_ + n_aug_));
    }
  }
}

Tracker::~Tracker() = default;

void Tracker::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    // Initializing differs depending on the first sensor that hits the cars
    if (meas_package.sensorType == MeasurementPackage::SensorType::LASER) {
      x_ << meas_package.rawMeasurements[0], meas_package.rawMeasurements[1], 0, 0, 0;
      P_ = Eigen::MatrixXd::Identity(5, 5);
      P_(0,0) = std_laspx_ * std_laspx_;
      P_(1,1) = std_laspy_ * std_laspy_;
    } else if (meas_package.sensorType == MeasurementPackage::SensorType::RADAR) {
      const double x = meas_package.rawMeasurements(0) * cos(meas_package.rawMeasurements(1));
      const double y = meas_package.rawMeasurements(0) * sin(meas_package.rawMeasurements(1));
      const double v = meas_package.rawMeasurements(2);
      x_ << x, y, v, meas_package.rawMeasurements(0), meas_package.rawMeasurements(2);
      P_ = Eigen::MatrixXd::Identity(5, 5);
      P_ = P_.array() * (std_radr_* std_radr_);
      P_(2,2) = std_radrd_ * std_radrd_;
      P_(3,3) = std_radphi_ * std_radphi_;
      P_(4,4) = std_radphi_ * std_radphi_;
    }
    time_us_ = meas_package.timeStamp;
    is_initialized_ = true;
    return;
  }
  // time difference in seconds
  const float dt = (meas_package.timeStamp - time_us_) / 1000000.0;
  time_us_ = meas_package.timeStamp;
  // Predict the next state & covariance
  Prediction(dt);
  // Update the state & covariance
  if (meas_package.sensorType == MeasurementPackage::SensorType::LASER) {
    UpdateLidar(meas_package);
  }
  // Both sensors can be used in an iteration if both measurements are available
  if (meas_package.sensorType == MeasurementPackage::SensorType::RADAR) {
    UpdateRadar(meas_package);
  }
}

void Tracker::Prediction(double delta_t) {
  // Create augmented mean vector, augmented state covariance
  Eigen::VectorXd x_aug = createAugmentedMeanVector();
  Eigen::MatrixXd P_aug = createAugmentedCovarianceMatrix();
  // Create square root matrix
  Eigen::MatrixXd A = P_aug.llt().matrixL();
  // Create augmented sigma points
  Eigen::MatrixXd Xsig_aug = createAugmentedSigmaPoints(A, x_aug);
  // Predict sigma points
  Xsig_pred_ = predictSigmaPoints(Xsig_aug, delta_t);
  // Predicted mean and covariance
  x_ = Xsig_pred_ * weights_;
  P_ = calculatePredictedCovariance(Xsig_pred_, x_);
}

Eigen::VectorXd Tracker::createAugmentedMeanVector() const {
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
  x_aug.setZero(n_aug_);
  x_aug.head(5) = x_;
  return x_aug;
}

Eigen::MatrixXd Tracker::createAugmentedCovarianceMatrix() const {
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
  P_aug.setZero(n_aug_, n_aug_);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug.bottomRightCorner(2, 2) << std_a_ * std_a_, 0, 0, std_yawdd_ * std_yawdd_;
  return P_aug;
}

Eigen::MatrixXd Tracker::createAugmentedSigmaPoints(const Eigen::MatrixXd& A, const Eigen::VectorXd& x_aug) const {
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.middleCols(1, n_aug_) = (std::sqrt(lambda_ + n_aug_) * A.array()).colwise() + x_aug.array();
  Xsig_aug.middleCols(n_aug_ + 1, n_aug_) = (-std::sqrt(lambda_ + n_aug_) * A.array()).colwise() + x_aug.array();
  return Xsig_aug;
}

Eigen::MatrixXd Tracker::predictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, double delta_t) const {

  Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  for (int i = 0; i < Xsig_aug.cols(); i++) {

    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yaw_rate = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);
    double px_p, py_p;

    if (fabs(yaw_rate) > 0.001) {
      px_p = px + (v/yaw_rate) * (sin(yaw + yaw_rate*delta_t) - sin(yaw));
      py_p = py + (v/yaw_rate) * (cos(yaw) - cos(yaw + yaw_rate*delta_t));
    } else {
      px_p = px + v*delta_t*cos(yaw);
      py_p = py + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yaw_rate*delta_t;
    double yawd_p = yaw_rate;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t*cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t*sin(yaw);
    v_p = v_p + nu_a*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
  return Xsig_pred;
}

Eigen::MatrixXd Tracker::calculatePredictedCovariance(const Eigen::MatrixXd& Xsig_pred, const Eigen::VectorXd& x) {
  // Initialize predicted covariance matrix
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(n_x_, n_x_);
  // Calculate predicted covariance matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Eigen::VectorXd x_diff = Xsig_pred.col(i) - x;
    // Normalize the angle between -pi to pi
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));
    P_pred = P_pred + weights_(i) * x_diff * x_diff.transpose();
  }
  return P_pred;
}

void Tracker::UpdateLidar(const MeasurementPackage& meas_package) {
  int n_z = 2;  // Measurement dimension
  Eigen::MatrixXd Zsig = createSigmaPointsLidar(n_z);
  Eigen::VectorXd z_pred = predictMeasurementMeanLidar(Zsig, n_z);
  Eigen::MatrixXd S = predictMeasurementCovarianceLidar(Zsig, z_pred, n_z);
  Eigen::MatrixXd Tc = calculateCrossCorrelationLidar(Zsig, z_pred, n_z);
  Eigen::MatrixXd K = calculateKalmanGain(Tc, S);
  updateStateMeanAndCovarianceLidar(K, z_pred, meas_package, S);
}

Eigen::VectorXd Tracker::predictMeasurementMeanLidar(const Eigen::MatrixXd& Zsig, int n_z) {
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);  // mean predicted measurement
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  return z_pred;
}

Eigen::MatrixXd Tracker::predictMeasurementCovarianceLidar(const Eigen::MatrixXd& Zsig, const Eigen::VectorXd& z_pred, int n_z) {
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);  // measurement covariance matrix
  // calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;  // residual
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);  // measurement noise covariance matrix
  R << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;
  S = S + R;
  return S;
}

Eigen::MatrixXd Tracker::calculateCrossCorrelationLidar(Eigen::MatrixXd Zsig, const Eigen::VectorXd& z_pred, int n_z) {
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;  // residual
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;  // state difference
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  return Tc;
}

void Tracker::updateStateMeanAndCovarianceLidar(Eigen::MatrixXd K, const Eigen::VectorXd& z_pred, const MeasurementPackage& meas_package, const Eigen::MatrixXd& S) {
  Eigen::VectorXd z_diff = meas_package.rawMeasurements - z_pred;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

void Tracker::UpdateRadar(const MeasurementPackage& meas_package) {
  int n_z = 3;  // Measurement dimension

  Eigen::MatrixXd Zsig = createSigmaPointsInMeasurementSpace(n_z);
  Eigen::VectorXd z_pred = predictMeasurementMean(Zsig, n_z);
  Eigen::MatrixXd S = predictMeasurementCovariance(Zsig, z_pred, n_z);
  Eigen::MatrixXd Tc = calculateCrossCorrelation(Zsig, z_pred, n_z);
  Eigen::MatrixXd K = calculateKalmanGain(Tc, S);
  updateStateMeanAndCovariance(K, z_pred, meas_package, S);
}

Eigen::MatrixXd Tracker::createSigmaPointsInMeasurementSpace(int n_z) {
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);
  }
  return Zsig;
}

Eigen::VectorXd Tracker::predictMeasurementMean(Eigen::MatrixXd Zsig, int n_z) {
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  return z_pred;
}

Eigen::MatrixXd Tracker::predictMeasurementCovariance(Eigen::MatrixXd Zsig, const Eigen::VectorXd& z_pred, int n_z) {
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;
  S = S + R; // Add measurement noise covariance matrix R to the measurement prediction covariance S
  return S;
}

Eigen::MatrixXd Tracker::calculateCrossCorrelation(Eigen::MatrixXd Zsig, const Eigen::VectorXd& z_pred, int n_z) {
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  return Tc;
}

Eigen::MatrixXd Tracker::calculateKalmanGain(const Eigen::MatrixXd& Tc, const Eigen::MatrixXd& S) {
  Eigen::MatrixXd K = Tc * S.inverse();
  return K;
}

void Tracker::updateStateMeanAndCovariance(Eigen::MatrixXd K, const Eigen::VectorXd& z_pred, const MeasurementPackage& meas_package, Eigen::MatrixXd& S) {
  Eigen::VectorXd z = meas_package.rawMeasurements;
  Eigen::VectorXd z_diff = z - z_pred;
  while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}
