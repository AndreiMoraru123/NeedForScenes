//
// Created by Andrei on 30-Apr-23.
//

#ifndef NFS_TRACKER_HPP
#define NFS_TRACKER_HPP

#include <pcl/common/common.h>
#include <Eigen/Dense>
#include "../Sensors/sensors.hpp"

class Tracker {
public:
  Tracker();
  virtual ~Tracker();
  void ProcessMeasurement(MeasurementPackage meas_package);
  void Prediction(double delta_t);
  void UpdateLidar(const MeasurementPackage& meas_package);
  void UpdateRadar(const MeasurementPackage& meas_package);

  bool is_initialized_; // Initially set to false, set to true in first call of ProcessMeasurement
  Eigen::VectorXd x_; // state vector
  Eigen::MatrixXd P_; // state covariance matrix
  Eigen::MatrixXd Xsig_pred_; // predicted sigma points matrix
  double std_a_; // Process noise standard deviation longitudinal acceleration in m/s^2
  long long time_us_; // time when the state is true, in us
  double std_yawdd_; // Process noise standard deviation yaw acceleration in rad/s^2
  double std_laspx_; // Laser measurement noise standard deviation position1 in m
  double std_laspy_; // Laser measurement noise standard deviation position2 in m
  double std_radr_; // Process noise standard deviation radius in m
  double std_radphi_; // Process noise standard deviation yaw angle in rad
  double std_radrd_; // Process noise standard deviation radius change in m/s
  Eigen::VectorXd weights_; // Weights of sigma points
  double lambda_; // spreading parameter
  int n_x_; // state dimension
  int n_aug_; // augmented state dimension

  // Helper functions for Prediction
  Eigen::MatrixXd calculatePredictedCovariance(const Eigen::MatrixXd &Xsig_pred, const Eigen::VectorXd &x);
  [[nodiscard]] Eigen::MatrixXd predictSigmaPoints(const Eigen::MatrixXd &Xsig_aug, double delta_t) const;
  [[nodiscard]] Eigen::MatrixXd createAugmentedSigmaPoints(const Eigen::MatrixXd &A, const Eigen::VectorXd &x_aug) const;
  [[nodiscard]] Eigen::MatrixXd createAugmentedCovarianceMatrix() const;
  [[nodiscard]] Eigen::VectorXd createAugmentedMeanVector() const;

  // Helper functions for Radar Update
  static Eigen::MatrixXd calculateKalmanGain(const Eigen::MatrixXd& Tc, const Eigen::MatrixXd& S);
  Eigen::MatrixXd calculateCrossCorrelation(Eigen::MatrixXd Zsig, const Eigen::VectorXd& z_pred, int n_z);
  Eigen::MatrixXd predictMeasurementCovariance(Eigen::MatrixXd Zsig, const Eigen::VectorXd& z_pred, int n_z);
  Eigen::VectorXd predictMeasurementMean(Eigen::MatrixXd Zsig, int n_z);
  Eigen::MatrixXd createSigmaPointsInMeasurementSpace(int n_z);
  void updateStateMeanAndCovariance(Eigen::MatrixXd K, const Eigen::VectorXd& z_pred, const MeasurementPackage& meas_package, Eigen::MatrixXd &S);

  // Helper functions for Lidar Update
  void updateStateMeanAndCovarianceLidar(Eigen::MatrixXd K, const Eigen::VectorXd& z_pred, const MeasurementPackage &meas_package, const Eigen::MatrixXd& S);
  Eigen::MatrixXd calculateCrossCorrelationLidar(Eigen::MatrixXd Zsig, const Eigen::VectorXd& z_pred, int n_z);
  Eigen::MatrixXd predictMeasurementCovarianceLidar(const Eigen::MatrixXd &Zsig, const Eigen::VectorXd &z_pred, int n_z);
  Eigen::VectorXd predictMeasurementMeanLidar(const Eigen::MatrixXd &Zsig, int n_z);
  Eigen::MatrixXd createSigmaPointsLidar(int z);
};

#endif // NFS_TRACKER_HPP
