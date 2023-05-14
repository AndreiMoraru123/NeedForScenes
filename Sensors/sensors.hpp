//
// Created by Andrei on 29-Apr-23.
//

#include <Eigen/Dense>

#ifndef NFS_SENSORS_HPP
#define NFS_SENSORS_HPP

class MeasurementPackage {
public:
  long long timeStamp;
  enum SensorType{
    LIDAR,
    RADAR
  } sensorType;

  Eigen::VectorXd rawMeasurements;
};

#endif // NFS_SENSORS_HPP
