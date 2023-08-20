//
// Created by Andrei on 30-Apr-23.
//

#ifndef NFS_TOOLS_HPP
#define NFS_TOOLS_HPP

#include "../Objects/car.hpp"
#include "../Sensors/sensors.hpp"
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <vector>

struct lidarMarker {
  double x, y;
  lidarMarker(double x, double y) : x(x), y(y) {}
};

struct radarMarker {
  double rho, phi, rhoDot;
  radarMarker(double rho, double phi, double rhoDot)
      : rho(rho), phi(phi), rhoDot(rhoDot) {}
};

class Tools {
public:
  Tools();
  virtual ~Tools();
  double noise(double stdDev, long long seedNum);
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> groundTruth;
  lidarMarker lidarSense(Car &car,
                         pcl::visualization::PCLVisualizer::Ptr &viewer,
                         long long timestamp, bool visualize);
  radarMarker radarSense(Car &car, Car ego,
                         pcl::visualization::PCLVisualizer::Ptr &viewer,
                         long long timestamp, bool visualize);
  void trackerResults(Car car, pcl::visualization::PCLVisualizer::Ptr &viewer,
                      double time, int steps);
  static Eigen::VectorXd
  calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                const std::vector<Eigen::VectorXd> &groundTruth);
  void savePcd(const typename pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
               std::string file);
  pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(const std::string &file);
};
#endif // NFS_TOOLS_HPP
