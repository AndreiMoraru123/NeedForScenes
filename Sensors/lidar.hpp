//
// Created by Andrei on 29-Apr-23.
//

#ifndef NFS_LIDAR_HPP
#define NFS_LIDAR_HPP

#include "../Objects/car.hpp"

class Ray {
public:
  Vect3 origin;
  double resolution;
  Vect3 direction;
  Vect3 castPosition;
  double castDistance;

  Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle,
      double setResolution);
  void rayCast(const std::vector<Car> &cars, double minDistance,
               double maxDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
               double slopeAngle, double stdErr);
};

class Lidar {
public:
  std::vector<Ray> rays;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<Car> cars;
  Vect3 position;
  double groundSlope;
  double minDistance;
  double maxDistance;
  double resolution;
  double stdErr;
  Lidar(std::vector<Car> setCars, double setGroundSlope);
  ~Lidar();
  void updateCars(std::vector<Car> setCars);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan();
};

#endif // NFS_LIDAR_HPP
