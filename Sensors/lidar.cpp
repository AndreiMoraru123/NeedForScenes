//
// Created by Andrei on 29-Apr-23.
//
#include "lidar.hpp"

Ray::Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle,
         double setResolution)
    : origin(setOrigin), resolution(setResolution),
      direction(resolution * cos(verticalAngle) * cos(horizontalAngle),
                resolution * cos(verticalAngle) * sin(horizontalAngle),
                resolution * sin(verticalAngle)),
      castPosition(origin), castDistance(0) {}

void Ray::rayCast(const std::vector<Car> &cars, double minDistance,
                  double maxDistance,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double slopeAngle,
                  double stdErr) {
  castPosition = origin;
  castDistance = 0;
  bool collision = false;

  while (!collision && castDistance < maxDistance &&
         (castPosition.y <= 6 && castPosition.y >= -6 && castPosition.x <= 50 &&
          castPosition.x >= -15)) {
    castPosition = castPosition + direction;
    castDistance += resolution;
    collision = (castPosition.z <= castPosition.x * tan(slopeAngle));
    if (!collision && castDistance < maxDistance) {
      for (Car car : cars) {
        collision |= car.checkCollision(castPosition);
        if (collision) {
          break;
        }
      }
    }
  }

  if ((castDistance >= minDistance) && (castDistance <= maxDistance) &&
      (castPosition.y <= 6 && castPosition.y >= -6 && castPosition.x <= 50 &&
       castPosition.x >= -15)) {
    double rx = ((double)rand() / (RAND_MAX));
    double ry = ((double)rand() / (RAND_MAX));
    double rz = ((double)rand() / (RAND_MAX));
    cloud->points.emplace_back(castPosition.x + rx * stdErr,
                               castPosition.y + ry * stdErr,
                               castPosition.z + rz * stdErr);
  }
}

Lidar::Lidar(std::vector<Car> setCars, double setGroundSlope)
    : cloud(new pcl::PointCloud<pcl::PointXYZ>), position(0, 0, 3.0) {

  minDistance = 0;
  maxDistance = 120;
  resolution = 0.2;
  stdErr = 0.02;
  cars = setCars;
  groundSlope = setGroundSlope;
  int numLayers = 64;
  double steepestAngle = -24.8 * M_PI / 180;
  double angleRange = 26.8 * M_PI / 180;
  double horizontalAngleInc = M_PI / 180;
  double angleIncrement = angleRange / numLayers;

  for (double angleVerical = steepestAngle;
       angleVerical < steepestAngle + angleRange;
       angleVerical += angleIncrement) {
    for (double angle = 0; angle <= 2 * M_PI; angle += horizontalAngleInc) {
      Ray ray(position, angle, angleVerical, resolution);
      rays.push_back(ray);
    }
  }
}

Lidar::~Lidar() { /**pcl deals with the memory via smart pointers */
}

void Lidar::updateCars(std::vector<Car> setCars) { cars = setCars; }

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar::scan() {
  cloud->points.clear();
  for (Ray ray : rays) {
    ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, stdErr);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  return cloud;
}