//
// Created by Andrei on 29-Apr-23.
//

#ifndef NFS_SCENE_HPP
#define NFS_SCENE_HPP

#include "../Objects/car.hpp"
#include "../Objects/road.hpp"
#include "../Objects/obstacle.hpp"
#include "../Objects/parkingspot.hpp"
#include "../Toolkit/tools.hpp"
#include <random>

class Scene {
public:
  std::vector<Car> traffic;
  Tools tools;
  bool pass = true;
  std::vector<bool> trackCars = {true, true, true};
  std::vector<double> rmseThreshold = {1.20,1.50,2.50,0.9};
  std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};
  std::vector<Obstacle> obstacles;
  std::vector<ParkingSpot> parkingSpots;
  bool visualize_lidar = true;
  bool visualize_radar = true;
  bool visualize_pcd = false;
  double projectedTime = 0;
  int projectedSteps = 0;
  explicit Scene(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void stepScene(Car& egoCar, double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer);
  bool checkTrafficCollision(Car& egoCar);
};

#endif // NFS_SCENE_HPP
