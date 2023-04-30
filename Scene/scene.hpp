//
// Created by Andrei on 29-Apr-23.
//

#ifndef NFS_SCENE_HPP
#define NFS_SCENE_HPP

#include "../Objects/car.hpp"
#include "../Objects/road.hpp"
#include "../Toolkit/tools.hpp"

class Scene {
public:
  std::vector<Car> traffic;
  Tools tools;
  bool pass = true;
  std::vector<bool> trackCars = {true};
  std::vector<double> rmseThreshold = {0.30,0.16,0.95,0.70};
  std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};
  bool visualize_lidar = true;
  bool visualize_radar = true;
  bool visualize_pcd = false;
  double projectedTime = 0;
  int projectedSteps = 0;

  explicit Scene(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void stepScene(Car& egoCar, double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer);
};

#endif // NFS_SCENE_HPP
