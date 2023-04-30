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
  std::vector<bool> trackCars = {true};
  bool visualize_lidar = true;
  bool visualize_radar = true;
  bool visualize_pcd = false;
  double projectedTime = 0;
  int projectedSteps = 0;

  explicit Scene(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void stepScene(Car& egoCar, double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer);
};

#endif // NFS_SCENE_HPP
