//
// Created by Andrei on 29-Apr-23.
//

#ifndef NFS_SCENE_HPP
#define NFS_SCENE_HPP

#include "../Objects/car.hpp"
#include "../Objects/road.hpp"

class Scene {
public:
  std::vector<Car> traffic;
  Car egoCar;

  std::vector<bool> track_cars = {true, true, true};
  bool visualize_lidar = true;
  bool visualize_radar = true;
  double projectedTime = 0;
  int projectedSteps = 0;

  Scene(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void stepScene(Car& egoCar, double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer);
};

#endif // NFS_SCENE_HPP
