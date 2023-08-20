//
// Created by Andrei on 01-May-23.
//

#ifndef NFS_CONTROLLER_HPP
#define NFS_CONTROLLER_HPP

#include "../Objects/car.hpp"
#include "../Scene/scene.hpp"
#include "../View/camera.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_map>

class EgoCarController {
public:
  EgoCarController(pcl::visualization::PCLVisualizer::Ptr &viewer, Car &car);
  void registerKeyboardCallbacks();
  void handleKeyboardInput();
  void update(float dt, Scene &scene);
  void stop();

private:
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  Car &car_;
  std::unordered_map<std::string, bool> keyStates_;
  CameraAngle currentAngle_;
  std::vector<CameraAngle> angles_;
  int timeUs_;
  const int intervalUs_ = 100000;
  void changeView(CameraAngle angle);
};

#endif // NFS_CONTROLLER_HPP
