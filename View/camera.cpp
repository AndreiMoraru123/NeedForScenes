//
// Created by Andrei on 22-Apr-23.
//
#include "camera.hpp"

void changeCameraView(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
  int distance = 20;

  switch (setAngle) {
  case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
  case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
  case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
  case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
  }
}