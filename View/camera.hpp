//
// Created by Andrei on 23-Apr-23.
//

#ifndef NFS_CAMERA_HPP
#define NFS_CAMERA_HPP

#pragma once

#include <pcl/visualization/pcl_visualizer.h>

enum CameraAngle {
  XY, TopDown, Side, FPS
};

void changeCameraView(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);

#endif // NFS_CAMERA_HPP
