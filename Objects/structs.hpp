//
// Created by Andrei on 22-Apr-23.
//

#ifndef NFS_STRUCTS_HPP
#define NFS_STRUCTS_HPP

#include <Eigen/Geometry>

struct Box {
  float length;
  float width;
  float height;
  Eigen::Vector3f position;
  Eigen::Quaternionf orientation;
  std::vector<Eigen::Vector3f> vertices; // body
  std::vector<std::vector<int>> faces; // face
};

#endif // NFS_STRUCTS_HPP
