//
// Created by Andrei on 23-Apr-23.
//
#include <pcl/visualization/pcl_visualizer.h>
#include "car.hpp"

#ifndef NFS_ROAD_HPP
#define NFS_ROAD_HPP

void renderRoad(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1, 1, 1));


#endif // NFS_ROAD_HPP
