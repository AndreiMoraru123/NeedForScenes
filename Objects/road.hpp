//
// Created by Andrei on 23-Apr-23.
//
#include "car.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <random>

#ifndef NFS_ROAD_HPP
#define NFS_ROAD_HPP

class Road {
public:
  Road();
  void computeCoefficients();
  void computeDummies();
  void renderPoles(pcl::visualization::PCLVisualizer::Ptr &viewer) const;
  void renderLanes(pcl::visualization::PCLVisualizer::Ptr &viewer) const;
  void renderDummies(pcl::visualization::PCLVisualizer::Ptr &viewer) const;
  void render(pcl::visualization::PCLVisualizer::Ptr &viewer) const;
  std::vector<pcl::ModelCoefficients> getLaneCoefficients() const;

private:
  float arenaRadius;
  float laneWidth;
  int laneCount;
  float poleRadius;
  float poleHeight;
  float poleSpacing;
  std::vector<pcl::ModelCoefficients> laneCoefficients;
  std::vector<pcl::ModelCoefficients> poleCoefficients;
  std::vector<pcl::PointXYZ> parkingSpots;
  std::vector<std::pair<double, pcl::PointXYZ>> obstacles;
};

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      const std::string &name, Color color = Color(1, 1, 1));

#endif // NFS_ROAD_HPP
