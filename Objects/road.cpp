//
// Created by Andrei on 23-Apr-23.
//

#include "road.hpp"

void renderRoad(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer) {
  float roadLengthAhead = 50.0;
  float roadLengthBehind = -15.0;
  float roadWidth = 12.0;
  float roadHeight = 0.2;

  viewer->addCube(roadLengthBehind, roadLengthAhead, -roadWidth / 2, roadWidth / 2, -roadHeight, 0, .2, .2, .2, "road");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "road");
  viewer->addLine(pcl::PointXYZ(roadLengthBehind, -roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead , -roadWidth / 6, 0.01), 1, 1, 0, "line1");
  viewer->addLine(pcl::PointXYZ(roadLengthBehind, roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead, roadWidth / 6, 0.01), 1, 1, 0, "line2");

  double poleSpace = 10; // spacing in between poles, poles start at 0;
  double poleCurve = 4; // pole distance from road curve
  double poleWidth = 0.5;
  double poleHeight = 3;

  double markerPos = (roadLengthBehind / poleSpace) * poleSpace - distancePos;
  while (markerPos < roadLengthBehind) {
    markerPos += poleSpace;
  }
  int poleIndex = 0;
  while (markerPos <= roadLengthAhead) {
    // left pole
    viewer->addCube(-poleWidth / 2 + markerPos, poleWidth / 2 + markerPos, - poleWidth / 2 + roadWidth / 2 + poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"l");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"l");
    viewer->addCube(-poleWidth / 2 + markerPos, poleWidth / 2 + markerPos, - poleWidth / 2 + roadWidth / 2 + poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"lframe");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"lframe");

    // right pole
    viewer->addCube(-poleWidth / 2 + markerPos, poleWidth / 2 + markerPos, - poleWidth / 2 - roadWidth / 2 - poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"r");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"r");
    viewer->addCube(-poleWidth / 2 + markerPos, poleWidth / 2 + markerPos, -poleWidth / 2 - roadWidth / 2 - poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"rframe");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"rframe");

    markerPos += poleSpace;
    poleIndex++;
  }
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color) {
  viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}