//
// Created by Andrei on 23-Apr-23.
//

#include "road.hpp"

void renderRoad(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer) {

  // Define the properties of the arena
  float arenaRadius = 50.0;
  float laneWidth = 5.0;
  float laneCount = 2;
  float poleRadius = 0.5;
  float poleHeight = 3.0;
  float poleSpacing = 10.0; // spacing in between poles, poles start at 0;

  // Render the lanes
  for (int i = 0; i < laneCount; ++i) {
    pcl::ModelCoefficients circleCoeffs;
    circleCoeffs.values.resize(3);
    circleCoeffs.values[0] = 0; // center x
    circleCoeffs.values[1] = 0; // center y
    circleCoeffs.values[2] = arenaRadius - i * laneWidth; // radius
    viewer->addCircle(circleCoeffs, "lane" + std::to_string(i), 0);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "lane" + std::to_string(i)); // Yellow lanes
  }

  // Render the poles
  for (float angle = 0.0; angle <= 2 * M_PI; angle += 2 * M_PI / poleSpacing) {
    pcl::ModelCoefficients poleCoeffs;
    poleCoeffs.values.resize(7);
    poleCoeffs.values[0] = arenaRadius * std::cos(angle); // x position
    poleCoeffs.values[1] = arenaRadius * std::sin(angle); // y position
    poleCoeffs.values[2] = 0; // z position
    poleCoeffs.values[3] = 0; // direction x
    poleCoeffs.values[4] = 0; // direction y
    poleCoeffs.values[5] = poleHeight; // direction z
    poleCoeffs.values[6] = poleRadius; // radius
    viewer->addCylinder(poleCoeffs, "pole" + std::to_string(angle));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.5, 0, "pole" + std::to_string(angle)); // Orange poles
  }
  // Add dummy parking spots
  std::vector<pcl::PointXYZ> dummyParkingSpots = {
      pcl::PointXYZ(-15, -30, 0),
      pcl::PointXYZ(-15, 30, 0),
      pcl::PointXYZ(15, -30, 0),
      pcl::PointXYZ(15, 30, 0)
  };

  for (size_t i = 0; i < dummyParkingSpots.size(); ++i) {
    pcl::PointXYZ location = dummyParkingSpots[i];
    double parkingSpotWidth = 2.0;
    double parkingSpotLength = 4.0;
    double parkingSpotHeight = 0.1;
    viewer->addCube(location.x - parkingSpotLength / 2, location.x + parkingSpotLength / 2,
                    location.y - parkingSpotWidth / 2, location.y + parkingSpotWidth / 2,
                    location.z, location.z + parkingSpotHeight,
                    0.5, 0.5, 0.5, "dummyParkingSpot" + std::to_string(i));
  }
  // Add dummy obstacles
  std::vector<pcl::PointXYZ> dummyObstacles = {
      pcl::PointXYZ(-15, -15, 0),
      pcl::PointXYZ(-15, 15, 0),
      pcl::PointXYZ(15, -15, 0),
      pcl::PointXYZ(15, 15, 0)
  };

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> disPos(-35.0, 35.0);
  std::uniform_real_distribution<> disDim(1.5, 2.0);

  for (size_t i = 0; i < dummyObstacles.size(); ++i) {
    pcl::PointXYZ location = dummyObstacles[i];
    pcl::ModelCoefficients obstacleCoeffs;
    obstacleCoeffs.values.resize(7);
    obstacleCoeffs.values[0] = location.x; // x position
    obstacleCoeffs.values[1] = location.y; // y position
    obstacleCoeffs.values[2] = 0; // z position
    obstacleCoeffs.values[3] = 0; // direction x
    obstacleCoeffs.values[4] = 0; // direction y
    obstacleCoeffs.values[5] = 1; // direction z
    obstacleCoeffs.values[6] = disDim(gen); // dimensions
    viewer->addCube(location.x, location.x + disDim(gen),
                    location.y, location.y + disDim(gen),
                    location.z, location.z + disDim(gen),
                    1, 1, 0, "dummyObstacle" + std::to_string(i));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "dummyObstacle" + std::to_string(i));
  }
}


void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name, Color color) {
  viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}