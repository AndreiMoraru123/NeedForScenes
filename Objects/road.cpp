//
// Created by Andrei on 23-Apr-23.
//

#include "road.hpp"

Road::Road()
    : arenaRadius(50.0), laneWidth(5.0), laneCount(2), poleRadius(0.5),
      poleHeight(3.0), poleSpacing(10.0) {
  computeCoefficients();
  computeDummies();
}

void Road::computeCoefficients() {

  for (int i = 0; i < laneCount; ++i) {
    pcl::ModelCoefficients circleCoeffs;
    circleCoeffs.values.resize(3);
    circleCoeffs.values[0] = 0;                           // center x
    circleCoeffs.values[1] = 0;                           // center y
    circleCoeffs.values[2] = arenaRadius - i * laneWidth; // radius
    laneCoefficients.push_back(circleCoeffs);
  }

  for (float angle = 0.0; angle <= 2 * M_PI; angle += 2 * M_PI / poleSpacing) {
    pcl::ModelCoefficients poleCoeffs;
    poleCoeffs.values.resize(7);
    poleCoeffs.values[0] = arenaRadius * std::cos(angle); // x position
    poleCoeffs.values[1] = arenaRadius * std::sin(angle); // y position
    poleCoeffs.values[2] = 0;                             // z position
    poleCoeffs.values[3] = 0;                             // direction x
    poleCoeffs.values[4] = 0;                             // direction y
    poleCoeffs.values[5] = poleHeight;                    // direction z
    poleCoeffs.values[6] = poleRadius;                    // radius
    poleCoefficients.push_back(poleCoeffs);
  }
}

void Road::renderLanes(pcl::visualization::PCLVisualizer::Ptr &viewer) const {
  for (int i = 0; i < laneCount; ++i) {
    viewer->addCircle(laneCoefficients[i], "lane" + std::to_string(i), 0);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0,
        "lane" + std::to_string(i)); // Yellow lanes
  }
}

void Road::renderPoles(pcl::visualization::PCLVisualizer::Ptr &viewer) const {
  for (size_t i = 0; i < poleCoefficients.size(); ++i) {
    viewer->addCylinder(poleCoefficients[i], "pole" + std::to_string(i));
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.5, 0,
        "pole" + std::to_string(i)); // Orange poles
  }
}

void Road::computeDummies() {

  std::vector<pcl::PointXYZ> dummyObstacles = {
      pcl::PointXYZ(-15, -15, 0), pcl::PointXYZ(-15, 15, 0),
      pcl::PointXYZ(15, -15, 0), pcl::PointXYZ(15, 15, 0)};

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> disDim(1.5, 2.0);
  std::uniform_real_distribution<> disLoc(-30.0, 30.0);

  std::vector<pcl::PointXYZ> dummyParkingSpots = {
      pcl::PointXYZ(disLoc(gen), -disLoc(gen), 0),
      pcl::PointXYZ(-disLoc(gen), disLoc(gen), 0),
      pcl::PointXYZ(disLoc(gen), -disLoc(gen), 0),
      pcl::PointXYZ(disLoc(gen), disLoc(gen), 0)};

  for (const auto &location : dummyObstacles) {
    double dimension = disDim(gen);
    obstacles.emplace_back(dimension, location);
  }

  for (auto &location : dummyParkingSpots) {
    parkingSpots.push_back(location);
  }
}

void Road::renderDummies(pcl::visualization::PCLVisualizer::Ptr &viewer) const {
  double parkingSpotWidth = 2.0;
  double parkingSpotLength = 4.0;
  double parkingSpotHeight = 0.1;

  for (size_t i = 0; i < parkingSpots.size(); ++i) {
    pcl::PointXYZ location = parkingSpots[i];
    viewer->addCube(
        location.x - parkingSpotLength / 2, location.x + parkingSpotLength / 2,
        location.y - parkingSpotWidth / 2, location.y + parkingSpotWidth / 2,
        location.z, location.z + parkingSpotHeight, 0.5, 0.5, 0.5,
        "dummyParkingSpot" + std::to_string(i));
  }

  for (size_t i = 0; i < obstacles.size(); ++i) {
    double dimension = obstacles[i].first;
    pcl::PointXYZ location = obstacles[i].second;
    viewer->addCube(location.x, location.x + dimension, location.y,
                    location.y + dimension, location.z, location.z + dimension,
                    1, 1, 0, "dummyObstacle" + std::to_string(i));
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0,
        "dummyObstacle" + std::to_string(i));
  }
}

void Road::render(pcl::visualization::PCLVisualizer::Ptr &viewer) const {
  renderLanes(viewer);
  renderPoles(viewer);
  renderDummies(viewer);
}

std::vector<pcl::ModelCoefficients> Road::getLaneCoefficients() const {
  return laneCoefficients;
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      const std::string &name, Color color) {
  viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
      name);
}