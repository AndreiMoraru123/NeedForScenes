//
// Created by Andrei on 29-Apr-23.
//

#ifndef NFS_SCENE_HPP
#define NFS_SCENE_HPP

#include "../Objects/car.hpp"
#include "../Objects/road.hpp"
#include "../Objects/obstacle.hpp"
#include "../Objects/parkingspot.hpp"
#include "../Toolkit/tools.hpp"
#include <random>
#include <chrono>

class Scene {
public:
  std::vector<Car> traffic;
  Road road;
  Tools tools;
  bool pass = true;
  bool win = false;
  std::vector<bool> trackCars = {true, true, true};
  std::vector<double> rmseThreshold = {1.20,1.50,2.50,0.9};
  std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};
  std::vector<Obstacle> obstacles;
  std::vector<ParkingSpot> parkingSpots;
  std::set<std::string> parkedSpots;
  bool visualize_lidar = true;
  bool visualize_radar = true;
  bool visualize_track = false;
  double projectedTime = 2;
  int projectedSteps = 6;
  explicit Scene(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void stepScene(Car& egoCar, double dt, long long timestamp, pcl::visualization::PCLVisualizer::Ptr& viewer);
  bool checkTrafficCollision(Car& egoCar);
  std::vector<Control> randomControlInstructions(std::mt19937& gen, Car& car, int numInstructions);
  Control randomControl(std::mt19937& gen, Car& car);
};

#endif // NFS_SCENE_HPP
