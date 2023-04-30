//
// Created by Andrei on 30-Apr-23.
//
#include <iostream>
#include <random>
#include "tools.hpp"

Tools::Tools() = default;
Tools::~Tools() = default;

double Tools::noise(double stdDev, long long seedNum) {
  std::mt19937::result_type seed = seedNum;
  auto dist = std::bind(std::normal_distribution<double>{0, stdDev}, std::mt19937(seed));
  return dist();
}

lidarMarker Tools::lidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize) {
  MeasurementPackage measPackage;
  measPackage.sensorType = MeasurementPackage::LASER;
  measPackage.rawMeasurements = Eigen::VectorXd(2);
  lidarMarker marker = lidarMarker(car.getPosition().x + noise(0.15, timestamp), car.getPosition().y + noise(0.15, timestamp + 1));
  if (visualize) {
    viewer->addSphere(pcl::PointXYZ(marker.x, marker.y, 3.0), 0.5, 1, 0, 0, car.getName() + "_lmarker");
  }
  measPackage.rawMeasurements << marker.x, marker.y;
  measPackage.timeStamp = timestamp;
//  car.ukf.processMeasurement(measPackage);
  return marker;
}

radarMarker Tools::radarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize) {
  double rho = sqrt((car.getPosition().x - ego.getPosition().x) * (car.getPosition().x - ego.getPosition().x) +
                    (car.getPosition().y - ego.getPosition().y) * (car.getPosition().y - ego.getPosition().y));
  double phi = atan2(car.getPosition().y - ego.getPosition().y, car.getPosition().x - ego.getPosition().x);
  double rhotDot = (car.getVelocity() * cos(car.getAngle()) * rho * cos(phi) +
                    car.getVelocity() * sin(car.getAngle()) * rho * sin(phi)) / rho;

  radarMarker marker = radarMarker(
      rho + noise(0.3, timestamp + 2),
      phi + noise(0.03, timestamp + 3),
      rhotDot + noise(0.3, timestamp + 4));

if (visualize) {
    viewer->addLine(pcl::PointXYZ(ego.getPosition().x, ego.getPosition().y, 3.0),
                    pcl::PointXYZ(car.getPosition().x, car.getPosition().y, 3.0),
                    1, 0, 0, car.getName() + "_rho");
    viewer->addArrow(
        pcl::PointXYZ(
            ego.getPosition().x + marker.rho * cos (marker.phi),
            ego.getPosition().y + marker.rho * sin(marker.phi),
            3.0
            ),
        pcl::PointXYZ(
            car.getPosition().x + marker.rho * cos(marker.phi) + marker.rhoDot * cos(marker.phi),
            ego.getPosition().y + marker.rho * sin(marker.phi) + marker.rhoDot * sin(marker.phi),
            3.0
            ), 1, 0, 0, car.getName() + "_rhoDot");
  }

  MeasurementPackage measPackage;
  measPackage.sensorType = MeasurementPackage::RADAR;
  measPackage.rawMeasurements = Eigen::VectorXd(3);
  measPackage.rawMeasurements << marker.rho, marker.phi, marker.rhoDot;
  measPackage.timeStamp = timestamp;
//  car.ukf.processMeasurement(measPackage);
  return marker;
}

void Tools::ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr &viewer, double time, int steps) {
//  UKF ukf = car.tracker;
//  viewer->addSphere(pcl::PointXYZ(ukf.getPosition().x, ukf.getPosition().y, 3.0), 0.5, 0, 1, 0, car.getName() + "_ukf");
  if (time > 0) {
    double dt = time / steps;
    double ct = dt;
    while (ct <= time) {
//      ukf.Predict(dt);
//      viewer->addSphere(pcl::PointXYZ(ukf.getPosition().x, ukf.getPosition().y, 3.0), 0.5, 0, 1, 0, car.getName() + "_ukf");
      ct += dt;
    }
  }
}

Eigen::VectorXd Tools::calculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &groundTruth) {

  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() != groundTruth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground truth data" << endl;
    return rmse;
  }

  for (unsigned int i = 0; i < estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - groundTruth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

void Tools::savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename) {
  pcl::io::savePCDFileASCII(filename, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " << filename << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Tools::loadPcd(std::string file) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1) {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " << file << std::endl;
  return cloud;
}