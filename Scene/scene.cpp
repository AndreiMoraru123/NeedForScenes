//
// Created by Andrei on 29-Apr-23.
//

#include "scene.hpp"

Scene::Scene(pcl::visualization::PCLVisualizer::Ptr& viewer) {

  tools = Tools();
  std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<> disPos(-35.0, 35.0);
  std::uniform_real_distribution<> disDim(1.5, 2.0);

  for (int i = 0; i <= 5; ++i) {
    Vect3 position(disPos(gen), disPos(gen), 0.0);
    Vect3 dimensions(disDim(gen), disDim(gen), disDim(gen));
    Obstacle obstacle(position, dimensions);
    obstacle.setName("obstacle" + std::to_string(i));
    obstacles.push_back(obstacle);
  }

  parkingSpots.emplace_back(pcl::PointXYZ(-30, -30, 0), 4.0, 2.0, 0.1, 1);
  parkingSpots.back().setName("parkingSpot1");
  parkingSpots.emplace_back(pcl::PointXYZ(-30, 30, 0), 4.0, 2.0, 0.1, 2);
  parkingSpots.back().setName("parkingSpot2");
  parkingSpots.emplace_back(pcl::PointXYZ(30, -30, 0), 4.0, 2.0, 0.1, 3);
  parkingSpots.back().setName("parkingSpot3");
  parkingSpots.emplace_back(pcl::PointXYZ(30, 30, 0), 4.0, 2.0, 0.1, 4);
  parkingSpots.back().setName("parkingSpot4");

  Car car1 = Car(
      Vect3(-10, 4, 0),  // Position
      Vect3(4, 2, 2),    // Dimensions
      Color(1, 0, 0),    // Color
      10,                // Angle
      2,                 // Front Center Distance
      "car1"             // Name
  );

  Car car2 = Car(
      Vect3(25, -40, 0), // Position
      Vect3(4, 2, 2),    // Dimensions
      Color(0, 0, 1),    // Color
      -6,                // Angle
      2,                 // Front Center Distance
      "car2"             // Name
  );

  Car car3 = Car(
      Vect3(-12, 30, 0), // Position
      Vect3(4, 2, 2),    // Dimensions
      Color(1, 1, 1),    // Color
      2,                 // Angle
      2,                 // Front Center Distance
      "car3"             // Name
  );


  if (trackCars[0]) {
    Tracker tracker1;
    car1.setTracker(tracker1);
  }

  if (trackCars[1]) {
    Tracker tracker2;
    car2.setTracker(tracker2);
  }

  if (trackCars[2]) {
    Tracker tracker3;
    car3.setTracker(tracker3);
  }

  int numInstructions = 20;
  car1.control(randomControlInstructions(gen, car1, numInstructions));
  car2.control(randomControlInstructions(gen, car2, numInstructions));
  car3.control(randomControlInstructions(gen, car3, numInstructions));

  traffic.push_back(car1);
  traffic.push_back(car2);
  traffic.push_back(car3);
  car1.render(viewer);
  car2.render(viewer);
  car3.render(viewer);
}

Control Scene::randomControl(std::mt19937& gen, Car& car) {
  std::uniform_real_distribution<> disTime(1, 9);
  std::uniform_real_distribution<> disAcceleration(-5, 2);
  std::uniform_real_distribution<> disSteering(-0.15, 0.15);

  Vect3 currentPosition = car.getPosition();

  double time;
  double acceleration;
  double steering;

  bool withinBounds;
  do {
    time = disTime(gen);
    acceleration = disAcceleration(gen);
    steering = disSteering(gen);

    Vect3 newPosition = currentPosition + Vect3(time * acceleration * cos(steering),
                                                time * acceleration * sin(steering),
                                                0);
    withinBounds = true;
    for (const auto& coeff : road.getLaneCoefficients()) {
      double distance = sqrt(
          pow(newPosition.x - coeff.values[0], 2) +
          pow(newPosition.y - coeff.values[1], 2));
      if (distance < coeff.values[2] / 2) {
        withinBounds = false;
        break;
      }
    }
  } while (!withinBounds);

  return Control(time, acceleration, steering);
}

std::vector<Control> Scene::randomControlInstructions(std::mt19937& gen, Car& car, int numInstructions) {
  std::vector<Control> instructions;

  for(int i=0; i<numInstructions; ++i) {
    instructions.push_back(randomControl(gen, car));
  }

  return instructions;
}

void Scene::stepScene(Car& egoCar, double dt, long long timestamp, pcl::visualization::PCLVisualizer::Ptr& viewer) {

  road.render(viewer);
  for (const ParkingSpot& parkingSpot : parkingSpots) {
    parkingSpot.render(viewer);
    if(parkingSpot.isCarParked(egoCar)) {
      parkedSpots.insert(parkingSpot.getName());
      viewer->addText("Parked!", 200, 200, 20, 1, 1, 1, "parkingText");
    }
  }

  if (parkedSpots.size() == parkingSpots.size()) {
    win = true;
    viewer->addText("Parked on all spots!", 200, 200, 20, 1, 1, 1, "completeParkingText");
  }

  egoCar.move(dt, timestamp);
  egoCar.render(viewer);

  Car predictedEgoCar = egoCar;
  predictedEgoCar.move(dt, timestamp);

  for (const Obstacle& obstacle: obstacles) {
    obstacle.render(viewer);
    if (obstacle.checkCollision(predictedEgoCar)) {
      viewer->addText("Crashed!", 200, 200, 20, 1, 1, 1, "collisionText");
      egoCar.setVelocity(0.0);
      Vect3 currentPos = egoCar.getPosition();
      Vect3 backStep = egoCar.getDirection() * -0.5;
      egoCar.setPosition(currentPos + backStep);
    }
  }

  for (size_t i = 0; i < traffic.size(); ++i) {
    Car& car = traffic[i];
    viewer->removeShape(car.getName());
    viewer->removeShape(car.getName() + "front");
    car.move(dt, timestamp);
    car.render(viewer);

    if (!visualize_pcd) {}

    if (trackCars[i]) {
      Eigen::VectorXd gt(4);
      gt << car.getPosition().x, car.getPosition().y,
            car.getVelocity() * cos(car.getAngle()),
            car.getVelocity() * sin(car.getAngle());

      tools.groundTruth.push_back(gt);
      tools.lidarSense(car, viewer, timestamp, visualize_lidar);
      tools.radarSense(car, egoCar, viewer, timestamp, visualize_radar);
      tools.trackerResults(car, viewer, projectedTime, projectedSteps);
      Eigen::VectorXd estimate(4);
      double v = traffic[i].getTracker().x_(2);
      double yaw = traffic[i].getTracker().x_(3);
      double vx = cos(yaw)*v;
      double vy = sin(yaw)*v;
      estimate << traffic[i].getTracker().x_[0], traffic[i].getTracker().x_[1], vx, vy;
      tools.estimations.push_back(estimate);

      Car estimatedCar = car;
      estimatedCar.setPosition(Vect3(estimate[0], estimate[1], car.getPosition().z));
      if (predictedEgoCar.checkCollision(estimatedCar.getPosition())) {
        viewer->addText("Crashed!", 200, 200, 20, 1, 1, 1, "collisionText");
        egoCar.setVelocity(0.0);
        Vect3 currentPos = egoCar.getPosition();
        Vect3 backStep = egoCar.getDirection() * -0.5;
        egoCar.setPosition(currentPos + backStep);
      }
    }
  }

  viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
  Eigen::VectorXd rmse = Tools::calculateRMSE(tools.estimations, tools.groundTruth);
  viewer->addText(" X: " + std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
  viewer->addText(" Y: " + std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
  viewer->addText("Vx: " + std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
  viewer->addText("Vy: " + std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

  if(timestamp > 1.0e6)
  {

    if(rmse[0] > rmseThreshold[0])
    {
      rmseFailLog[0] = rmse[0];
      pass = false;
    }
    if(rmse[1] > rmseThreshold[1])
    {
      rmseFailLog[1] = rmse[1];
      pass = false;
    }
    if(rmse[2] > rmseThreshold[2])
    {
      rmseFailLog[2] = rmse[2];
      pass = false;
    }
    if(rmse[3] > rmseThreshold[3])
    {
      rmseFailLog[3] = rmse[3];
      pass = false;
    }
  }
  if(!pass)
  {
    viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
    if(rmseFailLog[0] > 0)
      viewer->addText(" X: "+std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
    if(rmseFailLog[1] > 0)
      viewer->addText(" Y: "+std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
    if(rmseFailLog[2] > 0)
      viewer->addText("Vx: "+std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
    if(rmseFailLog[3] > 0)
      viewer->addText("Vy: "+std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
  }
}

bool Scene::checkTrafficCollision(Car& egoCar) {
  for (Car& car : traffic) {
    if (egoCar.checkCollision(car.getPosition())) {
      return true;
    }
  }
  return false;
}
