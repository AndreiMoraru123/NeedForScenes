//
// Created by Andrei on 29-Apr-23.
//

#include "scene.hpp"

Scene::Scene(pcl::visualization::PCLVisualizer::Ptr& viewer) {

  tools = Tools();
  Car car1 = Car(
      Vect3(-10, 4, 0),
      Vect3(4, 2, 2),
      Color(1, 0, 0),
      5, 0, 2,
      2, 2,
      "car1"
  );

  std::vector<Control> car1_instructions;
  Control a = Control(0.5*1e6, 0.5, 0.0);
  car1_instructions.push_back(a);
  a = Control(2.2*1e6, 0.0, -0.2);
  car1_instructions.push_back(a);
  a = Control(3.3*1e6, 0.0, 0.2);
  car1_instructions.push_back(a);
  a = Control(4.4*1e6, -2.0, 0.0);
  car1_instructions.push_back(a);
  car1.control(car1_instructions);

  if (trackCars[0]) {
    Tracker tracker1;
    car1.setTracker(tracker1);
  }
  traffic.push_back(car1);
  renderRoad(0, viewer);
  car1.render(viewer);
}

void Scene::stepScene(Car& egoCar, double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer) {

  renderRoad(egoVelocity * timestamp / 1e6, viewer);
  egoCar.move(egoVelocity, timestamp);
  egoCar.render(viewer);

  for (size_t i = 0; i < traffic.size(); ++i) {
    Car& car = traffic[i];
    viewer->removeShape(car.getName());
    viewer->removeShape(car.getName() + "front");
    car.move(egoVelocity, timestamp);
    car.render(viewer);

    if (!visualize_pcd) {}

    if (trackCars[i]) {
      Eigen::VectorXd gt(4);
      gt << car.getPosition().x, car.getPosition().y,
            car.getVelocity() * cos(car.getAngle()),
            car.getVelocity() * sin(car.getAngle());

      tools.groundTruth.push_back(gt);
      tools.lidarSense(car, viewer, timestamp, visualize_lidar);
      tools.radarSense(egoCar, car, viewer, timestamp, visualize_radar);
      tools.trackerResults(car, viewer, projectedTime, projectedSteps);
      Eigen::VectorXd estimate(4);
      double v = traffic[i].getTracker().x_(2);
      double yaw = traffic[i].getTracker().x_(3);
      double vx = cos(yaw)*v;
      double vy = sin(yaw)*v;
      estimate << traffic[i].getTracker().x_[0], traffic[i].getTracker().x_[1], vx, vy;
      tools.estimations.push_back(estimate);
    }
  }

  viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
  Eigen::VectorXd rmse = tools.calculateRMSE(tools.estimations, tools.groundTruth);
  viewer->addText(" X: "+std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
  viewer->addText(" Y: "+std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
  viewer->addText("Vx: "	+std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
  viewer->addText("Vy: "	+std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

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
