//
// Created by Andrei on 29-Apr-23.
//

#include "scene.hpp"

Scene::Scene(pcl::visualization::PCLVisualizer::Ptr& viewer) {

  Car car1 = Car(
      Vect3(0, 0, 0),
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
  traffic.push_back(car1);

  renderRoad(0, viewer);
}

void Scene::stepScene(Car& egoCar, double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr &viewer) {
  viewer->removeShape(egoCar.getName());
  viewer->removeShape(egoCar.getName() + "front");

  egoCar.move(egoVelocity, timestamp);
  egoCar.render(viewer);

  for (auto &car : traffic) {
    viewer->removeShape(car.getName());
    viewer->removeShape(car.getName() + "front");
    car.move(egoVelocity, timestamp);
    car.render(viewer);
  }
}
