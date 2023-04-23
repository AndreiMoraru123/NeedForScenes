//
// Created by Andrei on 22-Apr-23.
//

#ifndef NFS_CAR_HPP
#define NFS_CAR_HPP
#include "structs.hpp"
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <utility>
#include <vector>

class Color {
public:
  float r, g, b;
  Color(float setR, float setG, float setB)
      : r(setR), g(setG), b(setB)
  {}
  void changeColor(float setR, float setG, float setB) {
    r = setR;
    g = setG;
    b = setB;
  }
};

class Vect3 {
public:
  float x, y, z;
  Vect3(float setX, float setY, float setZ)
      : x(setX), y(setY), z(setZ)
  {}
  Vect3 operator + (const Vect3& vec) const {
    Vect3 result(x + vec.x, y + vec.y, z + vec.z);
    return result;
  }
};

class Control {
public:
  long long time_us;
  float acceleration;
  float steering;
  Control(long long t, float acc, float s)
      : time_us(t), acceleration(acc), steering(s)
  {}
};

class Car {
  Vect3 position, dimensions;
  Eigen::Quaternionf orientation;
  std::string name;
  Color color;
  float velocity;
  float angle;
  float acceleration;
  float steering;
  float front_center_distance;
  float rolling_instace;
  std::vector<Control> controls;
  int control_index;
  double sin_angle;
  double cos_angle;
public:
  Car();
  Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity,
      float setAngle, float setAcceleration, float setSteering,
      float setFrontCenterDistance, std::string setName);

  void render(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void accelerate(float acc);
  void steer(float s);
  void control(const std::vector<Control>& c);
  void move(float dt, int time_us);
  [[nodiscard]] Vect3 getPosition() const;
  [[nodiscard]] bool checkCollision(Vect3 point);
};

#endif // NFS_CAR_HPP
