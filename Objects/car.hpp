//
// Created by Andrei on 22-Apr-23.
//

#ifndef NFS_CAR_HPP
#define NFS_CAR_HPP
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <utility>
#include <vector>
#include "../Algo/tracker.hpp"

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
  Vect3 operator * (const float& scalar) const {
    Vect3 result(x * scalar, y * scalar, z * scalar);
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
public:
  Vect3 position, dimensions;
  Eigen::Quaternionf orientation;
  std::string name;
  Color color;
  Tracker tracker;
  float velocity;
  float angle;
  float acceleration;
  float steering;
  float front_center_distance;
  float rolling_instace;
  std::vector<Control> controls;
  int control_index;
  double sinNegAngle;
  double cosNegAngle;
public:
  Car();
  Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity,
      float setAngle, float setAcceleration, float setSteering,
      float setFrontCenterDistance, std::string setName);
  void render(pcl::visualization::PCLVisualizer::Ptr& viewer) const;
  void accelerate(float acc, int dir = 1);
  void steer(float s);
  void control(const std::vector<Control>& c);
  void move(float dt, int time_us);
  void setTracker(const Tracker& t);
  void setPosition(const Vect3& p);
  void setVelocity(float v);
  [[nodiscard]] Vect3& getPosition();
  [[nodiscard]] float& getVelocity();
  [[nodiscard]] float& getAngle();
  [[nodiscard]] std::string& getName();
  [[nodiscard]] Tracker& getTracker();
  [[nodiscard]] Vect3 getDirection();
  [[nodiscard]] bool checkCollision(Vect3 point);
};

#endif // NFS_CAR_HPP
