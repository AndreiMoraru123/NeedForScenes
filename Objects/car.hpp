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
  Car(): position(Vect3(0, 0, 0)), dimensions(Vect3(0, 0, 0)),
         orientation(Eigen::Quaternionf(0, 0, 0, 0)),
         name("car"), color(Color(0.0f, 0.0f, 0.0f)),
         velocity(0), angle(0), acceleration(0), steering(0), rolling_instace(0),
         front_center_distance(0), control_index(0), sin_angle(0), cos_angle(0) {}
  Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity,
      float setAngle, float setAcceleration, float setSteering,
      float setFrontCenterDistance, std::string setName)
      : position(setPosition), dimensions(setDimensions), color(setColor),
        velocity(setVelocity), angle(setAngle), acceleration(setAcceleration),
        steering(setSteering), front_center_distance(setFrontCenterDistance), name(std::move(setName)) {
    orientation = Eigen::Quaternionf(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    sin_angle = sin(angle);
    cos_angle = cos(angle);
    acceleration = 0;
    steering = 0;
    control_index = 0;
    rolling_instace = 0.02;
  }

  void render(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    viewer->addCube(Eigen::Vector3f(position.x, position.y, position.z),
                    orientation,
                    dimensions.x, dimensions.y, dimensions.z,
                    name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        color.r, color.g, color.b,
                                        name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        name);
    viewer->addCube(Eigen::Vector3f(position.x + front_center_distance * cos_angle,
                                              position.y + front_center_distance * sin_angle, position.z),
                    orientation,
                    dimensions.x, dimensions.y, dimensions.z,
                    name + "front");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        color.r, color.g, color.b,
                                        name + "front");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        name + "front");
  }

  void accelerate(float acc) {
    acceleration = acc;
  }

  void steer(float s) {
    steering = s;
  }

  void control(const std::vector<Control>& c) {
    for (auto& control : c) {
      controls.push_back(control);
    }
  }

  void move(float dt, int time_us) {
    if (!controls.empty() && control_index < (int) controls.size() - 1) {
      if (time_us >= controls[control_index + 1].time_us) {
        accelerate(controls[control_index + 1].acceleration);
        steer(controls[control_index + 1].steering);
        control_index++;
      }
    }

    position.x += velocity * cos(angle) * dt;
    position.y += velocity * sin(angle) * dt;

    angle += velocity * steering * dt / front_center_distance;
    orientation = Eigen::Quaternionf(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    velocity += acceleration * dt;
    sin_angle = sin(angle);
    cos_angle = cos(angle);

    // Apply rolling instance if there is no acceleration input
    if (acceleration == 0) {
      if (velocity > 0) {
        velocity -= rolling_instace;
        if (velocity < 0) {
          velocity = 0;
        }
      } else if (velocity < 0) {
        velocity += rolling_instace;
        if (velocity > 0) {
          velocity = 0;
        }
      }
    }
  }

  [[nodiscard]] Vect3 getPosition() const {
    return position;
  }

  static bool inbetween(double point, double center, double range) {
    return point >= center - range && point <= center + range;
  }

  [[nodiscard]] bool checkCollision(Vect3 point) const {
    double xPrime = (point.x - position.x) * cos_angle + (point.y - position.y) * sin_angle + position.x;
    double yPrime = -(point.x - position.x) * sin_angle + (point.y - position.y) * cos_angle + position.y;
    return (inbetween(xPrime, position.x, dimensions.x / 2) &&
            inbetween(yPrime, position.y, dimensions.y / 2) &&
            inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
           (inbetween(xPrime, position.x, dimensions.x / 4) &&
            inbetween(yPrime, position.y, dimensions.y / 2) &&
            inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));
  }
};

#endif // NFS_CAR_HPP
