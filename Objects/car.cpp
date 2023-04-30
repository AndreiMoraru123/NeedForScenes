//
// Created by Andrei on 23-Apr-23.
//
#include "car.hpp"

Car::Car()
    : position(Vect3(0, 0, 0)),
      dimensions(Vect3(0, 0, 0)),
      orientation(Eigen::Quaternionf(0, 0, 0, 0)),
      name("car"),
      color(Color(0.0f, 0.0f, 0.0f)),
      velocity(0),
      angle(0),
      acceleration(0),
      steering(0),
      rolling_instace(0),
      front_center_distance(0),
      control_index(0),
      sinNegAngle(0),
      cosNegAngle(0) {}

Car::Car(
    Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity,
    float setAngle, float setAcceleration, float setSteering,
    float setFrontCenterDistance, std::string setName
    ): position(setPosition),
      dimensions(setDimensions),
      color(setColor),
      velocity(setVelocity),
      angle(setAngle),
      acceleration(setAcceleration),
      steering(setSteering),
      front_center_distance(setFrontCenterDistance),
      name(std::move(setName)) {
  orientation = Eigen::Quaternionf(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
  sinNegAngle = sin(- angle);
  cosNegAngle = cos(- angle);
  acceleration = 0;
  steering = 0;
  control_index = 0;
  rolling_instace = 0.02;
}

void Car::render(pcl::visualization::PCLVisualizer::Ptr& viewer) {
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
  viewer->addCube(Eigen::Vector3f(position.x + front_center_distance * cosNegAngle,
                                  position.y + front_center_distance * sinNegAngle, position.z),
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

void Car::accelerate(float acc) {
  acceleration = acc;
}

void Car::steer(float s) {
  steering = s;
}

void Car::control(const std::vector<Control>& c) {
  for (auto& control : c) {
    controls.push_back(control);
  }
}

void Car::move(float dt, int time_us) {
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

[[nodiscard]] Vect3 Car::getPosition() const {return position;}
[[nodiscard]] std::string Car::getName() const {return name;}
[[nodiscard]] float Car::getVelocity() const {return velocity;}
[[nodiscard]] float Car::getAngle() const {return angle;}
[[nodiscard]] Tracker Car::getTracker() const {return tracker;}

void Car::setTracker(const Tracker& t) {tracker = t;}

static bool inbetween(double point, double center, double range) {
  return point >= center - range && point <= center + range;
}

[[nodiscard]] bool Car::checkCollision(Vect3 point) {
  double xPrime = ((point.x-position.x) * cosNegAngle - (point.y-position.y) * sinNegAngle)+position.x;
  double yPrime = ((point.y-position.y) * cosNegAngle + (point.x-position.x) * sinNegAngle)+position.y;
  return (
              inbetween(xPrime, position.x, dimensions.x / 2) &&
              inbetween(yPrime, position.y, dimensions.y / 2) &&
              inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
          (
              inbetween(xPrime, position.x, dimensions.x / 4) &&
              inbetween(yPrime, position.y, dimensions.y / 2) &&
              inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6)
          );
}