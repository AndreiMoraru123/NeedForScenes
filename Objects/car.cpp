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
      steering(0), rollingInstance(0), frontCenterDistance(0),
      control_index(0),
      sinNegAngle(0),
      cosNegAngle(0) {}

Car::Car(
    Vect3 setPosition, Vect3 setDimensions, Color setColor,
    float setAngle, float setFrontCenterDistance, std::string setName
    ): position(setPosition),
      dimensions(setDimensions),
      color(setColor),
      angle(setAngle), frontCenterDistance(setFrontCenterDistance),
      name(std::move(setName)) {
  orientation = Eigen::Quaternionf(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
  sinNegAngle = sin(- angle);
  cosNegAngle = cos(- angle);
  acceleration = 0;
  steering = 0;
  velocity = 0;
  control_index = 0;
  rollingInstance = 0.5;
}

void Car::renderBottom(pcl::visualization::PCLVisualizer::Ptr& viewer) const {
  viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z * 1 / 3),
                  orientation, dimensions.x, dimensions.y, dimensions.z * 2 / 3,
                  name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b, name);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
}

void Car::renderTop(pcl::visualization::PCLVisualizer::Ptr& viewer) const {
  viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z * 5 / 6),
                  orientation, dimensions.x / 2, dimensions.y,
                  dimensions.z * 1 / 3, name + "Top");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b, name + "Top");
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "Top");
}

Eigen::Vector3f Car::generateWindshieldPosition(int i) const {
  Eigen::Vector3f localPosition(i * dimensions.x / 4, 0, dimensions.z * 2 / 3);
  Eigen::Quaternionf quaternion(orientation);
  Eigen::AngleAxisf rotation(quaternion);
  Eigen::Vector3f rotatedPosition = rotation * localPosition;
  Eigen::Vector3f positionEigen(position.x, position.y, position.z);
  return positionEigen + rotatedPosition;
}

pcl::ModelCoefficients Car::generateWheelCoefficients(int i, int j) const {
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(position.x + i * dimensions.x / 5); // x position
  coeffs.values.push_back(position.y + j * dimensions.y / 2); // y position
  coeffs.values.push_back(position.z + dimensions.z / 7); // z position
  coeffs.values.push_back(1); // Direction x
  coeffs.values.push_back(0); // Direction y
  coeffs.values.push_back(0); // Direction z
  coeffs.values.push_back(dimensions.z * 1 / 5); // Radius
  return coeffs;
}

void Car::renderWheels(pcl::visualization::PCLVisualizer::Ptr &viewer) const {
  for(int i=-1; i<=1; i+=2) {
    for(int j=-1; j<=1; j+=2) {
      pcl::ModelCoefficients coeffs = generateWheelCoefficients(i, j);
      viewer->addCylinder(coeffs, name + "Wheel" + std::to_string(i) + std::to_string(j));
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                          0, 0, 0, name + "Wheel" + std::to_string(i) + std::to_string(j));
    }
  }
}

void Car::renderWindshields(pcl::visualization::PCLVisualizer::Ptr &viewer) const {
  for(int i=-1; i<=1; i+=2) {
    Eigen::Vector3f globalPosition = generateWindshieldPosition(i);
    viewer->addCube(globalPosition, orientation, dimensions.x / 4, dimensions.y, dimensions.z / 6, name + "Windshield" + std::to_string(i));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        0.3, 0.7, 1.0, name + "Windshield" + std::to_string(i));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        name + "Windshield" + std::to_string(i));
  }
}


void Car::render(pcl::visualization::PCLVisualizer::Ptr& viewer) const {
  renderBottom(viewer);
  renderTop(viewer);
  renderWheels(viewer);
  renderWindshields(viewer);
}


void Car::accelerate(float acc, int dir) {
  acceleration = acc * dir;
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
    if (time_us >= controls[control_index + 1].timeUs) {
      accelerate(controls[control_index + 1].acceleration);
      steer(controls[control_index + 1].steering);
      control_index++;
    }
  }

  position.x += velocity * cos(angle) * dt;
  position.y += velocity * sin(angle) * dt;
  angle += velocity * steering * dt / frontCenterDistance;
  orientation = Eigen::Quaternionf(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
  velocity += acceleration * dt;

  // Apply rolling instance if there is no acceleration input
  if (acceleration == 0) {
    if (velocity > 0) {
      velocity -= rollingInstance;
      if (velocity < 0) {
        velocity = 0;
      }
    } else if (velocity < 0) {
      velocity += rollingInstance;
      if (velocity > 0) {
        velocity = 0;
      }
    }
  }
}

[[nodiscard]] Vect3& Car::getPosition() {return position;}
[[nodiscard]] std::string& Car::getName() {return name;}
[[nodiscard]] float& Car::getVelocity() {return velocity;}
[[nodiscard]] float& Car::getAngle() {return angle;}
[[nodiscard]] Tracker& Car::getTracker() {return tracker;}
[[nodiscard]] Vect3 Car:: getDirection() {
  float angle = getAngle();
  return {cos(angle), sin(angle), 0.0f};
}

void Car::setTracker(const Tracker& t) {tracker = t;}
void Car::setPosition(const Vect3 &p) {position = p;}
void Car::setVelocity(float v) {velocity = v;}

static bool inbetween(double point, double center, double range) {
  return point >= center - range && point <= center + range;
}

[[nodiscard]] bool Car::checkCollision(Vect3 point) const {
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