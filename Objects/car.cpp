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
  // Render bottom of the car
  viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z * 1 / 3),
                  orientation,
                  dimensions.x, dimensions.y, dimensions.z * 2 / 3,
                  name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b,
                                      name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                      name);
  // Render top of the car
  viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z * 5 / 6),
                  orientation,
                  dimensions.x / 2, dimensions.y, dimensions.z * 1 / 3,
                  name + "Top");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b,
                                      name + "Top");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                      name + "Top");
  // Render wheels
  for(int i=-1; i<=1; i+=2) {
    for(int j=-1; j<=1; j+=2) {
      pcl::ModelCoefficients coeffs;
      coeffs.values.push_back(position.x + i * dimensions.x / 5); // x position
      coeffs.values.push_back(position.y + j * dimensions.y / 2); // y position
      coeffs.values.push_back(position.z + dimensions.z / 7); // z position
      coeffs.values.push_back(1); // Direction x
      coeffs.values.push_back(0); // Direction y
      coeffs.values.push_back(0); // Direction z
      coeffs.values.push_back(dimensions.z * 1 / 5); // Radius
      viewer->addCylinder(coeffs, name + "Wheel" + std::to_string(i) + std::to_string(j));
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                          0, 0, 0,
                                          name + "Wheel" + std::to_string(i) + std::to_string(j));
    }
  }
  // Render windshields
  for(int i=-1; i<=1; i+=2) {
    viewer->addCube(Eigen::Vector3f(position.x + i * dimensions.x / 4, position.y, dimensions.z * 2 / 3),
                    orientation,
                    dimensions.x / 4, dimensions.y, dimensions.z / 6,
                    name + "Windshield" + std::to_string(i));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        0.3, 0.7, 1.0,
                                        name + "Windshield" + std::to_string(i));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        name + "Windshield" + std::to_string(i));
  }
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

[[nodiscard]] Vect3& Car::getPosition() {return position;}
[[nodiscard]] std::string& Car::getName()  {return name;}
[[nodiscard]] float& Car::getVelocity()  {return velocity;}
[[nodiscard]] float& Car::getAngle()  {return angle;}
[[nodiscard]] Tracker& Car::getTracker() {return tracker;}

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