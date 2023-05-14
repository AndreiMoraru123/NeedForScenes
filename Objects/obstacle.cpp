//
// Created by Andrei on 14-May-23.
//

#include "obstacle.hpp"

Obstacle::Obstacle(const Vect3& position, const Vect3& dimensions, int id)
    : position_(position), dimensions_(dimensions), id_(id) {
}
[[maybe_unused]] Vect3 Obstacle::getPosition() const { return position_; }
[[maybe_unused]] Vect3 Obstacle::getDimensions() const { return dimensions_; }
[[maybe_unused]] std::string Obstacle::getName() const { return name_; }
void Obstacle::render(pcl::visualization::PCLVisualizer::Ptr& viewer) const {
  pcl::ModelCoefficients coeffs;
  coeffs.values.resize(7); // We need 7 values for the cylinder model coefficients
  coeffs.values[0] = position_.x; // x position
  coeffs.values[1] = position_.y; // y position
  coeffs.values[2] = position_.z; // z position
  coeffs.values[3] = 0; // direction x
  coeffs.values[4] = 0; // direction y
  coeffs.values[5] = dimensions_.z; // direction z
  coeffs.values[6] = dimensions_.x; // radius
  viewer->addCylinder(coeffs, name_);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, name_);
}
bool Obstacle::checkCollision(Car& car) const {
  return car.checkCollision(position_);
}
void Obstacle::setName(const std::string& name) { name_ = name; }
