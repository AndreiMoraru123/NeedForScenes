//
// Created by Andrei on 14-May-23.
//

#include "obstacle.hpp"

Obstacle::Obstacle(const Vect3 &position, const Vect3 &dimensions)
    : position_(position), dimensions_(dimensions) {}
[[maybe_unused]] Vect3 Obstacle::getPosition() const { return position_; }
[[maybe_unused]] Vect3 Obstacle::getDimensions() const { return dimensions_; }
[[maybe_unused]] std::string Obstacle::getName() const { return name_; }
void Obstacle::render(pcl::visualization::PCLVisualizer::Ptr &viewer) const {
  viewer->addCube(position_.x, position_.x + dimensions_.x, position_.y,
                  position_.y + dimensions_.y, position_.z,
                  position_.z + dimensions_.z, 1, 1, 0, name_);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      1, 1, 0, name_);
}
bool Obstacle::checkCollision(Car &car) const {
  return car.checkCollision(position_);
}
void Obstacle::setName(const std::string &name) { name_ = name; }
