//
// Created by Andrei on 14-May-23.
//

#include "parkingspot.hpp"

ParkingSpot::ParkingSpot(const pcl::PointXYZ& position, double length, double width, double height, int id)
    : position_(position), length_(length), width_(width), height_(height), id_(id) {}

[[maybe_unused]] pcl::PointXYZ ParkingSpot::getPosition() const { return position_; }
[[maybe_unused]] double ParkingSpot::getLength() const { return length_; }
[[maybe_unused]] double ParkingSpot::getWidth() const { return width_; }
[[maybe_unused]] double ParkingSpot::getHeight() const { return height_; }
[[maybe_unused]] std::string ParkingSpot::getName() const { return name_; }
void ParkingSpot::render(pcl::visualization::PCLVisualizer::Ptr& viewer) const {
  viewer->addCube(position_.x - length_ / 2, position_.x + length_ / 2,
                  position_.y - width_ / 2, position_.y + width_ / 2,
                  position_.z, position_.z + height_,
                  1.0, 1.0, 0.0, name_);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, name_);
}
void ParkingSpot::setName(const std::string& name) { name_ = name; }

