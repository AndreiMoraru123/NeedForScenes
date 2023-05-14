//
// Created by Andrei on 14-May-23.
//

#ifndef NFS_PARKINGSPOT_HPP
#define NFS_PARKINGSPOT_HPP

#include "../Objects/car.hpp"

class ParkingSpot {
public:
  ParkingSpot(const pcl::PointXYZ& position, double length, double width, double height, int id);
  [[maybe_unused]] [[nodiscard]] pcl::PointXYZ getPosition() const;
  [[maybe_unused]] [[nodiscard]] double getLength() const;
  [[maybe_unused]] [[nodiscard]] double getWidth() const;
  [[maybe_unused]] [[nodiscard]] double getHeight() const;
  [[maybe_unused]] [[nodiscard]] std::string getName() const;
  void render(pcl::visualization::PCLVisualizer::Ptr& viewer) const;
  void setName(const std::string& name);
private:
  pcl::PointXYZ position_;
  double length_;
  double width_;
  double height_;
  std::string name_;
  int id_;
};

#endif // NFS_PARKINGSPOT_HPP
