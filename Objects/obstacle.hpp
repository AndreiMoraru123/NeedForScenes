//
// Created by Andrei on 14-May-23.
//

#ifndef NFS_OBSTACLE_HPP
#define NFS_OBSTACLE_HPP

#include "../Objects/car.hpp"

class Obstacle {
public:
  Obstacle::Obstacle(
      const Vect3& position, const Vect3& dimensions, int id);
  [[maybe_unused]] Vect3 getPosition() const;
  [[maybe_unused]] Vect3 getDimensions() const;
  [[maybe_unused]] std::string getName() const;
  [[maybe_unused]] void render(pcl::visualization::PCLVisualizer::Ptr& viewer) const;
  bool checkCollision(Car& car) const;
  void setName(const std::string& name);
private:
  Vect3 position_;
  Vect3 dimensions_;
  std::string name_;
  int id_;
};

#endif // NFS_OBSTACLE_HPP