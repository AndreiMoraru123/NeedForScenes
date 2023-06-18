//
// Created by Andrei on 14-May-23.
//

#ifndef NFS_OBSTACLE_HPP
#define NFS_OBSTACLE_HPP

#include "../Objects/car.hpp"

class Obstacle {
public:
  Obstacle(const Vect3& position, const Vect3& dimensions);
  [[maybe_unused]] [[nodiscard]] Vect3 getPosition() const;
  [[maybe_unused]] [[nodiscard]] Vect3 getDimensions() const;
  [[maybe_unused]] [[nodiscard]] std::string getName() const;
  [[maybe_unused]] void render(pcl::visualization::PCLVisualizer::Ptr& viewer) const;
  bool checkCollision(Car& car) const;
  void setName(const std::string& name);
private:
  Vect3 position_;
  Vect3 dimensions_;
  std::string name_;
};

#endif // NFS_OBSTACLE_HPP
