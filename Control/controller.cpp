//
// Created by Andrei on 01-May-23.
//

#include "controller.hpp"


EgoCarController::EgoCarController(pcl::visualization::PCLVisualizer::Ptr& viewer, Car& car)
    : viewer_(viewer), car_(car), currentAngle_(FPS), timeUs_(0) {
  angles_ = {XY, TopDown, Side, FPS};
  keyStates_ = {
      {"Left", false},
      {"Right", false},
      {"Up", false},
      {"Down", false},
      {"x", false},
      {"space", false}
  };
}

void EgoCarController::registerKeyboardCallbacks() {
  viewer_->registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent& event) {
    if (keyStates_.find(event.getKeySym()) != keyStates_.end()) {
      keyStates_[event.getKeySym()] = event.keyDown();
    }
    if (event.isShiftPressed()) {car_.accelerate(10.0, 1);}
  });
}

void EgoCarController::handleKeyboardInput() {
  if (keyStates_["Up"]) {
    car_.accelerate(5.0, 1);
  } else if (keyStates_["Down"]) {
    car_.accelerate(3.0, -1);
  } else if (keyStates_["space"]) {
    car_.accelerate(-15.0, 1);
  } else {
    car_.accelerate(0.0, 1);
  }

  if (keyStates_["Left"]) {
    car_.steer(0.5);
  } else if (keyStates_["Right"]) {
    car_.steer(-0.5);
  } else {
    car_.steer(0.0);
  }

  if (keyStates_["x"]) {
    currentAngle_ = angles_[(currentAngle_ + 1) % angles_.size()];
    changeView(currentAngle_);
    keyStates_["x"] = false;
  }
}

void EgoCarController::update(float dt, Scene& scene) {
  scene.stepScene(car_, dt, 10, viewer_);
  timeUs_ += intervalUs_;
}

void EgoCarController::changeView(CameraAngle angle) {
  changeCameraView(angle, viewer_);
}

void EgoCarController::stop() {
  car_.accelerate(0.0, 1);
  car_.steer(0.0);
}