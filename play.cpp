#include "Objects/car.hpp"
#include "View/camera.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_map>

int main() {
  // Set up a PCLVisualizer
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->getRenderWindow()->GlobalWarningDisplayOff();
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  int* screenSize = viewer->getRenderWindow()->GetScreenSize();
  viewer->setSize(screenSize[0], screenSize[1]);

  // Set up the camera
  // Set the viewer's camera position and adjust clipping planes
  viewer->setCameraPosition(0, 0, 50,   // Camera position
                            0, 0, 0,    // Focal point
                            0, 1, 0,    // View up direction
                            0);         // Set camera viewpoint to (0, 0, 0)
  viewer->setCameraFieldOfView(0.8);   // Set the camera's field of view (default is 0.8)
  viewer->setCameraClipDistances(0.01, 500); // Set near and far clipping planes

  // Initialize a car with position, dimensions, color, velocity, angle, acceleration, steering, and front_center_distance
  Vect3 position(0, 0, 0);
  Vect3 dimensions(2.0, 1.0, 0.5);
  Color color(1.0, 0.0, 0.0);
  float velocity = 0.0;
  float angle = 0.0;
  float acceleration = 0.0;
  float steering = 0.0;
  float front_center_distance = 1.0;
  std::string car_name = "car";
  Car car(position, dimensions, color, velocity, angle, acceleration, steering, front_center_distance, car_name);

  // Set up key states for keyboard controls
  std::unordered_map<std::string, bool> key_states = {
      {"Left", false},
      {"Right", false},
      {"Up", false},
      {"Down", false},
      {"x", false},
      {"spec", false}
  };

  // Register keyboard callback function
  viewer->registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent& event) {
    if (key_states.find(event.getKeySym()) != key_states.end()) {
      key_states[event.getKeySym()] = event.keyDown();
    }
  });

  CameraAngle currentAngle = FPS;
  std::vector<CameraAngle> angles = {XY, TopDown, Side, FPS};

  float boundary_margin = 10.0; // Margin around the car to set as viewer boundary
  float x_min = -boundary_margin, x_max = boundary_margin;
  float y_min = -boundary_margin, y_max = boundary_margin;
  int time_us = 0;
  const int interval_us = 100000;
  float dt = interval_us * 1e-6;


  car.render(viewer); // Render the car initially before entering the loop
  // Main loop
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);

    // Remove the car from the viewer
    viewer->removeShape(car_name);
    viewer->removeShape(car_name + "front");

    // Update the car's acceleration and steering based on keyboard input
    float reverse_multiplier = 1;
    if (key_states["Up"]) {
      car.accelerate(1.0);
    } else if (key_states["Down"]) {
      car.accelerate(1.0);
      reverse_multiplier = -1;
    } else if (key_states["space"]) {
      car.accelerate(-2.0);
    } else {
      car.accelerate(0.0);
    }

    if (key_states["Left"]) {
      car.steer(0.2);
    } else if (key_states["Right"]) {
      car.steer(-0.2);
    } else {
      car.steer(0.0);
    }

    if (key_states["x"]) {
      currentAngle = angles[(currentAngle + 1) % angles.size()];
      changeCameraView(currentAngle, viewer);
      key_states["x"] = false;
    }

    // Move the car based on controls
    car.move(dt * reverse_multiplier, time_us);
    time_us += interval_us;

    Vect3 car_position = car.getPosition();

    if (car_position.x < x_min || car_position.x > x_max ||
        car_position.y < y_min || car_position.y > y_max) {
      // Set the viewer's boundary based on the car's position
      viewer->setCameraPosition(car_position.x - 10, car_position.y - 10,
                                car_position.z + 10, car_position.x,
                                car_position.y, car_position.z, 0, 0, 1);
      viewer->setCameraClipDistances(0.01, 500);
      // Update the boundary limits
      x_min = car_position.x - boundary_margin;
      x_max = car_position.x + boundary_margin;
      y_min = car_position.y - boundary_margin;
      y_max = car_position.y + boundary_margin;

    }

    // Render the car
    car.render(viewer);
  }
}