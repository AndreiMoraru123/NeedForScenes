#include "Objects/car.hpp"
#include "View/camera.hpp"
#include "Scene/scene.hpp"
#include "Control/controller.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_map>

int main() {
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
  Vect3 position(-2, -3, 0);
  Vect3 dimensions(4.0, 2.0, 2.0);
  Color color(0.0, 1.0, 0.0);
  float velocity = 0.0;
  float angle = 0.0;
  float acceleration = 0.0;
  float steering = 0.0;
  float front_center_distance = 1.0;
  std::string car_name = "egoCar";
  Car car(position, dimensions, color, velocity, angle, acceleration, steering, front_center_distance, car_name);
  EgoCarController carController(viewer, car);
  carController.registerKeyboardCallbacks();

  float boundary_margin = 100.0; // Margin around the car to set as viewer boundary
  float x_min = -boundary_margin, x_max = boundary_margin;
  float y_min = -boundary_margin, y_max = boundary_margin;
  int time_us = 0;
  const int interval_us = 100000;
  float dt = interval_us * 1e-6;
  Scene scene(viewer);
  
  while (!viewer->wasStopped()) {
    
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    carController.handleKeyboardInput();
    carController.update(dt, time_us, scene);
    viewer->spinOnce(100);
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
  }
}