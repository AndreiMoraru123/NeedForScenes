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

  viewer->setCameraPosition(50, 50, 50, // Camera position
                            0, 0, 0, // Focal point (origin)
                            0, 0, 1, // View up direction
                            0); // Set camera viewpoint to (0, 0, 0)
  viewer->setCameraFieldOfView(0.8);
  viewer->setCameraClipDistances(0.01, 500);

  Vect3 position(-2, -3, 0);
  Vect3 dimensions(4.0, 2.0, 2.0);
  Color color(0.0, 1.0, 0.0);
  float angle = -180.0;
  float front_center_distance = 1.0;
  std::string car_name = "egoCar";
  Car car = Car(
      position, dimensions, color,
      angle, front_center_distance,
      car_name
  );
  EgoCarController carController(viewer, car);
  carController.registerKeyboardCallbacks();

  float boundary_margin = 100.0;
  float x_min = -boundary_margin, x_max = boundary_margin;
  float y_min = -boundary_margin, y_max = boundary_margin;
  const int interval_us = 100000;
  float dt = interval_us * 1e-6;
  Scene scene(viewer);
  
  while (!viewer->wasStopped()) {
    
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    carController.handleKeyboardInput();
    carController.update(dt, scene);
    viewer->spinOnce(1);
    Vect3 car_position = car.getPosition();
    
    if (car_position.x < x_min || car_position.x > x_max ||
        car_position.y < y_min || car_position.y > y_max) {
      viewer->setCameraPosition(car_position.x - 10, car_position.y - 10,
                                car_position.z + 10, car_position.x,
                                car_position.y, car_position.z,
                                0, 0, 1);
      viewer->setCameraClipDistances(0.01, 500);
      x_min = car_position.x - boundary_margin;
      x_max = car_position.x + boundary_margin;
      y_min = car_position.y - boundary_margin;
      y_max = car_position.y + boundary_margin;
    }

    if (scene.checkTrafficCollision(car)) {
      carController.stop();
    }

    if (scene.win) {
      int textWidth = 50;
      int textHeight = 50;
      int xpos = (screenSize[0] - textWidth) / 2;
      int ypos = (screenSize[1] - textHeight);
      int fontSize = 40;
      double r = 1.0, g = 1.0, b = 1.0;
      viewer->addText("You've won the game!", xpos, ypos, fontSize, r, g, b, "winning text");
      viewer->spinOnce(3000);
      break;
    }
  }
}
