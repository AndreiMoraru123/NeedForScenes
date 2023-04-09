#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_map>

int main() {
  // Create a visualizer (an object that will display the point cloud)
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  // Define cube parameters
  double cube_x = 100.0;
  double cube_y = 100.0;
  double cube_z = 100.0;
  double cube_size = 100.0;
  double cube_rot_x = 100.0;
  double cube_rot_y = 100.0;
  double cube_rot_z = 100.0;

  // Define movement parameters for the cube (velocity and acceleration)
  double cube_vel_x = 0.0;
  double cube_vel_y = 0.0;
  double cube_vel_z = 0.0;
  double cube_accel_x = 0.0;
  double cube_accel_y = 0.0;
  double cube_accel_z = 0.0;

  // Map to store the key states
  std::unordered_map<std::string, bool> key_states = {
      {"Left", false},
      {"Right", false},
      {"Up", false},
      {"Down", false}
  };

  // Add the cube to the viewer (the viewer will display the cube)
  viewer.addCube(cube_x, cube_x + cube_size,
                 cube_y, cube_y + cube_size,
                 cube_z, cube_z + cube_size,
                 cube_rot_x, cube_rot_y, cube_rot_z, "Cube");

  // Register keyboard callback function
  viewer.registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent& event) {
    if (key_states.find(event.getKeySym()) != key_states.end()) {
      key_states[event.getKeySym()] = event.keyDown();
    }
  });

  // Start the visualization loop
  while (!viewer.wasStopped()) {
    // Update the cube acceleration based on key states
    if (key_states["Left"]) {
      cube_accel_x = -10.0;
    } else if (key_states["Right"]) {
      cube_accel_x = 10.0;
    } else {
      cube_accel_x = 0.0;
    }

    if (key_states["Up"]) {
      cube_accel_y = 10.0;
    } else if (key_states["Down"]) {
      cube_accel_y = -10.0;
    } else {
      cube_accel_y = 0.0;
    }

    // Update the cube position and velocity
    cube_x += cube_vel_x;
    cube_y += cube_vel_y;
    cube_z += cube_vel_z;
    cube_vel_x += cube_accel_x;
    cube_vel_y += cube_accel_y;
    cube_vel_z += cube_accel_z;

    // Bounce the cube off the boundaries
    if (cube_x < -500.0 || cube_x > 500.0) {
      cube_vel_x *= -1.0;
    }
    if (cube_y < -500.0 || cube_y > 500.0) {
      cube_vel_y *= -1.0;
    }
    if (cube_z < -500.0 || cube_z > 500.0) {
      cube_vel_z *= 1.0;
    }

    // Update the cube in the viewer
    viewer.updateShapePose("Cube", Eigen::Affine3f(
                                       Eigen::Translation3f(
                                           cube_x,
                                           cube_y,
                                           cube_z)));

    // Update the viewer will be updated once
    viewer.spinOnce(100);
  }

  return 0;
}
