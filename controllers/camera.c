#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 64
#define MAX_SPEED 0
//6.28

using namespace webots;

int main(int argc, char **argv) {
 WbDeviceTag camera;
 
 wb_robot_init();
 
 camera = wb_get_device("camera");
 
 wb_camera_enable(camera, TIME_STEP);

 wb_robot_cleanup();

 return 0;
}