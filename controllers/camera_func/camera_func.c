#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>

#define TIME_STEP 64
#define MAX_SPEED 0
//6.28

int main(int argc, char **argv) {
 WbDeviceTag camera;
 
 wb_robot_init();
 
 camera = wb_robot_get_device("camera");
 
 wb_camera_enable(camera, TIME_STEP);
 


 wb_robot_cleanup();

 return 0;
}
