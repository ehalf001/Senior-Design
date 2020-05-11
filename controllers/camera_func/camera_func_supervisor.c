#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  WbDeviceTag pathDisplay;
  pathDisplay = wb_robot_get_device("path display");
  
  int width = wb_display_get_width(pathDisplay);
  int height = wb_display_get_height(pathDisplay);

  // do this once only
  WbNodeRef getNode = wb_supervisor_node_get_from_def("Hexabot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(getNode, "translation");

  while (wb_robot_step(TIME_STEP) != -1) {
    // this is done repeatedly
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    printf("MY_ROBOT is at position: %g %g %g\n", values[0], values[1], values[2]);
  }

  wb_robot_cleanup();
  return 0;
}