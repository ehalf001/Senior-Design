#include <stdio.h>
#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64

int main() {
  WbDeviceTag camera;
  //WbDeviceTag dSensor, gps;
  
  wb_robot_init();
  
  // enable camera
  camera = wb_robot_get_device("Hexabot_Camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
/*  
  // enable distance sensor
  dSensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(dSensor, TIME_STEP);
  
  // enable gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP);
  
  // print gps coord
  const double *gps_values = wb_gps_get_values(gps);
  printf("Coord of robot: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);
*/  

  printf("tezt\n");
  while (wb_robot_step(TIME_STEP) != -1) {
    //const unsigned char *image = wb_camera_get_image(camera);
    //const double object_distance = wb_distance_sensor_get_value(dSensor) / 1000;
    //wb_camera_set_focal_distance(camera, object_distance);
    
    //num = wb_camera_recognition_get_number_of_objects(camera);
    
/* Get current number of object recognized */
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    printf("\nRecognized %d objects.\n", number_of_objects);

    /* Get and display all the objects information */
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    for (int i = 0; i < number_of_objects; ++i) {
      printf("Model of object %d: %s\n", i, objects[i].model);
      printf("Id of object %d: %d\n", i, objects[i].id);
      printf("Relative position of object %d: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
             objects[i].position[2]);
      printf("Relative orientation of object %d: %lf %lf %lf %lf\n", i, objects[i].orientation[0], objects[i].orientation[1],
             objects[i].orientation[2], objects[i].orientation[3]);
      printf("Size of object %d: %lf %lf\n", i, objects[i].size[0], objects[i].size[1]);
      printf("Position of the object %d on the camera image: %d %d\n", i, objects[i].position_on_image[0],
             objects[i].position_on_image[1]);
      printf("Size of the object %d on the camera image: %d %d\n", i, objects[i].size_on_image[0], objects[i].size_on_image[1]);
      for (int j = 0; j < objects[i].number_of_colors; ++j)
        printf("- Color %d/%d: %lf %lf %lf\n", j + 1, objects[i].number_of_colors, objects[i].colors[3 * j],
               objects[i].colors[3 * j + 1], objects[i].colors[3 * j + 2]);
    }

  }
  
  wb_robot_cleanup();
  
  return 0;
}
