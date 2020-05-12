#include <stdio.h>
#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/lidar.h>
#include <webots/display.h>
#include <webots/camera_recognition_object.h>
#include <webots/distance_sensor.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 64

#define purple 0xC030C0
#define black 0x000000
#define gray 0xC0C0C0
#define white 0xFFFFFF
#define RED 0xBB2222
#define GREEN 0x22BB11
#define BLUE 0x2222BB

// the lidar values are clamped between 0 and CLAMP_MAX
#define CLAMP_MAX 1.0

// function retrieving the min value between x and y
#define MIN(x, y) ((x) > (y) ? (y) : (x))

// function displaying lidar values on a display
// display, width, height, lidar vals, num of samples, fov
static void Display(WbDeviceTag d, int dw, int dh, const float *v, int ns, float fov){
  int dw2 = dw / 2;
  int dh2 = dh / 2;
  float fov2 = fov / 2;
  const int px[] = {dw2, dw2 + dw * cos(-fov2 - M_PI_2), dw2 + dw * cos(fov2 - M_PI_2)};
  const int py[] = {dh2, dh2 + dh * sin(-fov2 - M_PI_2), dh2 + dh * sin(fov2 - M_PI_2)};

  wb_display_set_color(d, white);
  wb_display_fill_rectangle(d, 0, 0, dw, dh);
  //wb_display_set_color(d, gray);
  //wb_display_fill_polygon(d, px, py, 3);
  wb_display_set_color(d, black);
  wb_display_draw_polygon(d, px, py, 3);
  wb_display_draw_line(d, dw2, 0, dw2, dh);
  wb_display_draw_line(d, 0, dh2, dw, dh2);
  wb_display_draw_oval(d, dw2, dh2, dw2, dh2);
  wb_display_set_color(d, purple);

  int i;
  for (i = 0; i < ns; i++) {
    float f = MIN(CLAMP_MAX, v[i]);
    float alpha = -fov2 + fov * i / ns - M_PI_2;
    wb_display_draw_line(d, dw2, dh2, dw2 + f * cos(alpha) * dw2 / CLAMP_MAX, dh2 + f * sin(alpha) * dh2 / CLAMP_MAX);
  }
}



int main() { 
  wb_robot_init();
  
  WbDeviceTag camera, gps, utm30lx, display;
  
  // enable camera
  camera = wb_robot_get_device("Hexabot_Camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  
  // enable gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP);
  
  // enable hokuyo lidar
  utm30lx = wb_robot_get_device("Hokuyo UTM-30LX");
  wb_lidar_enable(utm30lx, TIME_STEP);
  
  // enable display for lidar
  display = wb_robot_get_device("display");
  //wb_lidar_enable(display, TIME_STEP); 
  
  // print gps coord
  const double *gps_values = wb_gps_get_values(gps);
  printf("Coord of robot: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);

  // lidar
  int utm30lx_samples = wb_lidar_get_horizontal_resolution(utm30lx);
  double utm30lx_field_of_view = wb_lidar_get_fov(utm30lx);

  // for hokuyo
  int display_width = wb_display_get_width(display);
  int display_height = wb_display_get_height(display); 

  printf("tezt\n");
  
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
   // print gps coord
    //const double *gps_values = wb_gps_get_values(gps);
    //printf("Using the GPS device: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);
  
    // lidar display  
    const float *utm30lx_values = wb_lidar_get_range_image(utm30lx);
    Display(display, display_width, display_height, utm30lx_values, utm30lx_samples, utm30lx_field_of_view);

    
    printf("Horizontal Resolution : %d \n", utm30lx_samples);
    printf("FOV : %f \n", utm30lx_field_of_view);
    
    
    // try to get lidar values
    const float *lidarValues = wb_lidar_get_range_image(utm30lx);
    const float *lidarValueRange = wb_lidar_get_layer_range_image(utm30lx,0);
    
    printf("Range0 : %f \n", lidarValueRange[0]);
  
    // lidarValues range is resolution - 1080
    printf("0 : %f \n", lidarValues[0]);
    printf("360 : %f \n", lidarValues[360]);
    printf("720 : %f \n", lidarValues[720]);
    printf("1080 : %f \n", lidarValues[1079]);
    
    
    
    
 /*   
    // get current number of object recognized
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    printf("\nRecognized %d objects.\n", number_of_objects);

    // get and display all the objects information
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    for (int i = 0; i < number_of_objects; ++i) {
      printf("Model of object %d: %s\n", i, objects[i].model);
      printf("Id of object %d: %d\n", i, objects[i].id);
      printf("Relative position of object %d: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
             objects[i].position[2]);
      printf("Relative orientation of object %d: %lf %lf %lf %lf\n", i, objects[i].orientation[0], objects[i].orientation[1],
             objects[i].orientation[2], objects[i].orientation[3]);
      printf("Position of the object %d on the camera image: %d %d\n", i, objects[i].position_on_image[0],
             objects[i].position_on_image[1]);
      for (int j = 0; j < objects[i].number_of_colors; ++j)
        printf("- Color %d/%d: %lf %lf %lf\n", j + 1, objects[i].number_of_colors, objects[i].colors[3 * j],
               objects[i].colors[3 * j + 1], objects[i].colors[3 * j + 2]);
    }
*/


  }
  
  wb_robot_cleanup();
  
  return 0;
}
