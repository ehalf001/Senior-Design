
/*
 * File:          my_controller_c.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/Keyboard.h>

#include <webots/gyro.h>
#include <webots/display.h>
#include <webots/lidar.h>
#include <webots/camera_recognition_object.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "Auxillaries/GPS.c"
#include "Auxillaries/Compass.c"
#include "System/Walking.c"

#define purple 0xC030C0
#define black 0x000000
#define gray 0xC0C0C0
#define white 0xFFFFFF

#define TIME_STEP 32

// the lidar values are clamped between 0 and CLAMP_MAX
#define CLAMP_MAX 1.0

// function retrieving the min value between x and y
#define MIN(x, y) ((x) > (y) ? (y) : (x))

// function displaying lidar values on a display
// display, width, height, lidar vals, num of samples, fov
void Display(WbDeviceTag d, int dw, int dh, const float *v, int ns, float fov){
  int dw2 = dw / 2;
  int dh2 = dh / 2;
  float fov2 = fov / 2;
  const int px[] = {dw2, dw2 + dw * cos(-fov2 - M_PI_2), dw2 + dw * cos(fov2 - M_PI_2)};
  const int py[] = {dh2, dh2 + dh * sin(-fov2 - M_PI_2), dh2 + dh * sin(fov2 - M_PI_2)};

  wb_display_set_color(d, white);
  wb_display_fill_rectangle(d, 0, 0, dw, dh);
  wb_display_set_color(d, gray);
  wb_display_fill_polygon(d, px, py, 3);
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




int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

   WbDeviceTag Hexabot_Camera = wb_robot_get_device("Hexabot_Camera");
   wb_camera_enable(Hexabot_Camera, 10);
   wb_camera_recognition_enable(Hexabot_Camera, TIME_STEP);
   
    // get current number of object recognized by the camera
    int numObjects = wb_camera_recognition_get_number_of_objects(Hexabot_Camera);
    printf("\nRecognized %d objects.\n", numObjects);

    // get and display all the objects information
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(Hexabot_Camera);
    for (int i = 0; i < numObjects; ++i) {
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


  WbDeviceTag utm30lx, Lidar_Display;
  // enable hokuyo lidar
  utm30lx = wb_robot_get_device("Hokuyo UTM-30LX");
  wb_lidar_enable(utm30lx, TIME_STEP);
  
  // enable display for lidar
  Lidar_Display = wb_robot_get_device("Lidar_Display");
  wb_lidar_enable(Lidar_Display, TIME_STEP); 
  
  // lidar
  int utm30lx_samples = wb_lidar_get_horizontal_resolution(utm30lx);
  double utm30lx_field_of_view = wb_lidar_get_fov(utm30lx);

  // for hokuyo
  int display_width = wb_display_get_width(Lidar_Display);
  int display_height = wb_display_get_height(Lidar_Display); 

   //GPS Enable
   struct GPS Gps = GPS_Init();

   //Compass Enable
   struct Compass COMP = Compass_Init();
   
   //Walking Enable
   struct Walking WalkPatt = Walking_Init();

   while (wb_robot_step(TIME_STEP) != -1) 
   {
      // lidar display  
      const float *utm30lx_values = wb_lidar_get_range_image(utm30lx);
      Display(Lidar_Display, display_width, display_height, utm30lx_values, utm30lx_samples, utm30lx_field_of_view);    
   
      double time = wb_robot_get_time();


      Gps = GPS_Loop(Gps);
      COMP = Compass_Loop(COMP);
      if(Gps.quadrant == 1)
      {
         Gps.angle = 90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
           WalkPatt = TurnLEFT(WalkPatt, time);
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
            WalkPatt = TurnRIGHT(WalkPatt, time);
         else if(Gps.distance > .5)
            WalkPatt = Foward(WalkPatt, time);
         
       }
       else if(Gps.quadrant == 2)
       {
         Gps.angle = -90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
            WalkPatt = TurnLEFT(WalkPatt, time);
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
            WalkPatt = TurnRIGHT(WalkPatt, time);
         else if(Gps.distance > .5)
            WalkPatt = Foward(WalkPatt, time);
       }
       else if(Gps.quadrant == 3)
       {
         Gps.angle = -90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
            WalkPatt = TurnLEFT(WalkPatt, time);
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
            WalkPatt = TurnRIGHT(WalkPatt, time);
         else if(Gps.distance > .5)
            WalkPatt = Foward(WalkPatt, time);
       }
       else if(Gps.quadrant == 4)
       {
         Gps.angle = 90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
            WalkPatt = TurnLEFT(WalkPatt, time);
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
            WalkPatt = TurnRIGHT(WalkPatt, time);
         else if(Gps.distance > .5)
            WalkPatt = Foward(WalkPatt, time);
       }
   };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  COMP = Compass_Disable(COMP);
  Gps = GPS_Disable(Gps);
  wb_robot_cleanup();//

  return 0;
}

