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
#include "Auxillaries/Camera.c"

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

  WbDeviceTag utm30lx, Lidar_Display;
  // enable hokuyo lidar
  utm30lx = wb_robot_get_device("Hokuyo UTM-30LX");
  wb_lidar_enable(utm30lx, TIME_STEP);
  
  // enable display for lidar
  Lidar_Display = wb_robot_get_device("Lidar_Display");
  //wb_lidar_enable(Lidar_Display, TIME_STEP); 
  
  // lidar
  int utm30lx_samples = wb_lidar_get_horizontal_resolution(utm30lx);
  double utm30lx_field_of_view = wb_lidar_get_fov(utm30lx);

  // for hokuyo
  int display_width = wb_display_get_width(Lidar_Display);
  int display_height = wb_display_get_height(Lidar_Display); 
  
   //Camera Enable
   struct Camera Cam = Camera_Init();

   //GPS Enable
   struct GPS Gps = GPS_Init();
   
    //Gyro enable
   WbDeviceTag Hexabot_Gyro = wb_robot_get_device("Hexabot_Gyro");
   wb_gyro_enable(Hexabot_Gyro,10);

   //Compass Engable
   struct Compass COMP = Compass_Init();
   
   //Robot Walking Motors
   WbDeviceTag Hexabot_Motors[18] = {wb_robot_get_device("Hexabot_Leg0_Motor1"), wb_robot_get_device("Hexabot_Leg0_Motor2"), wb_robot_get_device("Hexabot_Leg0_Motor3"),
                                    wb_robot_get_device("Hexabot_Leg1_Motor1"), wb_robot_get_device("Hexabot_Leg1_Motor2"), wb_robot_get_device("Hexabot_Leg1_Motor3"),
                                    wb_robot_get_device("Hexabot_Leg2_Motor1"), wb_robot_get_device("Hexabot_Leg2_Motor2"), wb_robot_get_device("Hexabot_Leg2_Motor3"), 
                                    wb_robot_get_device("Hexabot_Leg3_Motor1"), wb_robot_get_device("Hexabot_Leg3_Motor2"), wb_robot_get_device("Hexabot_Leg3_Motor3"),
                                    wb_robot_get_device("Hexabot_Leg4_Motor1"), wb_robot_get_device("Hexabot_Leg4_Motor2"), wb_robot_get_device("Hexabot_Leg4_Motor3"),
                                    wb_robot_get_device("Hexabot_Leg5_Motor1"), wb_robot_get_device("Hexabot_Leg5_Motor2"), wb_robot_get_device("Hexabot_Leg5_Motor3")
                                     };
   
   //Robot Walking Variables
   const double f = .5;
   const double a[18] = {.1, .1, -.1, -.1, -.1, .1, .1, .1, -.1, .1, .1, -.1, -.1, -.1, .1, .1, .1, -.1};
   const double p[18] = {0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5};
   const double d[18] = {-.1, -.2, -.3, .5, -.2, -.3, 1.1, -.2, -.3, -.1, -.2, -.3, 0, -.2, .1, 0, -.2, .1};
   //Enable Walking
   wb_keyboard_enable(TIME_STEP);
   int key;
   
   int i;//for loop
   for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * 1 + p[i]) + d[i]);
   
   while (wb_robot_step(TIME_STEP) != -1) 
   {
      // lidar display  
      const float *utm30lx_values = wb_lidar_get_range_image(utm30lx);
      Display(Lidar_Display, display_width, display_height, utm30lx_values, utm30lx_samples, utm30lx_field_of_view);    
   
      double time = wb_robot_get_time();
      key = wb_keyboard_get_key();
      if(key == WB_KEYBOARD_UP)
        for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
      else if(key == WB_KEYBOARD_DOWN)
        for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
      else if(key == WB_KEYBOARD_LEFT)
      {
        for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
        for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
  
      }
      else if(key == WB_KEYBOARD_RIGHT)
      {
        for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
        for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
        
      }
      
      Cam = Camera_Loop(Cam);
      Gps = GPS_Loop(Gps);
      COMP = Compass_Loop(COMP);
      if(Gps.quadrant == 1)
      {
         Gps.angle = 90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(Gps.distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         
       }
       else if(Gps.quadrant == 2)
       {
         Gps.angle = -90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(Gps.distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
       }
       else if(Gps.quadrant == 3)
       {
         Gps.angle = -90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(Gps.distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
       }
       else if(Gps.quadrant == 4)
       {
         Gps.angle = 90 - Gps.angle;
         if(COMP.degree > Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(COMP.degree < Gps.angle && abs(COMP.degree - Gps.angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(Gps.distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
       }
   };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  COMP = Compass_Disable(COMP);
  wb_gyro_disable(Hexabot_Gyro);
  Gps = GPS_Disable(Gps);
  
  wb_keyboard_disable();
  wb_robot_cleanup();//

  return 0;
}
