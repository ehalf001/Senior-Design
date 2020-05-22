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

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "Auxillaries/GPS.c"
#include "Auxillaries/Compass.c"
#include "Auxillaries/Camera.c"
#include "Auxillaries/Lidar.c"

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  //Lidar Enable
  struct Lidar lidar = Lidar_Init();
  
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
      lidar = Lidar_Loop(lidar);

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

  /* Cleanup webots resources */
  COMP = Compass_Disable(COMP);
  wb_gyro_disable(Hexabot_Gyro);
  Gps = GPS_Disable(Gps);
  Cam = Camera_Disable(Cam);
  lidar = Lidar_Disable(lidar);
  wb_keyboard_disable();
  wb_robot_cleanup();

  return 0;
}
