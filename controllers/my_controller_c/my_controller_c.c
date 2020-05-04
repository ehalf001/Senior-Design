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

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

void wb_camera_enable(WbDeviceTag tag, int sampling_period);
void wb_camera_disable(WbDeviceTag tag);
int wb_camera_get_sampling_period(WbDeviceTag tag);

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();


   WbDeviceTag Hexabot_Camera = wb_robot_get_device("Hexabot_Camera");
   wb_camera_enable(Hexabot_Camera, 10);
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
   };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_keyboard_disable();
  wb_robot_cleanup();

  return 0;
}
