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
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/compass.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64


//Camera
void wb_camera_enable(WbDeviceTag tag, int sampling_period);
void wb_camera_disable(WbDeviceTag tag);
int wb_camera_get_sampling_period(WbDeviceTag tag);



double get_bearing_in_degrees(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < -180.0)
    bearing = bearing + 360.0;
   return bearing;
}


int Quadrant(double x1, double z1, double x2, double z2)
{
   if(x2 > x1 && z2 > z1)
       return 1;
   else if(x2 > x1 && z2 < z1)
       return 2;
   else if(x2 < x1 && z2 < z1)
       return 3;
   else if(x2 < x1 && z2 > z1)
       return 4;
   else return 0;
}



int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();


   WbDeviceTag Hexabot_Camera = wb_robot_get_device("Hexabot_Camera");
   wb_camera_enable(Hexabot_Camera, 10);
   
   //GPS Enable
   WbDeviceTag Hexabot_GPS = wb_robot_get_device("Hexabot_GPS");
   wb_gps_enable(Hexabot_GPS,10);
   const double *pos = wb_gps_get_values(Hexabot_GPS);
   double pos_x = pos[0];
   double pos_y = pos[1];
   double pos_z = pos[2];
   
   double vel = wb_gps_get_speed(Hexabot_GPS);
   
    //Gyro enable
   WbDeviceTag Hexabot_Gyro = wb_robot_get_device("Hexabot_Gyro");
   wb_gyro_enable(Hexabot_Gyro,10);
   
/*   const double *gyro = wb_gyro_get_values(Hexabot_Gyro);
   double gyro_x = gyro[0];
   double gyro_y = gyro[1];
   double gyro_z = gyro[2];
 */  
   //Compass Engable
   WbDeviceTag Hexabot_Compass = wb_robot_get_device("Hexabot_Compass");
   wb_compass_enable(Hexabot_Compass, 10);
   
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
   int quadrant;
   double distance;
   for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * 1 + p[i]) + d[i]);
   double degree = get_bearing_in_degrees(Hexabot_Compass);
   
   const double x_destination = 3.04;
   const double z_destination = 2.99;
   
   
   double angle;
   
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
      
      
      

      /*
      if(angle < k/f + c-5/f)
      {
        for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
        for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
        angle += 1;
      }*/
      angle = (atan((x_destination - pos_x)/(z_destination - pos_z))) / M_PI * 180.0;
      distance = sqrt(pow((x_destination - pos_x),2) + pow((z_destination - pos_z),2));
        
      pos_x = pos[0];
      pos_y = pos[1];
      pos_z = pos[2];
      
      vel = wb_gps_get_speed(Hexabot_GPS);
      
      degree = get_bearing_in_degrees(Hexabot_Compass);
      
      quadrant = Quadrant(pos_x, pos_z, x_destination, z_destination);
      
//      gyro_x = gyro[0];
//      gyro_y = gyro[1];
//      gyro_z = gyro[2];
      if (vel >= 0.01)
      {
        printf("Postion: x[%g] y[%g] z[%g]\n", pos_x,pos_y,pos_z);
        //printf("Velocity: %g\n", vel);
        //printf("Angular velocity: x[%g] y[%g] z[%g]\n", gyro_x,gyro_y,gyro_z);
        printf("Degrees: r[%g]\n", degree);
        printf("Angle: r[%g]\n", angle); 
        printf("Quadrant: %d\n", quadrant);
        printf("Distance: %g\n", distance);
      }
      
      if(quadrant == 1)
      {
         angle = 90 - angle;
         if( degree > angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if( degree < angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         
       }
       else if(quadrant == 2)
       {
         angle = -90 - angle;
         if( degree > angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if( degree < angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
       }
       else if(quadrant == 3)
       {
         angle = -90 - angle;
         if( degree > angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if( degree < angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
       }
       else if(quadrant == 4)
       {
         angle = 90 - angle;
         if( degree > angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if( degree < angle && abs(degree - angle) > 2)
         {
           for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(-2.0 * M_PI * f * time + p[i]) + d[i]);
           for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
         else if(distance > .5)
         {
           for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
             wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
         }
       }
   };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_compass_disable(Hexabot_Compass);
  wb_gyro_disable(Hexabot_Gyro);
  wb_gps_disable(Hexabot_GPS);
  wb_keyboard_disable();
  wb_robot_cleanup();//

  return 0;
}
