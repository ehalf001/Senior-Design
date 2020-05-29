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
#include <webots/Keyboard.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "../Auxillaries/GPS.c"
#include "../Auxillaries/Compass.c"
#include "../Auxillaries/Camera.c"
#include "Walking.c"




struct Walking DecisionTree_Pattern(struct Walking Walkstate, struct GPS *GpsState, struct Compass CompState, struct Camera Cam)
{
   double time = wb_robot_get_time();
   struct Walking newState = Walkstate;
   int key;
   bool GoalFound = Cam.goal;
   bool NoObst = true;
   if(GoalFound)
   {
     if(GpsState->x_destination == 100 || GpsState->z_destination == 100)
     {
       GpsState->x_destination = GpsState->pos_x +.1 - Cam.x_relative*cos(CompState.degree*M_PI/180.0) + Cam.z_relative*sin(CompState.degree*M_PI/180.0);
       GpsState->z_destination = GpsState->pos_z - Cam.x_relative*sin(CompState.degree*M_PI/180.0) + Cam.z_relative*cos(CompState.degree*M_PI/180.0);// + .305*cos(M_PI - CompState.degree); 
     }
     if(NoObst)
     {
         if(GpsState->quadrant == 1)
         {
            GpsState->angle = 90 - GpsState->angle;
            if((CompState.degree < GpsState->angle && CompState.degree > (GpsState->angle - 180)) && abs(CompState.degree - GpsState->angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState->angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState->distance > .5)
               newState = Foward(newState, time);
           
         }
         else if(GpsState->quadrant == 2)
         {
            GpsState->angle = -90 - GpsState->angle;
            if((CompState.degree < GpsState->angle && CompState.degree > (180 - GpsState->angle)) && abs(CompState.degree - GpsState->angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState->angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState->distance > .5)
               newState = Foward(newState, time);
         }
         else if(GpsState->quadrant == 3)
         {
            GpsState->angle = -90 - GpsState->angle;
            if((CompState.degree < GpsState->angle && CompState.degree > (180 - GpsState->angle)) && abs(CompState.degree - GpsState->angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState->angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState->distance > .5)
               newState = Foward(newState, time);
         }
         else if(GpsState->quadrant == 4)
         {
            GpsState->angle = 90 - GpsState->angle;
            if((CompState.degree < GpsState->angle && CompState.degree > (180 - GpsState->angle)) && abs(CompState.degree - GpsState->angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState->angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState->distance > .5)
               newState = Foward(newState, time);
         }
      }
  }
  else
  {
     if(Cam.centerPixel < 31)
     {
       newState = TurnLEFT(newState, time);
     }
     else
     {
       newState = TurnRIGHT(newState, time);
     }
  }
  key = wb_keyboard_get_key();
  if(key == WB_KEYBOARD_UP)
  {
    newState = Foward(newState, time);
  }
  else if(key == WB_KEYBOARD_DOWN)
  {
    newState = Backward(newState, time);
  }
  else if(key == WB_KEYBOARD_LEFT)
  {
    newState = TurnLEFT(newState, time);
  }
  else if(key == WB_KEYBOARD_RIGHT)
  {
    newState = TurnRIGHT(newState, time);
  }
      
  return newState;
}

