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
#include "MappingSystem.c"
#include "Walking.c"

// RETURN an ARRAY of 3 bools 1 int
//{LEFT,TOP,RIGHT,CENTER_OF_LEFT}
void Lidar_Wall_Detection(struct Lidar lidar, int *wall)
{
   
   
   int i, startPt, endPt, mid;
   startPt = -1;
   endPt = -1;
   for(i = 0; i < 1080; ++i){
     if(lidar.lidar_utm30lx_values[i] < .75 && startPt == -1){
       startPt = i;
     }
     if(lidar.lidar_utm30lx_values[i] > .75 && startPt != -1){
       endPt = i - 1;
       //mid = (endPt + startPt) / 2;
       //float ret = mid / 1080.0 * 270.0;
       
       float startDgre = startPt/1080.0 * 270;
       float endDgre = endPt/1080.0 * 270;
       
       if(startDgre <= 90){wall[0] = 1;}
       if(((180 < startDgre) && (startDgre <= 270)) || ((180 < endDgre) && (endDgre <= 270))){wall[2] = 1;}
       if( ((90 < startDgre) && (startDgre <= 180)) || ((90 < endDgre) && (endDgre <= 180)) || ((wall[0] == 1) && (wall[2]==1)) ){wall[1] = 1;}
       
       mid = (wall[1]) ? (endPt + 180) / 2 : (endPt + startPt)/2;
       float ret = mid / 1080.0 * 270.0;
       wall[3] = wall[2] ? ret : 0;
       startPt = -1;
       endPt = -1;       
     }
   }
   // //return wall;
} 


struct Walking Pledge_Algorithm(struct Lidar lidar, double time) 
{
  struct Walking Pledge_Walk = Walking_Init();
  
  int wall[4] = {0,0,0,0};
  Lidar_Wall_Detection(lidar, &wall);
  //int wall[5] = {0,0,0,0,0}; 
  
  if (wall[0]) { printf("LEFT\n");}
  if (wall[1]) { printf("WALL\n"); }
  if (wall[2]) { printf("RIGHT\n"); } 
  
  if ( !(wall[0] || wall[1] || wall[2]) || (wall[2] & !wall[1]))
  {
    //walk foreward
    Pledge_Walk = Forward(Pledge_Walk, time);
  } 
  else if (wall[1])
  {
    //turn left
    Pledge_Walk = TurnLEFT(Pledge_Walk, time);
  }
  else if (wall[3] >= 250)
  {
    //turn right
    Pledge_Walk = TurnRIGHT(Pledge_Walk, time);
  }
  else
  {
    //walk foreward
    Pledge_Walk = Forward(Pledge_Walk, time);
  }
  
  return Pledge_Walk;
}


struct Walking DecisionTree_Pattern(struct Walking Walkstate, struct GPS *GpsState, struct Compass CompState, struct Camera Cam, struct Lidar LidarState)
{
   double time = wb_robot_get_time();
   struct Walking newState = Walkstate;
   int key;
   bool GoalFound = Cam.goal;
   bool NoObst = true;
   


   
   if(GoalFound)
   {
//     if(GpsState->x_destination == 100 || GpsState->z_destination == 100)
//     {
       
       GpsState->x_destination = GpsState->pos_x +.1 - Cam.x_relative*cos(CompState.degree*M_PI/180.0) + Cam.z_relative*sin(CompState.degree*M_PI/180.0);
       GpsState->z_destination = GpsState->pos_z - Cam.x_relative*sin(CompState.degree*M_PI/180.0) + Cam.z_relative*cos(CompState.degree*M_PI/180.0);// + .305*cos(M_PI - CompState.degree); 
//       printf("x: %f z: %f\n", GpsState->x_destination, GpsState->z_destination);
       if(GpsState->distance > .5)
         newState = Forward(newState, time);
//     }
     /*if(NoObst)
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
      }*/
      
  }
  else
  {
     newState = Pledge_Algorithm(LidarState, time);
  }
  key = wb_keyboard_get_key();
  if(key == WB_KEYBOARD_UP)
  {
    newState = Forward(newState, time);
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
  