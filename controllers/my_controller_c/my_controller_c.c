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


#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


#include "Auxillaries/Camera.c"
#include "Auxillaries/Lidar.c"
#include "System/DecisionTree.c"
#include "Auxillaries/PathDisplay.c"



int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  //Lidar Enable
  struct Lidar lidar = Lidar_Init();
  
  //Camera Enable
  struct Camera Cam = Camera_Init();

   //GPS Enable
   struct GPS Gps = GPS_Init();

   //Compass Enable
   struct Compass COMP = Compass_Init();
   
   //PathDisplay Enable
   struct PathDisplay path = PathDisplay_Init();
   
   //Walking Enable
   struct Walking WalkPatt = Walking_Init();

   while (wb_robot_step(TIME_STEP) != -1) 
   {
      
      //Camera Update
      Cam = Camera_Loop(Cam);
      
      //Lidar Update
      lidar = Lidar_Loop(lidar);
      
      //GPS Update
      Gps = GPS_Loop(Gps);
      
      //Compass Update
      COMP = Compass_Loop(COMP);
      
      //Path Display Update
      path = PathDisplay_Loop(path,Gps);
      
      //WalkPatt = DecisionTree_Pattern(WalkPatt, Gps, COMP);
      
      float getA = getDisplay(lidar);
      
      if(getA == 280){
        printf("No object here...\n");
      } else {
      
      int d = getA / 270 * 1080;
    
      float dVal = lidar.lidar_utm30lx_values[d] + 0.305;
      
      float objX = Gps.pos_x + dVal * cos(M_PI_2 * (getA + 45 + COMP.degree) / 360);
      float objZ = Gps.pos_z + (-1) * dVal * sin(M_PI_2 * (getA + 45 + COMP.degree) / 360);
      
      
      printf("getA val : %f \n d : %f\n", getA, dVal);
      printf("object x : %f \n object z : %f \n", objX, objZ);
      printf("degree : %f\n", COMP.degree);
     
      }
      
   };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  COMP = Compass_Disable(COMP);
  Gps = GPS_Disable(Gps);
  Cam = Camera_Disable(Cam);
  lidar = Lidar_Disable(lidar);
  wb_robot_cleanup();//

  return 0;
}

