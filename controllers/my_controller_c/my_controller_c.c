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

#include "System/DecisionTree.c"
#include "System/MappingSystem.c"

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  wb_keyboard_enable(TIME_STEP);
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
   
  //MappingSystem Enable
  struct Map map = MappingSystem_Init();
  
  
  while (wb_robot_step(TIME_STEP) != -1) 
   {
      
      
      //Lidar Update
      lidar = Lidar_Loop(lidar);
      
      //GPS Update
      Gps = GPS_Loop(Gps);
      
      //Camera Update
      Cam = Camera_Loop(Cam);

      //Compass Update
      COMP = Compass_Loop(COMP);
      
      //Path Display Update
      path = PathDisplay_Loop(path,Gps);
      
      WalkPatt = DecisionTree_Pattern(WalkPatt, &Gps, COMP, Cam);
      
      
      
      MappingSystem(map,lidar,Gps,COMP,path);
      
   };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  COMP = Compass_Disable(COMP);
  Gps = GPS_Disable(Gps);
  Cam = Camera_Disable(Cam);
  lidar = Lidar_Disable(lidar);
  wb_keyboard_disable();
  wb_robot_cleanup();//

  return 0;
}

