
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

#include "Auxillaries/GPS.c"
#include "Auxillaries/Compass.c"
#include "Auxillaries/Camera.c"
#include "Auxillaries/Lidar.c"
#include "System/Walking.c"


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  double time = wb_robot_get_time();
  
  //Lidar Enable
  struct Lidar lidar = Lidar_Init();
  
  //Camera Enable
  struct Camera Cam = Camera_Init();

   //GPS Enable
   struct GPS Gps = GPS_Init();

   //Compass Enable
   struct Compass COMP = Compass_Init();
   
   //Walking Enable
   struct Walking WalkPatt = Walking_Init();

   while (wb_robot_step(TIME_STEP) != -1) 
   {
      time = wb_robot_get_time();
      
      //Camera Update
      Cam = Camera_Loop(Cam);
      
      //Lidar Update
      lidar = Lidar_Loop(lidar);
      
      //GPS Update
      Gps = GPS_Loop(Gps);
      
      //Compass Update
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
  Cam = Camera_Disable(Cam);
  lidar = Lidar_Disable(lidar);
  wb_robot_cleanup();//

  return 0;
}

