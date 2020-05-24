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

#include "../Auxillaries/GPS.c"
#include "../Auxillaries/Compass.c"
#include "Walking.c"




struct Walking DecisionTree_Pattern(struct Walking Walkstate, struct GPS GpsState, struct Compass CompState)
{
   double time = wb_robot_get_time();
   struct Walking newState = Walkstate;
   
   bool GoalFound = true;
   bool NoObst = true;
   if(GoalFound)
   {
     if(NoObst)
     {
         if(GpsState.quadrant == 1)
         {
            GpsState.angle = 90 - GpsState.angle;
            if((CompState.degree < GpsState.angle && CompState.degree > (GpsState.angle - 180)) && abs(CompState.degree - GpsState.angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState.angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState.distance > .5)
               newState = Foward(newState, time);
           
         }
         else if(GpsState.quadrant == 2)
         {
            GpsState.angle = -90 - GpsState.angle;
            if((CompState.degree < GpsState.angle && CompState.degree > (180 - GpsState.angle)) && abs(CompState.degree - GpsState.angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState.angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState.distance > .5)
               newState = Foward(newState, time);
         }
         else if(GpsState.quadrant == 3)
         {
            GpsState.angle = -90 - GpsState.angle;
            if((CompState.degree < GpsState.angle && CompState.degree > (180 - GpsState.angle)) && abs(CompState.degree - GpsState.angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState.angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState.distance > .5)
               newState = Foward(newState, time);
         }
         else if(GpsState.quadrant == 4)
         {
            GpsState.angle = 90 - GpsState.angle;
            if((CompState.degree < GpsState.angle && CompState.degree > (180 - GpsState.angle)) && abs(CompState.degree - GpsState.angle) > 2)
              newState = TurnRIGHT(newState, time);
            else if(abs(CompState.degree - GpsState.angle) > 2)
               newState = TurnLEFT(newState, time);
            else if(GpsState.distance > .5)
               newState = Foward(newState, time);
         }
      }
    }
    return newState;
}



