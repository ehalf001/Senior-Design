/*
 * File:          Adaptive Walking Algorithm (WIP)
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

#define STEP_SIZE .1

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  
  wb_robot_init();
  
  time_t t;
  srand((unsigned) time(&t));
  
  //GPS Enable
  struct GPS Gps = GPS_Init();
  
  float hillclimbing_Search[100][7];
  hillclimbing_Search[0][0] = .1; //a1
  hillclimbing_Search[0][1] = .1; //a2
  hillclimbing_Search[0][2] = .1; //a3
  hillclimbing_Search[0][3] = 0; //p1
  hillclimbing_Search[0][4] = 2; //p2
  hillclimbing_Search[0][5] = 2.5; //p3
  hillclimbing_Search[0][6] = .062952; //Average Velocity
   
  //Walking Enable
  struct Walking WalkPatt = Walking_Init(.1,.1,.1,0,2,2.5);

  int i = 0;
  int j = 0;
  int k = 0;
  int tCount = 1;
  float totalVel = 0;
  float avgVel = 0;
  bool localMax = false;
  float search[7] = {.1,.1,.1,0,2,2.5,-1};
  while (wb_robot_step(TIME_STEP) != -1) 
   {
      if(j == 100)
        break;
      double time = wb_robot_get_time();
      Gps = GPS_Loop(Gps);      
      i += 1;
      totalVel += Gps.vel;
      avgVel = totalVel / i;
      //Walk Forward
      WalkPatt = Foward(WalkPatt, time);
      
      
      
      if(time == (600*tCount))
      {
          printf("Current velocity: %f\n", avgVel);
          printf("Current Vars a1: %f a2: %f a3: %f\n p1: %f p2: %f p3: %f\n",
                 search[0], search[1], search[2],
                 search[3], search[4], search[5]);
          tCount += 1;
          if(avgVel > search[6])
          {
            search[6] = avgVel;
            hillclimbing_Search[j][0] = search[0];
            hillclimbing_Search[j][1] = search[1];
            hillclimbing_Search[j][2] = search[2];
            hillclimbing_Search[j][3] = search[3];
            hillclimbing_Search[j][4] = search[4];
            hillclimbing_Search[j][5] = search[5];
            hillclimbing_Search[j][6] = search[6];
          }
          else
          {
            k += 1;
          }
          printf("k value: %d\n",k);
          if(k == 14)
          {
            k = 0;
            localMax = true;
          }
          else if(k > 6)
          {
            if(search[k % 7] > 0)
              search[k % 7] = search[k % 7] - STEP_SIZE;
          }
          else
          {
            if(k % 7  < 3) //a1 - a3 upper bound
            {
              if(search[k % 7] < (.4 - STEP_SIZE))
                search[k % 7] = search[k % 7] + STEP_SIZE;
            }
            else if(k % 7  < 4) //p1 upper bound
            {
              if(search[k % 7] == 0)
                search[k % 7] = search[k % 7] + STEP_SIZE;
            }
            else if(k % 7  < 5) //p2 upper bound
            {
              if(search[k % 7] < (2.4 - STEP_SIZE))
                search[k % 7] = search[k % 7] + STEP_SIZE;
            }
            else if(k % 7  < 4) //p3 upper bound
            {
              if(search[k % 7] < (2.9 - STEP_SIZE))
                search[k % 7] = search[k % 7] + STEP_SIZE;
            }
            else
              search[k % 7] = search[k % 7] + STEP_SIZE;
          }
          WalkPatt = Walking_Init(search[0],search[1],search[2],search[3],search[4],search[5]);
 
      }
      
      if(localMax)
      {
          
          j += 1;
          search[0] = ((rand() % 4) + 1)/ 10.0;  //a1
          search[1] = ((rand() % 4) + 1)/ 10.0;  //a2
          search[2] = ((rand() % 4) + 1)/ 10.0;  //a3
          search[3] = ((rand() % 2))/ 10.0;  //p1
          search[4] = ((rand() % 10) + 15)/ 10.0; //p2
          search[5] = ((rand() % 10) + 20)/ 10.0; //p3
          search[6] = -1;
          WalkPatt = Walking_Init(search[0],search[1],search[2],search[3],search[4],search[5]);
          localMax = false; 
      }
   };
  float velMax = 0.0;
  for(j = 0; j < 100; j += 1) //finds maximum of local maximas
    if(velMax < hillclimbing_Search[j][6])
    {
      velMax = hillclimbing_Search[j][6];
      i = j;
    }
  
  printf("Maximum velocity: %f\n", velMax);
  printf("Variables a1: %f a2: %f a3: %f\n p1: %f p2: %f p3: %f\n",
         hillclimbing_Search[i][0], hillclimbing_Search[i][1], hillclimbing_Search[i][2],
         hillclimbing_Search[i][3], hillclimbing_Search[i][4], hillclimbing_Search[i][5]);
  Gps = GPS_Disable(Gps);
  wb_robot_cleanup();//

  return 0;
}

