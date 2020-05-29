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

#include "../Auxillaries/Lidar.c"
#include "../Auxillaries/PathDisplay.c"

//double oldT = wb_robot_get_time();
int cnt = 0;
double objX, objZ;

struct Map{
  bool loop;
};

struct Map MappingSystem_Init(){
  struct Map NewMappingSystem;
  NewMappingSystem.loop = false;
  return NewMappingSystem;
}

void MappingSystem(struct Map map, struct Lidar lidar, struct GPS Gps, struct Compass COMP, struct PathDisplay path){
    int i;
    for(i = 0; i < 1080; ++i){ 
      if(lidar.lidar_utm30lx_values[i] < .75){
        float getA = i / 1080.0 * 270.0;
        float dVal = lidar.lidar_utm30lx_values[i];
        double phi = 135 - COMP.degree - getA;
        //printf("Phi val : %f \n", phi);
        phi = M_PI * phi / 180;
        objZ = Gps.pos_z - dVal*sin(phi);
        objX = Gps.pos_x + dVal*cos(phi);
        //printf("object x : %f \nobject z : %f \n", objX, objZ);
        path = updateDisplayObstacles(path,objZ,objX);
        }
    }     
    
    if(cnt > 20){
      int coordinate = path.width * (Gps.pos_z + GROUND_Z / 2) / GROUND_Z + 64*(path.height * (Gps.pos_x + GROUND_X / 2) / GROUND_X);

      if(path.color[coordinate] == 1){
        map.loop = true;
      } else { map.loop = false; }
      cnt = 0;
    } else {cnt++;}
}