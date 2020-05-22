#include <webots/robot.h>
#include <webots/gps.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

struct GPS
{
  WbDeviceTag Hexabot_GPS;
  double pos_x, pos_y, pos_z, vel, distance, angle;
  double *pos, x_destination, z_destination;
  int quadrant;
};

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

struct GPS GPS_Init() 
{
  /* necessary to initialize webots stuff */

   //GPS Enable
   struct GPS initGPS;
   initGPS.Hexabot_GPS = wb_robot_get_device("Hexabot_GPS");
   wb_gps_enable(initGPS.Hexabot_GPS,10);
   initGPS.pos = wb_gps_get_values(initGPS.Hexabot_GPS);
   initGPS.pos_x = initGPS.pos[0];
   initGPS.pos_y = initGPS.pos[1];
   initGPS.pos_z = initGPS.pos[2];
   initGPS.vel = wb_gps_get_speed(initGPS.Hexabot_GPS);
   initGPS.x_destination = 3.04;
   initGPS.z_destination = 2.99;
   return initGPS;
}   

struct GPS GPS_Loop(struct GPS oldGPS)
{
      struct GPS newGPS = oldGPS;
 
      newGPS.angle = (atan((oldGPS.x_destination - oldGPS.pos_x)/(oldGPS.z_destination - oldGPS.pos_z))) / M_PI * 180.0;
      newGPS.distance = sqrt(pow((oldGPS.x_destination - oldGPS.pos_x),2) + pow((oldGPS.z_destination - oldGPS.pos_z),2));
        
      newGPS.pos_x = oldGPS.pos[0];
      newGPS.pos_y = oldGPS.pos[1];
      newGPS.pos_z = oldGPS.pos[2];
      
      newGPS.vel = wb_gps_get_speed(oldGPS.Hexabot_GPS);
      
      newGPS.quadrant = Quadrant(oldGPS.pos_x, oldGPS.pos_z, oldGPS.x_destination, oldGPS.z_destination);

      if (newGPS.vel >= 0.01)
      {
        printf("Postion: x[%g] y[%g] z[%g]\n", newGPS.pos_x, newGPS.pos_y, newGPS.pos_z);
        printf("Angle: r[%g]\n", newGPS.angle); 
        printf("Quadrant: %d\n", newGPS.quadrant);
        printf("Distance: %g\n", newGPS.distance);
      }
      return newGPS;
}

struct GPS GPS_Disable(struct GPS oldGPS)
{
  struct GPS newGPS = oldGPS;
  wb_gps_disable(newGPS.Hexabot_GPS);
  return newGPS;
}
