#include <webots/robot.h>
#include <webots/compass.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

struct Compass
{
    WbDeviceTag Hexabot_Compass;
    double degree;
};

double get_bearing_in_degrees(WbDeviceTag tag) 
{
    const double *north = wb_compass_get_values(tag);
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < -180.0)
      bearing = bearing + 360.0;
     return bearing;
}

struct Compass Compass_Init() 
{
     struct Compass CompassInit;
     //Compass Engable
     CompassInit.Hexabot_Compass = wb_robot_get_device("Hexabot_Compass");
     wb_compass_enable(CompassInit.Hexabot_Compass, 10);
     CompassInit.degree = get_bearing_in_degrees(CompassInit.Hexabot_Compass);
     return CompassInit;
}   

struct Compass Compass_Loop(struct Compass CompassOld) 
{
    struct Compass CompassNew = CompassOld;
    //printf("Degree: %g\n", CompassNew.degree);
    CompassNew.degree = get_bearing_in_degrees(CompassNew.Hexabot_Compass);
    return CompassNew;       
}

struct Compass Compass_Disable(struct Compass CompassOld)
{
    struct Compass CompassNew = CompassOld;
    wb_compass_disable(CompassNew.Hexabot_Compass);
    return CompassNew;
}