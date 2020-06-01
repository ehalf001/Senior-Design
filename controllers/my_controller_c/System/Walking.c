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
#include <webots/motor.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define TIME_STEP 32

struct Walking {
   double f;
   double a[18];
   double p[18];
   double d[18];
   WbDeviceTag Hexabot_Motors[18];
   double x_destination, z_destination;
};

struct Walking Walking_Init()
{
    struct Walking WalkInit;
    WalkInit.f = .5;
    double aa[18] = {.1, .1, -.1, -.1, -.1, .1, .1, .1, -.1, .1, .1, -.1, -.1, -.1, .1, .1, .1, -.1};
    double pp[18] = {0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5, 0, 2, 2.5};
    double dd[18] = {-.1, -.2, -.3, .5, -.2, -.3, 1.1, -.2, -.3, -.1, -.2, -.3, 0, -.2, .1, 0, -.2, .1};
    WbDeviceTag Motors[18]= {wb_robot_get_device("Hexabot_Leg0_Motor1"), wb_robot_get_device("Hexabot_Leg0_Motor2"), wb_robot_get_device("Hexabot_Leg0_Motor3"),
                             wb_robot_get_device("Hexabot_Leg1_Motor1"), wb_robot_get_device("Hexabot_Leg1_Motor2"), wb_robot_get_device("Hexabot_Leg1_Motor3"),
                             wb_robot_get_device("Hexabot_Leg2_Motor1"), wb_robot_get_device("Hexabot_Leg2_Motor2"), wb_robot_get_device("Hexabot_Leg2_Motor3"), 
                             wb_robot_get_device("Hexabot_Leg3_Motor1"), wb_robot_get_device("Hexabot_Leg3_Motor2"), wb_robot_get_device("Hexabot_Leg3_Motor3"),
                             wb_robot_get_device("Hexabot_Leg4_Motor1"), wb_robot_get_device("Hexabot_Leg4_Motor2"), wb_robot_get_device("Hexabot_Leg4_Motor3"),
                             wb_robot_get_device("Hexabot_Leg5_Motor1"), wb_robot_get_device("Hexabot_Leg5_Motor2"), wb_robot_get_device("Hexabot_Leg5_Motor3")
                            };
    int i;//for loop
    for(i = 0; i < 18; ++i)
    {
      WalkInit.a[i] = aa[i];
      WalkInit.p[i] = pp[i];
      WalkInit.d[i] = dd[i];
      WalkInit.Hexabot_Motors[i] = Motors[i];
    }
    for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
           wb_motor_set_position(WalkInit.Hexabot_Motors[i], WalkInit.a[i] * sin(2.0 * M_PI * WalkInit.f * 1 + WalkInit.p[i]) + WalkInit.d[i]);
   WalkInit.x_destination = 100;
   WalkInit.z_destination = 100;
   return WalkInit;
};

struct Walking Forward(struct Walking Robot, double time)
{
    struct Walking Ret = Robot;
    int i;
    for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
       wb_motor_set_position(Ret.Hexabot_Motors[i], Robot.a[i] * sin(2.0 * M_PI * Robot.f * time + Robot.p[i]) + Robot.d[i]);
    return Ret;
}

struct Walking Backward(struct Walking Robot, double time)
{
    struct Walking Ret = Robot;
    int i;
    for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
       wb_motor_set_position(Ret.Hexabot_Motors[i], Robot.a[i] * sin(-2.0 * M_PI * Robot.f * time + Robot.p[i]) + Robot.d[i]);
    return Ret;
}

struct Walking TurnLEFT(struct Walking Robot, double time)
{
    struct Walking Ret = Robot;
    int i;
    for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
       wb_motor_set_position(Ret.Hexabot_Motors[i], Robot.a[i] * sin(2.0 * M_PI * Robot.f * time + Robot.p[i]) + Robot.d[i]);
    for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
       wb_motor_set_position(Ret.Hexabot_Motors[i], Robot.a[i] * sin(-2.0 * M_PI * Robot.f * time + Robot.p[i]) + Robot.d[i]);
    return Ret;
}

struct Walking TurnRIGHT(struct Walking Robot, double time)
{
    struct Walking Ret = Robot;
    int i;
    for (i = 0; i < 9; ++i)  // Apply a sinuosidal function for each motor.
        wb_motor_set_position(Ret.Hexabot_Motors[i], Robot.a[i] * sin(-2.0 * M_PI * Robot.f * time + Robot.p[i]) + Robot.d[i]);
    for (i = 9; i < 18; ++i)  // Apply a sinuosidal function for each motor.
        wb_motor_set_position(Ret.Hexabot_Motors[i], Robot.a[i] * sin(2.0 * M_PI * Robot.f * time + Robot.p[i]) + Robot.d[i]);
    return Ret;
}

