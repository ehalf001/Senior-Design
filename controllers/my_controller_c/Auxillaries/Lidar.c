#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/display.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define TIME_STEP 32

#define purple 0xC030C0
#define black 0x000000
#define gray 0xC0C0C0
#define white 0xFFFFFF

// the lidar values are clamped between 0 and CLAMP_MAX
#define CLAMP_MAX 1.0

// function retrieving the min value between x and y
#define MIN(x, y) ((x) > (y) ? (y) : (x))

// function displaying lidar values on a display
// display, width, height, lidar vals, num of samples, fov
void Display(WbDeviceTag d, int dw, int dh, const float *v, int ns, float fov){
  int dw2 = dw / 2;
  int dh2 = dh / 2;
  float fov2 = fov / 2;
  const int px[] = {dw2, dw2 + dw * cos(-fov2 - M_PI_2), dw2 + dw * cos(fov2 - M_PI_2)};
  const int py[] = {dh2, dh2 + dh * sin(-fov2 - M_PI_2), dh2 + dh * sin(fov2 - M_PI_2)};

  wb_display_set_color(d, white);
  wb_display_fill_rectangle(d, 0, 0, dw, dh);
  wb_display_set_color(d, gray);
  wb_display_fill_polygon(d, px, py, 3);
  wb_display_set_color(d, black);
  wb_display_draw_polygon(d, px, py, 3);
  wb_display_draw_line(d, dw2, 0, dw2, dh);
  wb_display_draw_line(d, 0, dh2, dw, dh2);
  wb_display_draw_oval(d, dw2, dh2, dw2, dh2);
  wb_display_set_color(d, purple);

  int i;
  for (i = 0; i < ns; i++) {
    float f = MIN(CLAMP_MAX, v[i]);
    float alpha = -fov2 + fov * i / ns - M_PI_2;
    wb_display_draw_line(d, dw2, dh2, dw2 + f * cos(alpha) * dw2 / CLAMP_MAX, dh2 + f * sin(alpha) * dh2 / CLAMP_MAX);
  }
}

struct Lidar
{
    WbDeviceTag Hexabot_Lidar, Display_Lidar;
    int lidar_utm30lx_samples, display_width, display_height;
    double lidar_utm30lx_fov;
};

struct Lidar Lidar_Init() 
{
    struct Lidar LidarInit;
    //Lidar Engable   
    LidarInit.Hexabot_Lidar = wb_robot_get_device("Hokuyo UTM-30LX");
    LidarInit.Display_Lidar = wb_robot_get_device("Lidar_Display");
    wb_lidar_enable(LidarInit.Hexabot_Lidar, TIME_STEP);
    LidarInit.lidar_utm30lx_samples = wb_lidar_get_horizontal_resolution(LidarInit.Hexabot_Lidar);
    LidarInit.lidar_utm30lx_fov = wb_lidar_get_fov(LidarInit.Hexabot_Lidar);
    LidarInit.display_width = wb_display_get_width(LidarInit.Display_Lidar);
    LidarInit.display_height = wb_display_get_height(LidarInit.Display_Lidar);
    return LidarInit;
}   

struct Lidar Lidar_Loop(struct Lidar LidarOld) 
{
    struct Lidar LidarNew = LidarOld;
    
    const float *utm30lx_values = wb_lidar_get_range_image(LidarNew.Hexabot_Lidar);
    Display(LidarNew.Display_Lidar, 
            LidarNew.display_width, 
            LidarNew.display_height, 
            utm30lx_values, 
            LidarNew.lidar_utm30lx_samples, 
            LidarNew.lidar_utm30lx_fov);    
    return LidarNew;
}

struct Lidar Lidar_Disable(struct Lidar LidarOld)
{
    struct Lidar LidarNew = LidarOld;
    wb_lidar_disable(LidarNew.Hexabot_Lidar);
    return LidarNew;
}