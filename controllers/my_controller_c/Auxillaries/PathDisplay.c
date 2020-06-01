#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/display.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define GROUND_X 10.0
#define GROUND_Z -10.0

#define white 0xFFFFFF
#define black 0x000000
#define blue 0x2222BB
#define red 0xFF0000

#define free_space 0
#define walked 1
#define obstacle 2

struct PathDisplay{
  WbDeviceTag Hexabot_PathDisplay;
  int height, width;
  int color[4096];
 // int wallProb[4096];
};

struct PathDisplay PathDisplay_Init() {
  struct PathDisplay PathDisplayInit;
  int i;
  for(i = 0; i < 4096; i++)
  {
    PathDisplayInit.color[i] = free_space;
   // PathDisplayInit.wallProb = free_space;
  }
  
  PathDisplayInit.Hexabot_PathDisplay = wb_robot_get_device("Path_Display");
  
  PathDisplayInit.width = wb_display_get_width(PathDisplayInit.Hexabot_PathDisplay);
  PathDisplayInit.height = wb_display_get_height(PathDisplayInit.Hexabot_PathDisplay);
  
  wb_display_set_color(PathDisplayInit.Hexabot_PathDisplay, white);
  wb_display_fill_rectangle(PathDisplayInit.Hexabot_PathDisplay, 0, 0, PathDisplayInit.width, PathDisplayInit.height);
  wb_display_set_opacity(PathDisplayInit.Hexabot_PathDisplay, 0.05);
  wb_display_set_color(PathDisplayInit.Hexabot_PathDisplay, black);
  wb_display_draw_line(PathDisplayInit.Hexabot_PathDisplay, 0, PathDisplayInit.height / 2, PathDisplayInit.width - 1, PathDisplayInit.height / 2);
  wb_display_draw_text(PathDisplayInit.Hexabot_PathDisplay, "z", PathDisplayInit.width - 10, PathDisplayInit.height / 2 - 10);
  wb_display_draw_line(PathDisplayInit.Hexabot_PathDisplay, PathDisplayInit.width / 2, 0, PathDisplayInit.width / 2, PathDisplayInit.height - 1);
  wb_display_draw_text(PathDisplayInit.Hexabot_PathDisplay, "x", PathDisplayInit.width / 2 - 10, PathDisplayInit.height - 10);

  return PathDisplayInit;
}

struct PathDisplay PathDisplay_Loop(struct PathDisplay PathDisplayOld, struct GPS gps) {
    struct PathDisplay PathDisplayNew = PathDisplayOld;
    // display the robot position
    wb_display_set_color(PathDisplayNew.Hexabot_PathDisplay, blue);
    wb_display_draw_pixel(PathDisplayNew.Hexabot_PathDisplay, PathDisplayNew.width * (((-1) * gps.pos_z) + GROUND_Z / 2) / GROUND_Z,
                         PathDisplayNew.height * (((-1) * gps.pos_x) + GROUND_X / 2) / GROUND_X);

    int coordinate = PathDisplayNew.width * (gps.pos_z + GROUND_Z / 2) / GROUND_Z + 64*(PathDisplayNew.height * (gps.pos_x + GROUND_X / 2) / GROUND_X);
    PathDisplayNew.color[coordinate] = walked;
    return PathDisplayNew;
}

struct PathDisplay updateDisplayObstacles(struct PathDisplay path, double x, double z, bool wall){
  struct PathDisplay PathDisplayNew = path;
  int coordinate = PathDisplayNew.width * (z + GROUND_Z / 2) / GROUND_Z + 64*(PathDisplayNew.height * (x + GROUND_X / 2) / GROUND_X);
  PathDisplayNew.color[coordinate] = obstacle;
  wb_display_set_opacity(PathDisplayNew.Hexabot_PathDisplay, 1);
  wb_display_set_color(PathDisplayNew.Hexabot_PathDisplay, red);
  wb_display_draw_pixel(PathDisplayNew.Hexabot_PathDisplay, PathDisplayNew.width * (x + GROUND_X / 2) / GROUND_X,
                        PathDisplayNew.height * (z + GROUND_Z / 2) / GROUND_Z); 
  return PathDisplayNew;
}
