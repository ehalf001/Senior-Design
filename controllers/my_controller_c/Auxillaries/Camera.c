#include <webots/robot.h>
#include <webots/camera.h>

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define TIME_STEP 32

struct Camera
{
    WbDeviceTag Hexabot_Camera;
    int numObjects;
};

struct Camera Camera_Init() 
{
    struct Camera CameraInit;
    //Camera Engable   
    CameraInit.Hexabot_Camera = wb_robot_get_device("Hexabot_Camera");
    wb_camera_enable(CameraInit.Hexabot_Camera, TIME_STEP);
    wb_camera_recognition_enable(CameraInit.Hexabot_Camera, TIME_STEP);
    return CameraInit;
}   

struct Camera Camera_Loop(struct Camera CameraOld) 
{
    struct Camera CameraNew = CameraOld;
    
    // get current number of object recognized by the camera
    CameraNew.numObjects = wb_camera_recognition_get_number_of_objects(CameraNew.Hexabot_Camera);
    printf("\nRecognized %d objects.\n", CameraNew.numObjects);

    // get and display all the objects information
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(CameraNew.Hexabot_Camera);
/*
    for (int i = 0; i < CameraNew.numObjects; ++i) {
      printf("Model of object %d: %s\n", i, objects[i].model);
      printf("Id of object %d: %d\n", i, objects[i].id);
      printf("Relative position of object %d: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
             objects[i].position[2]);
      printf("Relative orientation of object %d: %lf %lf %lf %lf\n", i, objects[i].orientation[0], objects[i].orientation[1],
             objects[i].orientation[2], objects[i].orientation[3]);
      printf("Position of the object %d on the camera image: %d %d\n", i, objects[i].position_on_image[0],
             objects[i].position_on_image[1]);
      for (int j = 0; j < objects[i].number_of_colors; ++j)
        printf("- Color %d/%d: %lf %lf %lf\n", j + 1, objects[i].number_of_colors, objects[i].colors[3 * j],
               objects[i].colors[3 * j + 1], objects[i].colors[3 * j + 2]);
    }      
*/
    return CameraNew;
}

struct Camera Camera_Disable(struct Camera CameraOld)
{
    struct Camera CameraNew = CameraOld;
    wb_camera_disable(CameraNew.Hexabot_Camera);
    return CameraNew;
}