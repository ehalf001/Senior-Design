## CS179J Senior Design
# Adaptive Subterranean Robotic Hexapod Simulation

Our software simulated hexapod will attempt to reach a goal point on a virtual 3D map.
We are using terrain mapping and recognition tactics to drive the robot while we improve upon a base walking algorithm with a learning techinique. 

### Prerequisites
* [Webots](https://cyberbotics.com/doc/guide/installation-procedure) (most recent installation)

## Walking
The robot starts with a base walking algorithm that utilizes sinusoidal patterns to walk forwards and backwards, as well as turning clockwise and counter-clockwise. This walking is improved upon over time automatically as the robot tries different things in an attempt to traverse the map faster.

Below is an example of how we get our robot to walk with the press of the UP arrow key: 
```c
/* 
> a is an array of amplitudes  (will be altered in learning approach)
> M_PI is the value of pi
> f is the frequency (usually between .5 and 1)
> time is time
> p is an array of offset values Φ (will be altered in learning approach)
> d is an array of physical offset values
*/
if(key == WB_KEYBOARD_UP)
    for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
        wb_motor_set_position(Hexabot_Motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
```
## auxiliaries 

* GPS
    * Using a basic GPS device, our robot is able to find its velocity and relative position on the map.

* Compass
    * Using a basic Compass device, our robot is able to recognize its orientation to the map's polar axis.

> example of compass declaration and initialization in controller
```c
struct Camera{
    WbDeviceTag Hexabot_Camera;
    int numObjects;
};

struct Camera Camera_Init() {
    struct Camera CameraInit;
    //Camera Enable   
    CameraInit.Hexabot_Camera = wb_robot_get_device("Hexabot_Camera");
    wb_camera_enable(CameraInit.Hexabot_Camera, TIME_STEP);
    wb_camera_recognition_enable(CameraInit.Hexabot_Camera, TIME_STEP);
    return CameraInit;
} 
```
All devices are instantiated in this fashion for ease of access and readability.

Each device has 4 associated structs. One for each of object definition, device initialization, device function steps, and device disabling.

* Camera
    * Using a basic camera with a 144p resolution and a 31.25Hz refresh rate, paired with camera recognition capabilities, our robot scopes out its goal while it traverses the map.
    
* Lidar
    * Using the HokuyoUtm30lx Lidar, our robot maps is capable of recognizing walls and obstacles while maneuvering around them as it makes note of them on in its map memory and displaying it on a feed visible to the user.
![readme_camera_lidar_example](https://github.com/ehalf001/Senior-Design/blob/master/README_source/Readme_Camera_Lidar_Example.png)
>Camera display (left) shows known information on recognizable objects while lidar display (right) detects obstacles and translates it into a map that the robot uses to navigate the map.

## Pathing
text here
### Obstacle Detection
text here
#### Camera
text here

## Mapping
text here
### Lidar
text here

## Display
text here
### Camera
text here
### Lidar
text here
### Map
text here
