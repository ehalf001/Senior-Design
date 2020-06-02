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

###  Adaptive Walking: Hill Climbing Search
[For Emanuel to fill]

## Auxiliaries 

* ### GPS
    * Using a basic GPS device, our robot is able to find its velocity and relative position on the map.

* ### Compass
    * Using a basic Compass device, our robot is able to recognize its orientation to the map's polar axis.

* ### Camera
    * Using a basic camera with a 144p resolution and a 31.25Hz refresh rate, paired with camera recognition capabilities, our robot scopes out its goal while it traverses the map.
    
* ### Lidar
    * Using the HokuyoUtm30lx Lidar, our robot maps is capable of recognizing walls and obstacles while maneuvering around them as it makes note of them on in its map memory and displaying it on a feed visible to the user.
    * The lidar is set to have a detection range of 1 meter.

### Example of Camera declaration and initialization in controller
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

## Display
Our robot simulation displays 3 auxiliary windows that demonstrate what the robot sees and thinks.
* ### Camera
	* The camera display window shows exactly what the robot sees through the camera. When the camera recognizes an object, it's indentified in a red box that the user can mouse over to show all information that the robot can extrapolate on the object.
* ### Lidar
	* The lidar display window shows our 1080 rays feedback in a 270 degree range in around the robot. 
	* White space indicates that the lidar's laser has bounced off a surface and returned to the lidar. 
	* Purple space indicates that there was no object to bounce the laser off within the set radius of 1 meter.

* ### Map
	*  The map display window is a visual representation of our mapping system.
	* Blue indicates space that the robot has previously occupied.
	* Red indicated walls or obstacles that the robot has recognized.

![readme_camera_lidar_example](https://github.com/ehalf001/Senior-Design/blob/master/README_source/Readme_Camera_Lidar_Example.png)
>Camera display (left) shows known information on recognizable objects while lidar display (right) detects obstacles and translates it into a map that the robot uses to navigate the map.

## Mapping
[for Chris to fill]
* ### Lidar
	* text here

## Decision Tree
The Decision Tree is an algorithm that the robot uses to decide what it should do in any given moment. It takes input from the auxiliaries and controls what system of pathing the robot should use.
* ### Pathing
	* The robot utilizes a famous maze solving algorithm called the Pledge algorithm to traverse a connected maze.
		* The Pledge algorithm has the robot hug the wall to its right as it continues forward, and is able to recognize turns, intersections, and  deadends. 
		* Note: Pledge algorithm only works if the maze is connected.
* ### Goal Recognition 
	* Using camera recognition, when the robot sees that the goal is in view of the robot, it will forgo the pledge algorithm to reorrient itself and bee-line to the goal.

To see a video demonstation of the robot in action, [click here!](https://youtu.be/qBtW_v19oYU) 
