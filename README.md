
## CS179J Senior Design

# Adaptive Subterranean Robotic Hexapod Simulation

Our software simulated hexapod will attempt to reach a goal point on a virtual 3D map. We are using terrain mapping and recognition tactics to drive the robot while we improve upon a base walking algorithm with a learning techinique.

### Prerequisites

-   [Webots](https://cyberbotics.com/doc/guide/installation-procedure) (most recent installation)

## Walking

The robot starts with a base walking algorithm that utilizes sinusoidal patterns to walk forwards and backwards, as well as turning clockwise and counter-clockwise. This type of walking is called a tripod gait. This walking is improved upon over time automatically as the robot tries different things in an attempt to traverse the map faster.

Below is an example of how we get our robot to walk with the press of the UP arrow key:

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

### Adaptive Walking: Hill Climbing Search

To use this component of the project, you have to open the adaptive walking world and make sure that the robot is reading into the adaptive walking algorithm controller.

The robot will start with a random set of bounded variables to test its walking. It tests by taking the average velocity over a span of 10 minutes. Then it increments and decrements a combination of those variables and switches to a new combination if the next combination is slower. Once it finds the local max, it stores it and its velocity, on the hill climbing array. It repeats this process 100 times and takes the maximum of those local maximas. Step size is determined at the top of the code. Increasing the step size that it increments variables will increase the time that the simulation will run.

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

This snippet is of storage

## Auxiliaries

-   ### GPS
    
    -   Using a basic GPS device, our robot is able to find its velocity and relative position on the map.

-   ### Compass
    
    -   Using a basic Compass device, our robot is able to recognize its orientation to the map's polar axis.

-   ### Camera
    
    -   Using a basic camera with a 144p resolution and a 31.25Hz refresh rate, paired with camera recognition capabilities, our robot scopes out its goal while it traverses the map.

-   ### Lidar
    
    -   Using the HokuyoUtm30lx Lidar, our robot maps is capable of recognizing walls and obstacles while maneuvering around them as it makes note of them on in its map memory and displaying it on a feed visible to the user.
    -   The lidar is set to have a detection range of 1 meter.

### Example of Camera declaration and initialization in controller

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

All devices are instantiated in this fashion for ease of access and readability.

Each device has 4 associated structs. One for each of object definition, device initialization, device function steps, and device disabling.

## Display

Our robot simulation displays 3 auxiliary windows that demonstrate what the robot sees and thinks.

-   ### Camera
    
    -   The camera display window shows exactly what the robot sees through the camera. When the camera recognizes an object, it's indentified in a red box that the user can mouse over to show all information that the robot can extrapolate on the object.

-   ### Lidar
    
    -   The lidar display window shows our 1080 rays feedback in a 270 degree range in around the robot.
    -   White space indicates that the lidar's laser has bounced off a surface and returned to the lidar.
    -   Purple space indicates that there was no object to bounce the laser off within the set radius of 1 meter.

-   ### Map
    
    -   The map display window is a visual representation of our mapping system.
    -   Blue indicates space that the robot has previously occupied.
    -   Red indicated walls or obstacles that the robot has recognized.

[![readme_camera_lidar_example](https://github.com/ehalf001/Senior-Design/raw/master/README_source/Readme_Camera_Lidar_Example.png)](https://github.com/ehalf001/Senior-Design/blob/master/README_source/Readme_Camera_Lidar_Example.png)

> Camera display (left) shows known information on recognizable objects while lidar display (right) detects obstacles and translates it into a map that the robot uses to navigate the map.

## Mapping

The mapping system is a subsystem the robotic hexapod uses to determine its bearings throughout the simulated arena. On a display, it will draw its relative position on the map while also drawing any obstacles picked up by the lidar. The mapping system makes an array that corresponds to a 1-to-1 mapping of the terrain. 

-   ### Lidar
    
    -   The mapping system uses the lidar to gather information of the terrain it is on. 

## Decision Tree

The Decision Tree is an algorithm that the robot uses to decide what it should do in any given moment. It takes input from the auxiliaries and controls what system of pathing the robot should use.

-   ### Pathing
    
    -   The robot utilizes a famous maze solving algorithm called the Pledge algorithm to traverse a connected maze.
        -   The Pledge algorithm has the robot hug the wall to its right as it continues forward, and is able to recognize turns, intersections, and deadends.
        -   Note: Pledge algorithm only works if the maze is connected.

-   ### Goal Recognition
    
    -   Using camera recognition, when the robot sees that the goal is in view of the robot, it will forgo the pledge algorithm to reorrient itself and bee-line to the goal.

To see a video demonstation of the robot in action, [click here!](https://youtu.be/qBtW_v19oYU)