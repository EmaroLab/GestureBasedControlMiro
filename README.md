 # Gesture Based Control of MiRo Robot

 ## The Project
 The main goal of the project is to guide MiRo using a wearable device though specific gestures.
 The Robot must be able to override the users command when an obstacle is detected in order to avoid collision with it.
 This Project has been developed for the Software Architecture course of the master degree program in Robotics Engineering at University of Genoa.

 ### MiRo Companion Robot
 MiRo is a animal-like robot developed as a prototype companion.
 It was designed with a bio-inspired architecture based on the neuroscience knowledge of the mammalian brain.

 ### The wearable device
 In order to guide MiRo through gestures, a smartwatch with a 9-axis IMU sensor has been used.
 [LG G WATCH R](https://www.lg.com/wearable-technology/lg-G-Watch-R-W110)

 ### The architecture
 The software architecture developed for this project is based on a bio-inspired approach.
 The problem has been addressed by using a behaviour-based design pattern.
 There are two possible MiRo behaviors addressed in this project:
 * Gesture Based behavior
 * Obstacle Avoidance behavior
 
 The **Gesture Based behavior** regards the robot's ability to follow the user's command. The user must wear a smartwatch in order to control
Miro by performing specific gestures. Basically, the data from the smartwatch's accelerometer are converted into input for MiRo control.
 In particular, depending on the accelerometer values are set some specific lights pattern of Miro's body and Miro's body linear and angular velocities.
 
 Two modalities of control are available: *BASIC* and *ADVANCED*
 * The *BASIC* mode is basically a step control that allows Miro to stay still, rotate left/right, go straight forward/backward only. 
 This mode allows a novice user to start getting comfortable with the control gestures and with the Robot responsiveness.
 * The *ADVANCED* mode allows a smoother but more sensitive control, enabling combination of basic commands. e.g turn left and go forward
 This mode allows an expert user to perform more complex trajectories and to exert a more natural control.

 The **Obstacle Avoidance behavior** overrides the Gesture Based behavior when and obstacle is detected by using the Robot's Sonar.
 When an Obstacle is detected the Robot's body becomes red to signal the dangerous situation to the user. It starts turning of few degrees until the obstacle is not detected anymore.
 But, before this the user must suggest the robot in wich direction turning.
 Once the collision has been avoided the control goes back to the user.

 ## The implementation 

 Each module which is part of the architecture has been implemente as a ROS node.

 For comunication between the nodes has been used a Publish/Subscibe messaging pattern.

In order to allow the smartwatch to communicate with the ROS Master, the imu
stream app must be installed on both smartwatch and smartphone paired.
The  imu stream  is  used  to  stream  IMU  data  from smartwatch to an MQTT broker.
The  mqtt_ros_bridge is  a  bridge  that  allows  to subscribe to MQTT topics and publish contents of MQTT messages to ROS.

 The *imu_data_map* node subscribes to the smartwatch's accelerometer data published by MQTT broker and maps the linear accellerations into linear and angular velocities and publish them.

 The *gbb_miro* node uses these velocities to publish a message of type platform_control that contains miro body velocity and miro body lights pattern.

 The *oab_miro* node subscibes to sonar sensor to detect the presence of an obstacle, and publish a message of type platform_control that contains miro body velocity and miro body lights pattern. 
 
 The *switching_behavior* node subscribes to both platform_control messages from gbb_miro and oab_miro that corresponds to the two different behaviors.
 It decides which behavior to publish on the Robot depending on the presence or not of the obstacle.

 This architecture has been developed to be modular and scalable, since each behavior can be easily modified or substituted, and new behaviours can be easily added.
 Starting from this software architecture another project has been developed with more and complex behaviors, *e.g a more complete Obstacle Avoidance strategy based on BUG2 Algorithm has been used as Obstacle Avoidance behavior.*

 # Gettin Started

 ## Prerequisites

 ### ROS
This project is developed using [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu):
* rosdistro: kinetic
* rosversion: 1.12.13

### Smartwatch and Smartphone Setup
In order to publish imu sensor data from your smartwatch to ROS nodes you must have a smartwatch paired with a smartphone.
The smartphone acts as the bridge between the smartwatch and the ros master running on your computer.

Follow the instructions reported in [imu_stream](https://github.com/EmaroLab/imu_stream) to download the app for both the smartphone and the smartwatch.

### MQTT ROS Bridge

In order to succesfully subscribe to MQTT topics and publish contents of MQTT messages to ROS follow the instruction in [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge/tree/feature/multiple_smartwatches).
To work with the current project some parameter must be modified in the imu_bridge.launch 
The parameter device_name must be changed with the name of your personal smartwatch. 

### MiRo Workstation Setup
Download the [Miro Developer kit](http://labs.consequentialrobotics.com/miro/mdk/).

Follow the instructions from Consequential Robotics [Miro: Prepare Workstation](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html) to set up your workstation to work with miro. 
Strictly follow the instructions in the Install MDK section as the following steps will rely on this.

Not necessary to make static IP for your workstation (laptop) while setting up connection with MiRo.

### Gesture Based Control MiRo

Create a catkin workspace and clone all the packages in the src folder

```
$ git clone https://github.com/RobertaDelrio/GestureBasedControlMiro.git
$ cd GestureBasedControlMiro
$ catkin_make
$ source devel/setup.bash
```

## Run the Project

Open a new terminal and launch

```
$ roscore
```
mosquitto must be running on your PC for the birdge to work.

In a new terminal
```
$ mosquitto
```
Make sure that the IP in the IMU_stream app on the smartphone is the same showed by doing

```
$ ifconfig
```

Open the IMU_stream app on the smartwatch 

Launch the mqtt_bridge in a new terminal
```
$ cd catkin_ws
$ cd source devel/setup.bash
$ roslaunch mqtt_bridge imu_bridge.launch
```
To test if the connection between smartwatch and ROS is working start to transmitt the data from IMU_stream app on the smartwatch and check in a new terminal
```
$ rostopic echo \imu_left_hand
```
It should show the Imu data published by the smartwatch.

Connect the Miro robot to the ROS Master

```
$ ssh root@<MIRO-IP> 
$ sudo nano ./profile
```
Insert your IP after ROS_MASTER_IP

For more detailed instructions see [MIRO: Commission MIRO](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html)

The following command will start the project

```
$ roslaunch miro_sofar miro_sofar.launch
```
Parameters that can be set in the launch file ( all of them are explained inside the launch file itself):
* robot_name <--  Select if use real miro or simulated robot in Gazebo
* node_rate
* control_mode <-- Select the BASIC or ADVANCED mode
* sonar_treshold <-- Select the values below which an obstacle is detected

## Works based on the current Project
* Developement of a Pet-like Behavior for Miro Robot --> https://github.com/hoodedapollo/MiroBehaviours

## Acknowledgments

* [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge) 
* [imu_stream](https://github.com/EmaroLab/imu_stream)


### Team
* Roberta Delrio *roberta.delrio@studio.unibo.it*
* Sabrina Speranza *sabrina.speranza.ss.@gmail.com*