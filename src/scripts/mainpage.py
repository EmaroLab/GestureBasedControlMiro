##\mainpage
##\section aim_sec The aim of the Project
## The aim of the project is to control the bio-inspired social Robot MiRo with a wearable device.
## In particular a smartwatch with a 9-axis Imu sensor.
## @n The Robot should also be able to avoid collision with obstacle and override the user's control when an obstacle is detected.
## @n This Project has been developed for the Software Architecture course of the master degree program in Robotics Engineering at University of Genoa.
## \section sofar_sec The Software architecture
## The software architecture developed for this project is based on a bio-inspired approach.
## @n The problem has been addressed by using a behaviour-based design pattern.
## @n ADD IMAGE
## @n This architecture has been developed to be modular and scalable, since each behavior can be conveniently modified or substituted, and new behaviours can be easily added.
## @n There are two possible MiRo behaviors addressed in this project:
## <ul><li>Gesture Based behavior</li>
## <li> Obstacle Avoidance behavior</li></ul>
## \subsection gbb_sec Gesture Based Behavior
## The <b>Gesture Based behavior</b> regards the robot's ability to follow the user's command. 
## @n Hence, the data from the smartwatch's accelerometer are converted into input for MiRo control.
## @n In particular, depending on the accelerometer values are set some specific lights pattern of Miro's body and Miro's body linear and angular velocities.
## @n Two modalities of control are available: <i>BASIC</i> and <i>ADVANCED</i>
## <ul><li>The <i>BASIC</i> mode is basically a step control that allows Miro to stay still, rotate left/right, go straight forward/backward only. 
## @n This mode allows a novice user to start getting comfortable with the control gestures and with the Robot responsiveness.</li>
## <li> The *ADVANCED* mode allows a smoother but more sensitive control, enabling combination of basic commands. e.g turn left and go forward
## @n This mode allows an expert user to perform more complex trajectories and to exert a more natural control.</li></ul>
## \subsection aob_sec Obstacle Avoidance Behavior
## The <b>Obstacle Avoidance behavior</b> overrides the Gesture Based behavior when and obstacle is detected by using the Robot's Sonar.
## @n  When an Obstacle is detected the Robot's body becomes red to signal the dangerous situation to the user. It starts turning of few degrees until the obstacle is not detected anymore.
## @n But, before this the user must suggest the robot in wich direction turning.
## Once the collision has been avoided the control goes back to the user.
## \section det_sec The implementation
## The Robot Miro presents itself as a ROS node.
## @n Each module of the architecture has been implemente as a ROS node.
## @n For comunication between the nodes has been used a Publish/Subscibe messaging pattern.
## @n The behavior have been implemented through a miro_msg of type <a href="https://consequential.bitbucket.io/platform_control.msg">platform_control</a> 
## @n In particular, each behavior sets the body_vel (Miro Body's velocity) and lights_raw (Miro Body's lightening pattern)
## \subsection sum_dec Summary on nodes' functioning
## In the File and Class documentation is provided a more deailed description about all the nodes.
## @n
## @n In order to allow the smartwatch to communicate with the ROS Master, the <a href="https://github.com/EmaroLab/imu_stream" >imu_stream</a> app must be installed on both smartwatch and smartphone paired.
## @n The  imu_stream  is  used to stream IMU data from smartwatch to an MQTT broker.
## @n The <a href="https://github.com/EmaroLab/mqtt_ros_bridge/tree/feature/multiple_smartwatches">mqtt_ros_bridge</a> is a bridge that allows to subscribe to MQTT topics and publish contents of MQTT messages to ROS.
## @n @n The imu_data_map.py node subscribes to the  smartwatch’s accelerometer data, maps the linear accelerations into linear and angular velocities and publish them.
## @n The gbb_miro.py node uses these velocities to publish a message of type platform_control.
## @n The oab_miro.py node  subscribes  to  sonar  sensor  to detect  the  presence  of  an  obstacle,  and  publish  a message of type platform_control that contains Miro's body velocity and Miro's body lights pattern.
## @n The switching_behavior_miro.py node  subscribes  to  both platform_control  messages  published  by  gbb_miro.py and oab_miro.py that corresponds to the two different behaviors.
## @n It decides which behavior to publish on the Robot depending on the presence or not of the obstacle.
##\section hiw_sec How to run the code
## The code, the instructions about prerequisites and how to run the installation can be found on <a href="https://github.com/RobertaDelrio/GestureBasedControlMiro">GitHub</a>
##\section team_sec The Team
## Roberta Delrio: <i>roberta.delrio@studio.unibo.it</i>
## @n Sabrina Speranza: <i>sabrina.speranza.ss.@gmail.com</i>
