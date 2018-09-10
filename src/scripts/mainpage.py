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
## \subsection det_sec More in details about the architecture and its implementation
## Each Block of the architecture has been implemente as a ROS node.
## @n For comunication between the nodes has been used a Publish/Subscibe messaging pattern.
## @n The behavior have been implemented through a miro_msg of type <a href="https://consequential.bitbucket.io/platform_control.msg">platform_control</a> 
## @n In particular, each behavior sets the body_vel (Miro Body's velocity) and lights_raw (Miro Body's lightening pattern)
