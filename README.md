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
 
 The **Gesture Based behavior** consists in MiRo following the user's command. Hence, the data from the smartwatch's accelerometer are converted into input for MiRo control.
 In particular, depending on the accelerometer values are set some specific lights pattern of Miro's body and Miro's body linear and angular velocities.
 
 Two modalities of control are available: *BASIC* and *ADVANCED*
 The *BASIC* mode is basically a step control that allows Miro to stay still, rotate left/right, go straight forward/backward only. 
 This mode allows a novice user to start getting comfortable with the control gestures and with the Robot responsiveness.
 The *ADVANCED* mode allows a smoother but more sensitive control, enabling combination of basic commands. e.g turn left and go forward
 This mode allows an expert user to perform more complex trajectories and to exert a more natural control.

 The **Obstacle Avoidance behavior** overrides the Gesture Based behavior when and obstacle is detected by using the Robot's Sonar.
 When an Obstacle is detected the Robot's body becomes red to signal the dangerous situation to the user. It starts turning of few degrees until the obstacle is not detected anymore.
 But, before this the user must suggest the robot in wich direction turning.
 Once the collision has been avoided the control goes back to the user.

 #### More in details about the architecture 

 Each Block of the architecture has been implemente as a ROS node.

 For comunication between the nodes has been used a Publish/Subscibe messaging pattern.

 The *imu_mapping* node subscribes to the smartwatch's accelerometer data, maps the linear accellerations into linear and angular velocities and publish them.

 The *gbb_miro* node uses these velocities to publish a message of type platform_control that contains miro body velocity and miro body lights pattern.

 The *oab_miro* node subscibes to sonar sensor to detect the presence of an obstacle, and publish a message of type platform_control that contains miro body velocity and miro body lights pattern. 
 
 The *switching_behavior* node subscribes to both platform_control messages from gbb_miro and oab_miro that corresponds to the two different behaviors.
 It decide which behavior publish on the Robot depending on the presence or not of the obstacle.

 This architecture has been developed to be modular and scalable, since each behavior can be easily modified or substituted, and new behaviours can be easily added.
 Starting from this software architecture another project has been developed with more and complex behaviors, e.g a more complete Obstacle Avoidance strategy based on BUG2 Algorithm has been used as Obstacle Avoidance behavior.

 #Gettin Started

