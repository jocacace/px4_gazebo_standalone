# px4_gazebo_interface

This packages contains the implementation of a gazebo-ros plugin to directly control the propeller velocities of an UAV simulated with the  [PX4 SITL simulator](https://github.com/jocacace/Firmware)

Compiling this package you will generate the shared library: **libgazebo_ros_interface.so**. Including this plugin in your UAV model, you can dirctly send to gazebo the rotor velocity as *rad/s*. 

To test the plugin you can use the file included in the *launch* directory of [sitl_gazebo](https://github.com/jocacace/sitl_gazebo) package:

     $ roslaunch px4 tarot_tilt_salone.launch

In the following the input and the output of the plugin are listed. 

### Model output
- *Orientation* [sensor_msgs::Imu]: on topic */tarot/imu/data* the plugin plubishes the orientation of the UAV 
- *Global position* [sensor_msgs::NavSatFix]: on topic */1tarot/gps/fix* the output of the GPS Of the UAV
- *Local pose* [geometry_msgs::PoseStamped]: on topic */tarot/local_pose* the local position of the UAV

### Model input
- *Propeller velocity* [std_msgs/Float32MultiArray]: on topic */tarot/motor_vel* the desired velocity for UAV propellers must be published. The filed data of this message must be resized to store 4 float values (one for each propeller velocity). An example on how to publish this data is reported in the *example* folder of the package. You can ran the example with the following command:
              
        $ rosrun px4_gazebo_standalone motor_vel_example 

