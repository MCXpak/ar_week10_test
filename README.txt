Code developed by Mourad Lasga Terceras, 190411013

This ros package moves the panda robot's end effector to draw a square in the x-y plane. 

To run the package:

0) Download and install the following packages:
- git clone -b ROS-DISTRO-devel https://github.com/ros-planning/moveit_tutorials.git
- git clone -b ROS-DISTRO-devel https://github.com/ros-planning/panda_moveit_config.git

where ROS-DISTRO is the name of your ROS distribution (e.g. melodic).

1) Unzip the ar_week10_test.zip folder in the src folder of your catkin workspace

2) Build the catkin workspace (catkin_make)

3) Run the following (on four different terminals):

- roslaunch panda_moveit_config demo.launch
- rosrun ar_week10_test square_size_generator.py
- rosrun ar_week10_test move_panda_square.py
- rosrun rqt_plot rqt_plot
