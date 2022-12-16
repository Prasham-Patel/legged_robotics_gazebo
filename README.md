# Legged robot simulation - Python Gazebo

This repository contains all the files and code needed to simulate the notspot quadrupedal robot using [Gazebo](http://gazebosim.org/)  and [ROS](https://www.ros.org/).
The software runs on [ROS noetic](http://wiki.ros.org/noetic) and [Ubuntu 20.04](http://www.releases.ubuntu.com/20.04/). If you want to use a different ROS version, you might have to do some changes to the source code.

<img src="resources/notspot_render_new1" width="233"> <img src="resources/notspot_render_new2" width="233"> <img src="resources/notspot_render_new3" width="233"> 

## Setup
```
cd src && catkin_init_workspace
cd .. && catkin_make
source devel/setup.bash
roscd notspot_controller/scripts && chmod +x robot_controller_gazebo.py
cp -r RoboticsUtilities ~/.local/lib/python3.8/site-packages
roscd notspot_joystick/scripts && chmod +x ramped_joystick.py
```

In another terminal
```
source devel/setup.bash
roslaunch notspot run_robot_gazebo.launch
```

After all the nodes have started, you can click on the small TK window that pops up and control forward velocity with the up and down arrows and the rotational velocity with the left and right arrows 

There is a global boolean USE_JOY that sets if the simulation follows a pre-planned trajecotry or if it will follow user input

If the simulation does not change by switching USE_JOY, use this branch [cubi_poly_traj](https://github.com/Prasham-Patel/legged_robotics_gazebo/tree/cubic_poly_traj) to simulate pre-planned trajectory.

## Credits
 - mike4192: https://github.com/mike4192/spotMicro
 - https://github.com/lnotspotl/notspot_sim_py
