pioneer_p3dx
============

Code for running the mixed initiative experiments with the pioneer_p3dx robot


Nessery packages and dependancies
---------------------------------

* First install [rosaria] pioneer ROS drivers. Drivers listen for motor commands, provide odometry and other info.

```sh
cd ros_workspace/src
git clone https://github.com/amor-ros-pkg/rosaria.git
cd ros_workspace
rosdep update
rosdep install rosaria
catkin_make
```

* Installation of other things like:
[joystick drivers], [hokuyo laser drivers], [hector_slam], [frontier_exploration].

```sh
sudo apt-get install ros-hydro-joystick-drivers
sudo apt-get install ros-hydro-hokuyo-node
sudo apt-get install ros-hydro-frontier-exploration
sudo apt-get install ros-hydro-hector-slam
```


[rosaria]:http://wiki.ros.org/ROSARIA
[hokuyo laser drivers]:http://wiki.ros.org/hokuyo_node
[joystick drivers]:http://wiki.ros.org/joystick_drivers
[hector_slam]:http://wiki.ros.org/hector_slam
[frontier_exploration]:http://wiki.ros.org/frontier_exploration

