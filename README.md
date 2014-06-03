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

* Installation of other things like [joystick drivers] and [hokuyo laser drivers].

```sh
sudo apt-get install ros-hydro-joystick-drivers
sudo apt-get install ros-hydro-hokuyo-node
```


[rosaria]:http://wiki.ros.org/ROSARIA
[hokuyo laser drivers]:http://wiki.ros.org/hokuyo_node
[joystick drivers]:http://wiki.ros.org/joystick_drivers

