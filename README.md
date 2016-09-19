# Team 9 Racecar
Autonomous racecar project at MIT. The RACECAR platform runs on Robot Operating System (ROS).  

Nvidia Jetson TX 1 embedded super-computer with Hokuyo UST-10LX laser range finder, Stereolabs ZED stereo camera, Structure.io depth camera, Sparkfun IMU.


SETUP
-------
- 1) Go to shell-scripts and install the awesomeness
- 2) Install dependencies:
```
sudo apt-get install ros-indigo-geographic-msgs
sudo apt-get install ros-indigo-tf2-geometry-msgs
```

Running
-------
```
roslaunch launcher racecar_teleop.launch
roslaunch navstack navstack.launch
rosrun navstack freespace_goalplanner.py
```

Demo
-------
<img src="img/intro.gif">
<img src="img/vision.gif">
<img src="img/driving.gif">

Full Result: https://www.youtube.com/watch?v=IVsP1LJmKhw


