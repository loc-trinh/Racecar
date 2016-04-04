##Installing 
	Run:
		sudo apt-get install ros-indigo-teb-local-planner
		catkin_make
		rosdep install teb_local_planner  to install dependences
	install the navigation stack:
		git clone https://github.com/ros-planning/navigation/tree/indigo-devel.git
		catkin_make
		rosdep install --from-path ~/workspace/src
	
## Today we learned:
We installed teb_local_planner and the navigation stack. There are some similar issues running on the racecar to running it locally, but we probably debugged most of them. It's no longer complaining about the tf transform, but is instead complaining about the publishing rate of the control loop...something like

[WARM] blah blah Control loop missed its desired rate of 20.0000Hz.... the loop actually took 0.1968 seconds 

It's probably a configuration file in navigation somewhere, but we couldn't find it and are currently pretty tired. We tried messing with files in teb_local_planner/cfg, with all the yamls, particularly local_costmap_param.yaml and global_costmap_params.yaml. It shouldn't matter, but the warning are very frequent.

We are not sure where to go from here, since move_base is not publishing stuff, but we are able to run amcl and the nav_stack, without any errors. The dummy nodes have the outputs we expect from them, but we are unsure as to how to test where something is going wrong.

We are not specifically running teb_local_planner currently, just the vanilla move_base. To add in teb, amend the launch file, as shown here: 

http://wiki.ros.org/action/fullsearch/teb_local_planner/Tutorials/Configure%20and%20run%20Robot%20Navigation?action=fullsearch&context=180&value=linkto%3A%22teb_local_planner%2FTutorials%2FConfigure+and+run+Robot+Navigation%22

The launch file on the racecar is different than the one in the repo.

To run what we ran:

roslaunch teb_local_planner my_move_base.launch

this launch file contains the map_server, amcl (particle filter) parameters, a bag file (which is currently commented out), two dummy nodes for running move_base (testnode.py provides a goal) and an initial_pose for amcl.

roslaunch launcher racecar.launch

rosrun teb_local_planner cmd_vel_to_ackermann_drive
(this runs the conversion of the output velocity of the nav stack to the final ackermann comamnds. obvs this node isn't getting anything useful, but we can add it to the launch file later)

One thing that might help with debugging is this GUI:

rosrun rqt_reconfigure rqt_reconfigure

You can dynamically change parameters with this.

Don't forget to source.
====
4/4/16 1700 Update:

- Base NavStack running with map input from gmapper. Cost maps are dynamically and correctly generated for both global and local.
- Not yet generating navigation plan (need to hook in global and local planners correctly)
- Sensor data is all now conditioned correctly
- RVIZ config file created in /_cfg/ called "nav_stack" which visualizes all of this.
 
To run current stuff:
```
	1) roslaunch launcher racecar.launch #Launch the hardware
	2) roslaunch racecar_nav gmapper.launch #start generating SLAM map
	3) roslaunch racecar_nav nav_stack.launch #run the navigation stack
```

