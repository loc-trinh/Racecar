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
We installed all the junk on the racecar. There are some similar issues to running it locally, but we probably debugged most of them. It's no longer complaining about the tf transform, but is instead complaining about the publishing rate of the control loop...something like

[WARM] blah blah Control loop missed its desired rate of 20.0000Hz.... the loop actually took 0.1968 seconds 

It's probably a configuration file in navigation somewhere, but we couldn't find it and are currently pretty tired. We tried messing with files in teb_local_planner/cfg, with all the yamls, particularly local_costmap_param.yaml and global_costmap_params.yaml

Because of this, move_base is not publishing stuff, but we are able to run amcl and the nav_stack, getting them to all play nice together.

The launch file on the racecar is different than the one in the repo. Planning on committing it soonish.

Racecar is semi-charged.

We were running:
roslaunch teb_local_planner my_move_base.launch
roslaunch launcher racecar.launch or racecar_teleop.launch. Not entirely sure if it makes a difference here
rosrun teb_local_planner cmd_vel_to_ackermann_drive - this runs the conversion of the output velocity of the nav stack to the final ackermann comamnds. obvs this node isn't getting anything useful, but we can add it to the launch file later 
inside my_move_base, you'll find a map_server, amcl (particle filter), and the normal launch stuff for teb, a bag file (which we mostly deleted), and two dummy nodes that produce a goal for move_base and an initial pose for amcl (which is mostly zero and silly)

Don't forget to source.
