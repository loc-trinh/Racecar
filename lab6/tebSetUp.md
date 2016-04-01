##Installing 
	Run:
		sudo apt-get install ros-indigo-teb-local-planner
		catkin_make
		rosdep install teb_local_planner  to install dependences
	install the navigation stack:
		git clone https://github.com/ros-planning/navigation/tree/indigo-devel.git
		catkin_make
		rosdep install --from-path ~/workspace/src
	


