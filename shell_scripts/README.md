RSS-Team9 Convenience Scripts
===============
Introduction:
----------------
This folder contains useful scripts that make working with git, the racecar, and ros easier.

Installation:
----------------
```
./INSTALLME.sh
```

Usage:
----------------
Installation will add the following bash scripts to your environment:
```
cdws - cd to the racecar source folder of the catkin workspace
sshcar - ssh to the racecar
rosenv - print out ros environment variables
gs - git status
gpush $1 - Full commit from racecar repository root directory, where first argument is commit message
gpull - pull current branch from racecar repository root directory
gmb $1 - make branch where branch name is first arguement. Branch is pushed to origin and upstream is set.
roslocal - set ros environments for local/simulator usage
rosremote - set ros environment for roscore on physical racecar
```
