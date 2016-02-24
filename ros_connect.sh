#!/bin/bash
ip=$(/sbin/ifconfig eth0 | grep "inet addr" | cut -d ':' -f 2 | cut -d ' ' -f 1)
export ROS_IP=$ip
export ROS_MASTER_URI=http://192.168.100.63:11311
echo "Done!"