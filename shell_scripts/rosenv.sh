#!/bin/bash
if [ "$1" == "0" ]; then
    echo "Setting Environment Variables for Local..."
    host=$(hostname)
    export ROS_IP=$ip
    export ROS_MASTER_URI=http://$host:11311
elif [ "$1" == "1" ]; then
    echo "Setting Environment Variables for Remote..."
    ip=$(/sbin/ifconfig eth0 | grep "inet addr" | cut -d ':' -f 2 | cut -d ' ' -f 1)
    export ROS_IP=$ip
    export ROS_MASTER_URI=http://192.168.100.63:11311
else
    echo "Useage: rosenv 0; where 0 = local 1 = remote."
fi

echo "Done!"
