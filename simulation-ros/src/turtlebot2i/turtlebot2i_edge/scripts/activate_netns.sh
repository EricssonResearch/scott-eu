#!/bin/bash

if [ $EUID -ne 0 ]; then
	echo "Run as root"
	exit
fi

if [ $# -lt 1 ]; then
    echo "Usage: $0 nns"
    exit
fi

nns=$1
echo "network namespace: $nns"

# set environment variables for ROS
export ROS_HOSTNAME=$nns
export ROS_MASTER_URI="http://global:11311"

# run shell in network namespace
if [ $nns != "global" ]; then
    ip netns exec $nns bash
else
    bash
fi
