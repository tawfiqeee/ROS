#!/bin/bash

if [ -z "$1" ]; then
	echo "usage: set_heading.sh <heading>"
    exit
fi

Heading=$1

rostopic pub DesiredHeading std_msgs/Float32 $Heading