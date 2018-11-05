#!/bin/bash

# genereate urdf from xacro
# run chmod +x xacro2urdf.sh first
rosrun xacro xacro --inorder -o $2.urdf $1.xacro
