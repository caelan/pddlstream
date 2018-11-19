#!/bin/bash

# start with a xacro, generate urdf and finally dae
rosrun xacro xacro --inorder -o $1.urdf $1.xacro
rosrun collada_urdf urdf_to_collada $1.urdf $1.dae
openrave-robot.py $1.dae --info links

# to execute this shell script, run:
# chmod a+x xacro2dae.sh
# ./xacro2dae.sh
