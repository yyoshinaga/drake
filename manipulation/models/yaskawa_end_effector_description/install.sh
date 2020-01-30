#!/bin/sh
#copy all necessary resource files to fixed resource path
sudo mkdir -p /opt/dorabot/resources/end_effectors/conveyor_belt_dof1
sudo cp end_effector.xacro /opt/dorabot/resources/end_effectors/conveyor_belt_dof1/
sudo cp -r mesh /opt/dorabot/resources/end_effectors/conveyor_belt_dof1/