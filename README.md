# TRACKBOT

- author: Julian Rene Leclerc 
- course: Robotics Project, University of Tartu
- Year: Spring Semester 2024

## Description

Trackbot is a robot developed by Julian Leclerc at the University of tartu for the Robotics Project cource.
The constraints of the project were to develop a robot within a small box using a unique movement mechanism 
able to travese 1m in less than 2min and have a sense of the environment (through sensors) 

<br>The solution, Trackbot, is a small compact robot able to move through terrain using a suspenssion track mechanism. 
The movement is provided through 2 motors linked to wheels in suspenssion providing more adaptibility to the terrrain. 
The robot is controlled by a Raspberry Pi 3B+ running ROS noetic on a server version of ubuntu. 
To sense the environment the robot is equiped with a ultrasonic sensor and a camera in front.

## REP Details

- Code dir: Trackbot Package to include in ROS environment to run robot, includes .py nodes in trackbot/scripts/
- Parts dir: all the obj parts necessary for the project and assembly
- Sprint_Presentations dir: Presentations showed during sprint meetings displaying evolution and challenges of project
- BOM.xlsx: The list of components needed to build the robot
- Trackbot_Info.pdf: Guide for the Trackbot Package and nodes


## Set up Raspberry

- flash raspberry with ubuntu 20.04 LTS
- install ROS noetic and necessary packages
- setup the Trackbot package within the catkin/src rep

## Launch nodes

- setup .bashrc and source
- launch roscore
- source devel/setup.bash and desired launch node
