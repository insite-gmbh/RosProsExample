# RosProsExample

This repository contains a sample PROS application for a VEX V5 that shows how to implement ROS topics in a PROS application.
The application subscribes to the "/cmd_vel" topic and publishes
a topic called "chatter". The communication between the VEX V5 and the ROS master is done via a USB-To-Serial connection between the host and the V5. To bring this example to life, you'll need 

* a PROS development environment
* a ROS environment
* a VEX V5 control unit
* two VEX motors 
* and the USB cable to connect the host computer with the V5.

A pre-installed PROS and ROS environment is supplied as a docker image. 

Everything described here was developed and tested with a host computer running Ubuntu 18.04 and Docker 18.09.7.

## Running the docker image
To run the docker container on your host, open a terminal and enter the following command:

`docker run -it -v /dev:/dev --privileged --name ros-pros insitegmbh/ros-pros:latest`

## Uploading the example app to the V5
**This step will upload the example app to the first program slot in the V5!**

Connect the VEX V5 to the host computer. The serial connection should show up as `/dev/ttyACM0` and `/dev/ttyACM1`. 

In the (bash-) shell of the running container enter the following commands
```
cd ~ && cd pros-projects/RosProsExample && prosv5 make all && prosv5 upload
```

This will compile the example code and upload it to the V5.

If you later make any changes to the example project, you could save some typing by entering

`prosv5 mu`

## Starting the ROS master

````
cd ~
source ros-vex-workspace/devel/setup.bash
roslaunch rosserial_vex_v5 hello_world.launch
````

You should see the data of the chatter topic from the V5.

## Start a publisher for the topic /cmd_vel
Open a further terminal on the running container by entering:

`docker exec -it ros-pros /bin/bash`

Again enter these commands:

````
cd ~
source ros-vex-workspace/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
````

This will start a python script that displays a keyboard layout to create twist messages. If you press the key "U" for example the motors of the VEX are starting to turn.

## Change serial port and/or increase the baud-rate
If your system uses a different /dev/tty* to talk to the V5, you can change this in the launch file. Furthermore, initially rosserial uses a baudrate of 57600. Our experience is, that the system can be run at 115200 without problems. 
To modify the launch file, enter the command

`nano ~/ros-vex-workspace/src/rosserial/rosserial_vex_v5/launch/minimal_robot.launch'

