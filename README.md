# RosProsExample
docker run -it -v /dev:/dev --privileged --name ros-pros insitegmbh/ros-pros:latest

cd ~

cd pros-projects/RosProsExample

prosv5 make all

prosv5 upload

(later: prosv5 mu)

cd ~

source ros-vex-workspace/devel/setup.bash

roslaunch rosserial_vex_v5 hello_world.launch


new terminal:

docker exec -it ros-pros /bin/bash

rosrun teleop_twist_keyboard teleop_twist_keyboard.py


