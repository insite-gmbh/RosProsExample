# This is a Dockerfile for ros:melodic-desktop-full and a PROS development environment.
# It is used to setup the VEX V5 as ROS-node that connects
# to a ROS master via a serial connection.

FROM osrf/ros:melodic-desktop-bionic

# install ros packages
RUN apt-get update \
    && apt-get install -y ros-melodic-desktop-full=1.4.1-0* \
    && apt-get install -y ros-melodic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/* 

# download PROS stuff (gcc chain, pip, pros-cli)
 
ADD https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 /root/

RUN cd /root \
    && tar xjf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 \
    && rm gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 \
    && apt update \
    && apt-get install nano \
    && apt install -y python3-venv python3-pip \
    && python3.6 -m pip install --user https://github.com/purduesigbots/pros-cli/releases/download/3.1.4/pros_cli_v5-3.1.4-py3-none-any.whl

# setup paths in bash
RUN echo 'EDITOR='"'"'nano -w'"'" >> /root/.bashrc \
    && echo 'PATH=$PATH:/root/.local/bin' >> /root/.bashrc \
    && echo 'PATH=$PATH:/root/gcc-arm-none-eabi-9-2019-q4-major/bin/' >> /root/.bashrc

# setup ros serial 
RUN ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash && mkdir -p /root/ros-vex-workspace/src && cd /root/ros-vex-workspace/src && git clone https://github.com/ros-drivers/rosserial.git && cd .. && catkin_make && catkin_make install"]

# clone code example
RUN ["/bin/bash", "-c", "mkdir -p /root/pros-projects && cd /root/pros-projects && git clone https://github.com/insite-gmbh/RosProsExample.git"]


CMD ["bin/bash"]

