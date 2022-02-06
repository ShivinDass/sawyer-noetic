ARG PARENT_IMAGE=ubuntu:20.04
FROM $PARENT_IMAGE

# Copy the modified intera script configured for ros-noetic
COPY intera.sh /root/ros_ws/intera.sh

# Install and Build Sawyer Moveit Repo. Instructions obtained from:
# http://sdk.rethinkrobotics.com/intera/MoveIt_Tutorial

RUN ["/bin/bash", "-c", \
  "cd ~/ros_ws/src && \
  wstool merge https://raw.githubusercontent.com/RethinkRobotics/sawyer_moveit/master/sawyer_moveit.rosinstall && \
  wstool update && \
  cd ~/ros_ws/ && \
  source /opt/ros/noetic/setup.bash && \
  catkin build -j6"]

WORKDIR /root/code/
#COPY /examples /root/code/

COPY docker-entrypoint.sh /root/
COPY sawyer-robot.launch /root/
COPY multiple_cam.launch /root/

ENTRYPOINT ["/root/docker-entrypoint.sh"]
