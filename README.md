# Sawyer-Noetic
Docker setup to use Sawyer robot with ros noetic and moveit

## Quick Start Instructions

1. In the root folder of your cloned repository build and run the image by running:
  ```
  $ make run-nvidia-sawyer-robot
  ```
This should open an rviz window with moveit running and a model of Sawyer

2. Open another terminal and run the following command to access the bash shell inside the container:
  ```
  $ docker exec -it sawyer-robot bash
  ```

3. After entering the shell, run the following commands:
  ```
  $ source /opt/ros/noetic/setup.bash
  $ source /root/ros_ws/devel/setup.bash
  $ cd ~/ros_ws
  $ ./intera.sh
  ```
Now you can access all the functionalities of ros-noetic, moveit and intera_interface in this shell.

4. To use your custom python code on Sawyer robot, put your code in `docker/sawyer-robot/docker_code/` folder. There is a mount to this folder inside the container at `/root/code/` and any files can be accessed there

5. To exit the container, type `docker stop sawyer-robot` in a new terminal.


## Utility Code

There is some utility code available in `docker/sawyer-robot/docker_code/`. The following can be run after navigatiing to `/root/code/` inside the docker containder shell,
- Run `python3 init_pos.moveit.py` to set Sawyer in its initial position
- Run `python3 keyboard_pose_control.py` for collision-aware end-effector keyboard control in cartesian coordinates. For joint space keyboard control, intera also has a utility which can be run by `rosrun intera_examples joint_position_keyboard.py` although this is not collision-aware.
- Run `python3 camera_show.py` to access the camera input

