import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import tau

joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node("moveit_python_interface", anonymous=False)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface(synchronous=True)
move_group = moveit_commander.MoveGroupCommander("right_arm")

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "base"
box_pose.pose.position.x = 1.0
box_pose.pose.position.z = -0.015
scene.add_box("table", box_pose, size=(2,1.5,0.05))

# l = input("Enter to cont:")

joint_goal = [-0.1904423828125, -0.921279296875, -0.4195166015625, 1.5382763671875, 0.4234609375, 0.9454833984375, -2.18383984375]
move_group.go(joint_goal, wait=True)
move_group.stop()

# l = input("Enter to cont:")
scene.remove_world_object("table")