import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import intera_external_devices
import intera_interface
from math import tau
import tf
import numpy as np

def addTable(scene):

	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = "base"
	box_pose.pose.position.x = 1.0
	box_pose.pose.position.z = -0.015
	scene.add_box("table", box_pose, size=(2,1.5,0.05))


class KeyboardControl:

	def __init__(self, move_group):
		self.move_group = move_group
		self._limb = intera_interface.Limb('right')
		self._gripper = intera_interface.Gripper()
		self._gripper.calibrate()

		self.position_delta = 0.02
		self.orientation_delta = tau/30
		self.bindings = {
        '1': (self.move_position, [self.move_group, self.position_delta, 0, 0], "x increase"),
        'q': (self.move_position, [self.move_group, -self.position_delta, 0, 0], "x decrease"),
        '2': (self.move_position, [self.move_group, 0, self.position_delta, 0], "y increase"),
        'w': (self.move_position, [self.move_group, 0, -self.position_delta, 0], "y decrease"),
        '3': (self.move_position, [self.move_group, 0, 0, self.position_delta], "z increase"),
        'e': (self.move_position, [self.move_group, 0, 0, -self.position_delta], "z decrease"),
        '4': (self.move_orientation, [self.move_group, self.orientation_delta, 0, 0], "x_rot increase"),
        'r': (self.move_orientation, [self.move_group, -self.orientation_delta, 0, 0], "x_rot decrease"),
        '5': (self.move_orientation, [self.move_group, 0, self.orientation_delta, 0], "y_rot increase"),
        't': (self.move_orientation, [self.move_group, 0, -self.orientation_delta, 0], "y_rot decrease"),
        '6': (self.move_orientation, [self.move_group, 0, 0, self.orientation_delta], "z_rot increase"),
        'y': (self.move_orientation, [self.move_group, 0, 0, -self.orientation_delta], "z_rot decrease"),
        '7': (self.control_gripper, [True], "close gripper"),
        'u': (self.control_gripper, [False], "open gripper")
    }


	def move_position(self, move_group, dx, dy, dz):
		pose = move_group.get_current_pose()
		waypoints = [pose.pose]
		pose.pose.position.x += dx
		pose.pose.position.y += dy
		pose.pose.position.z += dz

		joint_angles = self._limb.ik_request(pose.pose, "right_gripper_tip")
		completed = move_group.go(joint_angles, wait=True)
		move_group.stop()

		return completed

	def move_orientation(self, move_group, roll, pitch, yaw):
		pose = move_group.get_current_pose()
		
		q = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.z])	
		diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		q = tf.transformations.quaternion_multiply(q, diff_q)

		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]

		joint_angles = self._limb.ik_request(pose.pose, "right_gripper_tip")
		completed = move_group.go(joint_angles, wait=True)
		move_group.stop()
		
		return completed

	def control_gripper(self, close_gripper_flag):
		if close_gripper_flag:
			self._gripper.close()
		else:
			self._gripper.open()


joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_python_interface", anonymous=False)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface(synchronous=True)
move_group = moveit_commander.MoveGroupCommander("right_arm")

move_group.set_max_velocity_scaling_factor(0.3)
move_group.set_max_acceleration_scaling_factor(0.3)

addTable(scene)
keyboard = KeyboardControl(move_group)
done = False
while not done and not rospy.is_shutdown():
	c = intera_external_devices.getch()
	if c:
		#catch Esc or ctrl-c
		if c in ['\x1b', '\x03']:
			done = True
		elif c in keyboard.bindings:
			cmd = keyboard.bindings[c]
			if c == '8' or c == 'i' or c == '9':
				cmd[0](cmd[1])
				print("command: %s" % (cmd[2],))
			else:
				#expand binding to something like "set_j(right, 'j0', 0.1)"
				cmd[0](*cmd[1])
				print("command: %s" % (cmd[2],))
		else:
			print("key bindings: ")
			print("  Esc: Quit")
			print("  ?: Help")
			for key, val in sorted(list(keyboard.bindings.items()),key=lambda x: x[1][2]):
				print("  %s: %s" % (key, val[2]))

scene.remove_world_object("table")