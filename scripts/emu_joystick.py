#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from crosbot_msgs.msg import ControlCommand
import os
import tf

from time import sleep
from espeak import espeak
from math import pi, sqrt

BUTTON_A = 1
BUTTON_B = 2
BUTTON_Y = 3
BUTTON_X = 0
BUTTON_LT = 6
BUTTON_LB = 4
BUTTON_RT = 7
BUTTON_RB = 5
BUTTON_START = 9
BUTTON_BACK = 8
BUTTON_LEFT_STICK = 10
BUTTON_RIGHT_STICK = 11

AXES_DPAD_LR = 0
AXES_DPAD_TB = 1
AXES_RIGHT_STICK_LR = 2
AXES_RIGHT_STICK_TB = 3
AXES_LEFT_STICK_LR = 4
AXES_LEFT_STICK_TB = 5

class JoystickNode:
	def __init__(self):
		self.motors_stopped = False
		self.flipper_stopped = False
		self.dying = False
		self.max_speed = rospy.get_param('~speed', 1.0)
		self.max_turn = rospy.get_param('~turn', 0.75)
		self.joint_states = {}
		self.twistPub = rospy.Publisher("/base/cmd_vel", Twist, queue_size=1)
		self.armPub = rospy.Publisher("/joint_control", JointState, queue_size=1)
		self.crosbotCommandPub = rospy.Publisher("/crosbot/commands", ControlCommand, queue_size=1)
		self.targetPoseArrayPub = rospy.Publisher("/targetPoseArray", PoseArray, queue_size=1)
		rospy.Subscriber("joy", Joy, self.callback)
		rospy.Subscriber("joint_states", JointState, self.callbackJoints)
		self.telemListener = tf.TransformListener()

		# Joint controls
		self.have_joint_states = False
		self.ideal_arm_angle = -0.785
		self.shoulder_offset = 0
		self.neck_tilt_offset = 0
		
		# Stuff for PoseArray
		self.dpadReleased = True
		self.targetPoseArrayLength = 0
		self.targetPoseArray = PoseArray()
		self.targetPoseArray.header.stamp = rospy.get_rostime()
		self.targetPoseArray.header.frame_id = "/icp_test"

		# Speak when initialised
		espeak.set_voice("en")
		self.sayText("Emu ready for teleop control")

	def sayText(self, text):
		#espeak.synth(text)
		print text

	def setVelocities(self, forward, turn):
		cmd = Twist()
		cmd.linear.x = forward * self.max_speed;
		cmd.linear.y = cmd.linear.z = 0;
		cmd.angular.z = turn * self.max_turn;
		cmd.angular.y = cmd.angular.x = 0;
		self.twistPub.publish(cmd)
		
	def setNavGoal(self, forward, right):
		goal = Pose()
		if forward == 0 or right == 0:
			dist = 3.5
		else:
			dist = 3.5/sqrt(2)
		goal.position.x = forward * dist
		goal.position.y = right * dist
		goal.position.z = 0
		if forward == 0:
			ang = right * pi/2.0
		elif right == 0:
			ang = (forward - 1)/2 * pi
		else:
			if forward == 1:
				ang = right * pi/4.0
			else:
				ang = right * pi*3.0/4.0				
		q =  tf.transformations.quaternion_from_euler(0, 0, ang)
		goal.orientation.x = q[0]
		goal.orientation.y = q[1]
		goal.orientation.z = q[2]
		goal.orientation.w = q[3]
		if self.targetPoseArrayLength > 1:
			goal.position.x += self.targetPoseArray.poses[self.targetPoseArrayLength - 2].position.x
			goal.position.y += self.targetPoseArray.poses[self.targetPoseArrayLength - 2].position.y
			
		if len(self.targetPoseArray.poses) < self.targetPoseArrayLength:
			self.targetPoseArray.poses.append(goal)
		else:
			self.targetPoseArray.poses[self.targetPoseArrayLength - 1] = goal

		print "targetPoseArray is:\n",
		for p in self.targetPoseArray.poses:
			print "\t(%f, %f)\n" % (p.position.x, p.position.y),		

		tfPoseArray = PoseArray()
		tfPoseArray.header = self.targetPoseArray.header
		
		for p in self.targetPoseArray.poses:		
			try:
				ps = PoseStamped()
				ps.header.stamp = rospy.Time(0)
				ps.header.frame_id = "base_link"
				ps.pose = p
				(trans,rot) = self.telemListener.lookupTransform('/icp_test', '/base_link', rospy.Time(0))
				tfGoal = self.telemListener.transformPose("/icp_test", ps)
				tfPoseArray.poses.append(tfGoal.pose)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				print e
		print "\ntfPoseArray is:\n",
		for p in tfPoseArray.poses:
			print "\t(%f, %f)\n" % (p.position.x, p.position.y),	
		self.targetPoseArrayPub.publish(tfPoseArray)

	def callbackJoints(self, joint_data):
		for i in range(len(joint_data.name)):
			self.joint_states[joint_data.name[i]] = joint_data.position[i]

		# If first time reading joints - set ideal_arm_angle
		if (not self.have_joint_states):
		    self.ideal_arm_angle = self.joint_states["arm_shoulder"]
		self.have_joint_states = True

	def callback(self, data):
		if self.dying:
			return
		if not self.have_joint_states:
			return
		#rospy.loginfo(rospy.get_caller_id() + " axes %s", data.axes)
		#rospy.loginfo(rospy.get_caller_id() + " buttons %s", data.buttons)

		### Process shutdown/killing of the computer

		# Shutdown the Computer
		if (data.buttons[BUTTON_LB] == 1 and data.buttons[BUTTON_RB] == 1 and data.buttons[BUTTON_RT] == 1 and data.buttons[BUTTON_LT] == 1 and data.buttons[BUTTON_START] == 1 and data.buttons[BUTTON_B] == 1):
			# wireless D/off -> B, LB, RT, start, left stick
			self.sayText("SHUTDOWN EMU")
			sleep(1.0)
			self.dying = True
			self.setVelocities(0,0)
			rospy.logwarn(rospy.get_caller_id() + " Shutting down the machine.")
			os.system("sudo shutdown -P now")
#			os.system("killall -s SIGINT roslaunch")
			return

		# Kill ROS launch file
		elif (data.buttons[BUTTON_LT] == 1 and data.buttons[BUTTON_RT] == 1 and data.buttons[BUTTON_RB] == 0 and data.buttons[BUTTON_LB] == 0 and data.buttons[BUTTON_BACK] == 1 and data.buttons[BUTTON_A] == 1):
			# wireless D/off -> A, RT, back, start
			self.sayText("Killing roslaunch")
			self.dying = True
			self.setVelocities(0,0)
			rospy.logwarn(rospy.get_caller_id() + " Killing roslaunches.")
			os.system("killall -s SIGINT roslaunch")
			return
		
		### The above return if successful

		# Drive/Autonomy Motor Control
		send_crosbot_command = False
		crosbot_command = ""
		if (data.buttons[BUTTON_RT] == 1 and data.buttons[BUTTON_LT] == 0):
			self.motors_stopped = False
			self.setVelocities(data.axes[AXES_RIGHT_STICK_TB]/2.0, data.axes[AXES_RIGHT_STICK_LR]/2.0)
			if (data.axes[AXES_DPAD_TB] != 0 or data.axes[AXES_DPAD_LR] != 0):
				self.sayText("Travelling along PoseArray")
				if self.dpadReleased == True:
					self.targetPoseArrayLength += 1
				self.dpadReleased = False
				self.setNavGoal(data.axes[AXES_DPAD_TB], data.axes[AXES_DPAD_LR])
				send_crosbot_command = True
				crosbot_command = "explore_command_follow_pose_array"
			else:		
				self.dpadReleased = True
				if (data.buttons[BUTTON_RB]):
					send_crosbot_command = True
					self.sayText("Mode set to right wall following")
					crosbot_command = "explore_command_rightwall_follow"
				elif (data.buttons[BUTTON_LB]):
					send_crosbot_command = True
					self.sayText("Mode set to left wall following")
					crosbot_command = "explore_command_leftwall_follow"
				elif (data.buttons[BUTTON_START]):
					self.sayText("Starting exploring")
					send_crosbot_command = True
					crosbot_command = "command_start"
				elif (data.buttons[BUTTON_BACK]):
					self.sayText("Stopping exploring")
					send_crosbot_command = True
					crosbot_command = "command_stop"
				elif (data.buttons[BUTTON_A]):
					self.sayText("Travelling to origin")
					send_crosbot_command = True
					crosbot_command = "explore_command_go_to_origin"
				elif (data.buttons[BUTTON_B]):
					self.sayText("Clearing PoseArray")
					self.targetPoseArrayLength = 0
					self.targetPoseArray.poses = []
					self.targetPoseArrayPub.publish(self.targetPoseArray)
					send_crosbot_command = True
					crosbot_command = "explore_command_clear_pose_array"

		# If no drive commands are being sent - then ensure motors are stopped
		elif (not self.motors_stopped):
			self.setVelocities(0.0, 0.0)
			self.motors_stopped = True

		# If back is pressed for any reason, then emergency stop should be sent
		if (data.buttons[BUTTON_RT] == 0 and data.buttons[BUTTON_BACK] == 1):
			self.sayText("Emergency Stop")
			send_crosbot_command = True
			crosbot_command = "command_EM_STOP"
		
		# Arm movements
		joints = []
		positions = []
		
		angle_change = 0.1
		arm_updated = False
		arm_base_pos = self.joint_states["arm_base"]
		arm_shoulder_pos = self.joint_states["arm_shoulder"]
		arm_elbow_pos = self.joint_states["arm_elbow"]
		neck_tilt_pos = self.joint_states["neck_tilt"]
		neck_pan_pos = self.joint_states["neck_pan"]

		# "Ideal angle" of arm (without any offsets)
		#ideal_arm_angle = arm_shoulder_pos - self.shoulder_offset

		# Zero joints
		if (data.buttons[BUTTON_LT] == 1 and data.buttons[BUTTON_RT] == 0 and data.buttons[BUTTON_BACK] == 0):
		    # Neck straight up
			if (data.axes[5] == 1):
				joints = ["neck_tilt", "neck_pan", "arm_base", "arm_shoulder", "arm_elbow"]
				positions = [ 0.0, 0.0, 0.0, 0.0, 0.0 ]
				self.neck_tilt_offset = 0
				self.shoulder_offset = 0
				self.ideal_arm_angle = 0
			elif (data.axes[5] == -1):
				joints = ["neck_tilt", "neck_pan", "arm_base"]
				positions = [ arm_shoulder_pos, 0.0, 0.0 ]
				self.neck_tilt_offset = 0
			# Neck at right angle
			elif (data.axes[4] == 1 or data.axes[4] == -1):
				joints[:] = []
				positions[:] = []
				joints = ["neck_tilt", "neck_pan", "arm_base", "arm_shoulder", "arm_elbow"]
				positions = [ -0.785, 0.0, 0.0, -0.785, 1.57 ]	
				self.neck_tilt_offset = 0
				self.shoulder_offset = 0
				self.ideal_arm_angle = -0.785
			# Neck collapsed 
			elif (data.axes[5] == -1):
				joints[:] = []
				positions[:] = []
				joints = ["neck_tilt", "neck_pan", "arm_base", "arm_shoulder", "arm_elbow"]
				positions = [ -1.25, 0.0, 0.0, -1.25, 2.5 ]
				self.neck_tilt_offset = 0
				self.shoulder_offset = 0
				self.ideal_arm_angle = -1.25
			# Head level
			elif (data.buttons[BUTTON_LB] == 1):
				joints[:] = []
				positions[:] = []
				joints = ["neck_tilt", "neck_pan"]
				positions = [ arm_shoulder_pos, 0.0 ]
				self.neck_tilt_offset = 0
			# Shoulder rotated forward
			elif (data.buttons[BUTTON_RB] == 1):
				joints[:] = []
				positions[:] = []
				joints = ["arm_base"]
				positions = [ 0.0 ]

		if (data.buttons[BUTTON_LT] == 0 and data.buttons[BUTTON_RT] == 0 and data.buttons[BUTTON_BACK] == 0):
			## arm_shoulder
			if (data.buttons[BUTTON_A] == 1 and data.buttons[BUTTON_Y] == 1):
				# Do nothing they cancel out
				pass
			elif (data.buttons[BUTTON_A] == 1):
				arm_updated = True
				self.ideal_arm_angle = self.ideal_arm_angle - angle_change
				#arm_shoulder_pos = arm_shoulder_pos - angle_change
			elif (data.buttons[BUTTON_Y] == 1):
				arm_updated = True
				self.ideal_arm_angle = self.ideal_arm_angle + angle_change
				#arm_shoulder_pos = arm_shoulder_pos + angle_change
			## arm_elbow
			if (data.buttons[BUTTON_X] == 1 and data.buttons[BUTTON_B] == 1):
				# Do nothing they cancel out
				pass
			elif (data.buttons[BUTTON_X] == 1):
				arm_updated = True
				arm_base_pos = arm_base_pos + angle_change
			elif (data.buttons[BUTTON_B] == 1):
				arm_updated = True
				arm_base_pos = arm_base_pos - angle_change
			## Shoulder forward-back
			if (data.buttons[BUTTON_LB] == 1 and data.buttons[BUTTON_RB] == 1):
				# Do nothing they cancel out
				pass
			elif (data.buttons[BUTTON_LB] == 1):
				arm_updated = True
				self.shoulder_offset = self.shoulder_offset - angle_change
			elif (data.buttons[BUTTON_RB] == 1):
				arm_updated = True
				self.shoulder_offset = self.shoulder_offset + angle_change
			## D-pad PTZ control
			if (data.axes[AXES_DPAD_LR] < 0):
				arm_updated = True
				neck_pan_pos = neck_pan_pos - angle_change
				#joints.append("neck_pan")
				#positions.append(self.joint_states["neck_pan"] - angle_change)
			if (data.axes[AXES_DPAD_LR] > 0):
				arm_updated = True
				neck_pan_pos = neck_pan_pos + angle_change
				#joints.append("neck_pan")
				#positions.append(self.joint_states["neck_pan"] + angle_change)
			if (data.axes[AXES_DPAD_TB] < 0):
				arm_updated = True
				self.neck_tilt_offset = self.neck_tilt_offset - angle_change
				#neck_tilt_offset = neck_tilt_offset - angle_change
				#joints.append("neck_tilt")
				#positions.append(self.joint_states["neck_tilt"] - angle_change)
			if (data.axes[AXES_DPAD_TB] > 0):
				arm_updated = True
				self.neck_tilt_offset = self.neck_tilt_offset + angle_change
				#neck_tilt_offset = neck_tilt_offset + angle_change
				#joints.append("neck_tilt")
				#positions.append(self.joint_states["neck_tilt"] + angle_change)
				

		# Publish crosbot command
		if (send_crosbot_command):
			crosbotCmd = ControlCommand()
			crosbotCmd.header.stamp = rospy.get_rostime()
			crosbotCmd.command = crosbot_command
			self.crosbotCommandPub.publish(crosbotCmd)

		# If joint updated - then calculated rest of arm joints
		if (arm_updated):
			# calculate new values
			self.ideal_arm_angle = min(self.ideal_arm_angle, 1.25)
			self.ideal_arm_angle = max(self.ideal_arm_angle, -1.25)

			arm_base_pos = min(arm_base_pos, 1.0)
			arm_base_pos = max(arm_base_pos, -1.0)

			arm_shoulder_pos = self.ideal_arm_angle + self.shoulder_offset
			arm_shoulder_pos = min(arm_shoulder_pos, 1.25)
			arm_shoulder_pos = max(arm_shoulder_pos, -1.25)

			arm_elbow_pos = self.ideal_arm_angle * -2
			arm_elbow_pos = min(arm_elbow_pos, 3.0)
			arm_elbow_pos = max(arm_elbow_pos, 0)

			neck_tilt_pos = self.ideal_arm_angle - self.shoulder_offset + self.neck_tilt_offset
			neck_tilt_pos = min(neck_tilt_pos, 1.25)
			neck_tilt_pos = max(neck_tilt_pos, -1.25)

			neck_pan_pos = min(neck_pan_pos, 1.25)
			neck_pan_pos = max(neck_pan_pos, -1.25)

			joints.append("arm_base")
			positions.append(arm_base_pos)
			joints.append("arm_shoulder")
			positions.append(arm_shoulder_pos)
			joints.append("arm_elbow")
			positions.append(arm_elbow_pos)
			joints.append("neck_tilt")
			positions.append(neck_tilt_pos)
			joints.append("neck_pan")
			positions.append(neck_pan_pos)

		# Publish points
		if (len(joints) > 0):
			cmd = JointState()
			cmd.header.stamp = rospy.get_rostime()
			cmd.name = joints
			cmd.position = positions
			self.armPub.publish(cmd)
					

if __name__ == '__main__':

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('emu_joystick', anonymous=False)

	node = JoystickNode()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
