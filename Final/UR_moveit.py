#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import roslib; roslib.load_manifest('ur_driver')
import actionlib
import numpy as np
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import String

robotPose = geometry_msgs.msg.Pose()

def callback(data):
	global robotPose
	robotPose = data

class transformation:
	
	def __init__(self):
		self.T = None
		self.transformationMatrix()
		
	#The function initializes the transformation matrix
	def transformationMatrix(self):
		#Some points in the robot frame
		Rx = np.array([0.386, 0.265, 0.681, 0.560, 0.601, 0.488])
		Ry = np.array([0.485, 0.491, 0.083, 0.085, -0.095, -0.138])
		Rz = np.array([0.132, 0.138, 0.006, 0.010, 0.109, 0.114])
		
		#Some points in the camera frame
		Cx = np.array([-0.390, -0.522, -0.076, -0.205, -0.144, -0.259])
		Cy = np.array([0.210, 0.203, 0.284, 0.281, 0.155, 0.141])
		Cz = np.array([1.277, 1.272, 0.880, 0.877, 0.723, 0.669])
		
		#A matrix for the camera frame, since we want to transform from camera frame to robot frame
		A = np.matrix([[np.sum(np.square(Cx)), np.sum(np.multiply(Cx,Cy)), np.sum(np.multiply(Cx,Cz)), np.sum(Cx)],
		  [np.sum(np.multiply(Cx,Cy)), np.sum(np.square(Cy)), np.sum(np.multiply(Cy,Cz)), np.sum(Cy)],
		  [np.sum(np.multiply(Cx,Cz)), np.sum(np.multiply(Cy,Cz)), np.sum(np.square(Cz)), np.sum(Cz)],
		  [np.sum(Cx), np.sum(Cy), np.sum(Cz), 6]])
	
		#Intermediate vectors for the final transfomration matrix betweem the frames	
		V1 = np.array([np.sum(np.multiply(Rx,Cx)), np.sum(np.multiply(Rx,Cy)), np.sum(np.multiply(Rx,Cz)), np.sum(Rx)])
		V2 = np.array([np.sum(np.multiply(Ry,Cx)), np.sum(np.multiply(Ry,Cy)), np.sum(np.multiply(Ry,Cz)), np.sum(Ry)])
		V3 = np.array([np.sum(np.multiply(Rz,Cx)), np.sum(np.multiply(Rz,Cy)), np.sum(np.multiply(Rz,Cz)), np.sum(Rz)])
			  
		rxx = np.dot(np.linalg.inv(A),np.reshape(V1, (4,1)))
		ryy = np.dot(np.linalg.inv(A),np.reshape(V2, (4,1)))
		rzz = np.dot(np.linalg.inv(A),np.reshape(V3, (4,1)))
		
		#The transformation matrix
		self.T = np.matrix([[rxx.item(0), rxx.item(1), rxx.item(2), rxx.item(3)],
							[ryy.item(0), ryy.item(1), ryy.item(2), ryy.item(3)],
							[rzz.item(0), rzz.item(1), rzz.item(2), rzz.item(3)],
							[0, 0, 0, 1]])
		
		#print(np.dot(T,np.reshape(np.array([Cx.item(0), Cy.item(0), Cz.item(0), 1]), (4,1))))
			  
	#The function calculates coordinates from camera frame to robot frame
	def inRobotFrame(self, x, y, z):
		#The function receives coordinates in the camera frame and results in coordinates in the robot frame
		self.transformationMatrix()
		res = np.dot(self.T,np.reshape(np.array([x, y, z, 1]), (4,1)))
		return res


def move_group_python_interface_tutorial():
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("ur3")
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
	rospy.Subscriber("robotPose", Pose, callback)

	#This sleep is ONLY to allow Rviz to come up.
	print "============ Waiting for RVIZ..."
	rospy.sleep(10)
	print "============ Starting tutorial "

	"""
	We can get the name of the reference frame for this robot

	print "============ Reference frame: %s" % group.get_planning_frame()
	We can also print the name of the end-effector link for this group

	print "============ Reference frame: %s" % group.get_end_effector_link()
	We can get a list of all the groups in the robot

	print "============ Robot Groups:"
	print robot.get_group_names()
	Sometimes for debugging it is useful to print the entire state of the robot.

	print "============ Printing robot state"
	print robot.get_current_state()
	print "============"
	"""

	#Planning a motion for this group to a desired pose for the end-effector
	print "============ Generating plan 1"
	pose_target = geometry_msgs.msg.Pose()

	"""pose_target.position.x = robotPose[0]
	pose_target.position.y = robotPose[0]
	pose_target.position.z = robotPose[0]
	pose_target.orientation.w = robotPose[0]
	pose_target.orientation.w = robotPose[0]
	pose_target.orientation.w = robotPose[0]
	pose_target.orientation.w = robotPose[0]"""

	pose_target = robotPose

	frame = transformation()
	res = frame.inRobotFrame(pose_target.position.x, pose_target.position.y, pose_target.position.z)

	pose_target.position.x = res.item(0)
	pose_target.position.y = res.item(1)
	pose_target.position.z = res.item(2)

	group.set_pose_target(pose_target)

	#Calling the planner to compute the plan and visualize it if successful. Note: this does not ask move_group to actually move the robot
	plan1 = group.plan()
	print "============ Waiting while RVIZ displays plan1..."
	rospy.sleep(5)

	#A blocking funtion, to move the actual robot. Requires a controller to be active and report success on execution of a trajectory
	group.go(wait=True)

	rospy.spin()
	moveit_commander.roscpp_shutdown()
	print "============ STOPPING"

if __name__=='__main__':
	try:
		move_group_python_interface_tutorial()
	except rospy.ROSInterruptException:
		pass