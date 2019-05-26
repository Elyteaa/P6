#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

robotPose = geometry_msgs.msg.Pose()

def callback(data):
	global robotPose
	robotPose = data

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