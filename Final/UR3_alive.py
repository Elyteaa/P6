#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import numpy as np
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float32MultiArray

#A global variable for the robot's position
Q0 = None

#Class for the transformation actions
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
      transformationMatrix()
      res = np.dot(self.T,np.reshape(np.array([x, y, z, 1]), (4,1)))
      return res

def callBack (data):
    global Q0
    position = data.data
    Q0 = [position[0], position[1], position[2], position[3], position[4], position[5]]

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("position", Float32MultiArray, callBack)

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

#Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
#Q3 = [1.5,-0.2,-1.57,0,0,0]

client = None

def move1():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        for i in range(10):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            """
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
            """
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
        client.send_goal(g)
        time.sleep(3.0)
        print "Interrupting"
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise
   
def main():
    global client
    
    frame = transformation()
    #Print the transformation matrix
    print(frame.T)
    
    #Just a trial
    res = frame.inRobotFrame(-0.390, 0.21, 1.277)   
    print(res)
    
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        if (Q0 != None):
            print "This program makes the robot move between the following three poses:"
            #print str([Q1[i]*180./pi for ti in xrange(0,6)])
            #print str([Q2[i]*180./pi for i in xrange(0,6)])
            #print str([Q3[i]*180./pi for i in xrange(0,6)])
            print "Please make sure that your robot can move freely between these poses before proceeding!"
            inp = raw_input("Continue? y/n: ")[0]
            if (inp == 'y'):
                #move1()
                move_repeated()
                #move_disordered()
                #move_interrupt()
            else:
                print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    listener()
    while not rospy.is_shutdown():
        main()
