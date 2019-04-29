#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher('position', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        hello_str = Float32MultiArray()
        hello_str.data = [2.2,0,-1.57,0,0,0]
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass