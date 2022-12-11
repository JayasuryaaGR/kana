#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('rec', Int32, queue_size=10)
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    num = Int32()
    while not rospy.is_shutdown():
        num.data+=1
        rospy.loginfo(num)
        pub.publish(num)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass