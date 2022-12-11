#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32

def control():
    pub = rospy.Publisher('robot_control', Int32, queue_size=10)
    rospy.init_node('robot_control', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    command = Int32()
    while not rospy.is_shutdown():
        command.data = int(input("Enter the number (1: Sanitise; 2: Pick Trash; 3: Mop:\n"))
        rospy.loginfo(command.data)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass