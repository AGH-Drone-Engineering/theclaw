#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def main():
    rospy.init_node('control_node')

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=4)

    def on_joy(data: Joy):
        rospy.logdebug(rospy.get_caller_id() + f"[joy] {data.axes}")
        msg = Twist()
        msg.linear.x = data.axes[1]
        msg.linear.y = data.axes[0]
        msg.angular.z = data.axes[2]
        cmd_vel_pub.publish(msg)

    rospy.Subscriber("joy", Joy, on_joy, queue_size=8)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
