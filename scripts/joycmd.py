#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def threshold(x, low=0.1):
    return x if abs(x) >= low else 0


def main():
    rospy.init_node('joycmd')

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def on_joy(data: Joy):
        rospy.logdebug(rospy.get_caller_id() + f"[joy] {data.axes}")
        msg = Twist()
        msg.linear.x = threshold(data.axes[1])
        msg.linear.y = threshold(data.axes[0])
        msg.angular.z = threshold(data.axes[3])
        cmd_vel_pub.publish(msg)

    rospy.Subscriber("joy", Joy, on_joy, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
