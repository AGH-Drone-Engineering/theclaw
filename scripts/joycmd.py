#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


def threshold(x, low=0.05):
    return x if abs(x) >= low else 0


def main():
    rospy.init_node('joycmd')

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    gripper_left_pub = rospy.Publisher('gripper_arm_left_controller/command', Float64, queue_size=1)
    gripper_right_pub = rospy.Publisher('gripper_arm_right_controller/command', Float64, queue_size=1)
    last_msg = Twist()

    def on_joy(data: Joy):
        rospy.logdebug(rospy.get_caller_id() + f"[joy] {data.axes}")
        msg = Twist()
        msg.linear.x = threshold(data.axes[1]) * 0.5
        msg.angular.z = threshold(data.axes[3]) * 2.0
        cmd_vel_pub.publish(msg)
        nonlocal last_msg
        last_msg = msg

        msg_left = Float64()
        msg_right = Float64()
        gripper_trigger = data.axes[5]
        msg_left.data = msg_right.data = gripper_trigger + 0.5
        gripper_left_pub.publish(msg_left)
        gripper_right_pub.publish(msg_right)

    rospy.Subscriber("joy", Joy, on_joy, queue_size=64)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if abs(last_msg.linear.x) > 0 or abs(last_msg.angular.z) > 0:
            cmd_vel_pub.publish(last_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
