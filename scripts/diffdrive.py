#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16


def clamp_byte(x):
    return max(-255, min(int(x), 255))


def main():
    rospy.init_node('diffdrive')

    left_float_pub = rospy.Publisher('left/command/float', Float64, queue_size=1)
    left_int_pub = rospy.Publisher('left/command/int', Int16, queue_size=1)

    right_float_pub = rospy.Publisher('right/command/float', Float64, queue_size=1)
    right_int_pub = rospy.Publisher('right/command/int', Int16, queue_size=1)

    def on_twist(data: Twist):
        left_int = Int16()
        left_float = Float64()
        right_int = Int16()
        right_float = Float64()

        fwd = data.linear.x
        turn = data.angular.z

        xl = fwd - turn
        xr = fwd + turn

        left_float.data = xl
        left_int.data = clamp_byte(xl * 255)

        right_float.data = xr
        right_int.data = clamp_byte(xr * 255)

        left_float_pub.publish(left_float)
        left_int_pub.publish(left_int)

        right_float_pub.publish(right_float)
        right_int_pub.publish(right_int)

    rospy.Subscriber("cmd_vel", Twist, on_twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
