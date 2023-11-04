#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def main():
    rospy.init_node('pose2odom')

    pub = rospy.Publisher('/scan_matcher/odom', Odometry, queue_size=32)

    def on_sub(data: PoseStamped):
        msg = Odometry()
        msg.header = data.header
        msg.child_frame_id = "base_link"
        msg.pose.pose = data.pose
        pub.publish(msg)

    rospy.Subscriber("/scan_matcher/pose", PoseStamped, on_sub, queue_size=32)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
