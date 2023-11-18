import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math


class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move base server")

    def goToWaypoint(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(theta/2)
        goal.target_pose.pose.orientation.w = math.cos(theta/2)

        rospy.loginfo("Sending goal location ...")
        self.client.send_goal(goal)
        rospy.loginfo("Waiting for the result ...")
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo("Goal reached!")
            return self.client.get_result()
