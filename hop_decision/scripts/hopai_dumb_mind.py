import rospy
import tf
import random
import math

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from hop_msgs.msg import *

##################################################
# The is the simple version of the decision module 
# of the robot using a FSM in python. pytransitions 
# is the package used for FSM implementation. 
# FOr more info:
# https://github.com/pytransitions/transitions
#
# Version: 0.1
# Author: Rui
##################################################
def to_pose_stamped(pos):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, pos[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

def talker():
	time1 = rospy.Time(6)
	time2 = rospy.Time(0.8)
	goal_pose  = [2.25,2,-0.5]
	post_stamp_goal = to_pose_stamped(goal_pose)
    pose_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.init_node('hop_dumb_decision', anonymous=True)
	pose_pub.publish(post_stamp_goal)
	time1.sleep()
	while not rospy.is_shutdown():
		goal_pose  = [2.25,2,-1]
		post_stamp_goal = to_pose_stamped(goal_pose)
		pose_pub.publish(post_stamp_goal)
		time2.sleep()
		goal_pose  = [2.25,2,0]
		post_stamp_goal = to_pose_stamped(goal_pose)
		pose_pub.publish(post_stamp_goal)
		time2.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    