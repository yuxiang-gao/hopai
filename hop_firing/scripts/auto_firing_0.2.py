import rospy
from std_msgs.msg import UInt8MultiArray
from hop_msgs.msg import *
from collections import Counter
import tf
import math

class GimbalAiming:
    cur_gimbal = GimbalAngle()
    goal_gimbal = EnemyPose()
    _gimbal_pub    = rospy.Publisher('enemy_pose',EnemyPose,queue_size=5)
    _shoot_pub     = rospy.Publisher('shoot', UInt8MultiArray, queue_size=1)

	def __init__(self):
        self.goal_gimbal.enemy_dist  = 3
        self.goal_gimbal.enemy_pitch = 0
        self.goal_gimbal.enemy_yaw   = 0
        self.cur_gimbal.yaw          = 0
        self.cur_gimbal.pitch        = 0
        self.shoot_msg = UInt8MultiArray(data=[0, 0, 1, 1500])

		rospy.init_node('gimbal_aiming', anonymous=True)
        
        rospy.Subscriber('gimbal',GimbalAngle, self._gimbal_callback)
        rospy.Subscriber('target_pose',EnemyPose, self._transform_callback)
        _gimbal_pub.publish(goal_gimbal)
        _shoot_pub.publish(shoot_msg)
        #rate 				= rospy.Rate(30) # 30hz
                


        #rate.sleep()

    def _gimbal_callback(self,msg):
        self.cur_gimbal.enemy_yaw   = msg.enemy_yaw
        self.cur_gimbal.enemy_pitch = msg.enemy_pitch

    def _transform_callback(self,msg):
        self.goal_gimbal.enemy_dist = msg.enemy_dist
        self.goal_gimbal.enemy_yaw  = msg.enemy_yaw + self.cur_gimbal.yaw
        self.goal_gimbal.enemy_pitch= msg.enemy_pitch + self.cur_gimbal.pitch
        self.shoot_msg.data = [1, 0, 1, 1500]



if __name__ == '__main__':
    try:
        GimbalAiming()
    except rospy.ROSInterruptException:
        pass













