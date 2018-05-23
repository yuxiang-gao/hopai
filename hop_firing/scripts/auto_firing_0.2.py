#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Point
from hop_msgs.msg import *
from collections import Counter
import tf
import math

class GimbalAiming:
	goal_gimbal = EnemyPos()
	goal_gimbal.enemy_dist  = 3
	goal_gimbal.enemy_pitch = 0
	goal_gimbal.enemy_yaw   = 0

	def __init__(self):
		rospy.init_node('gimbal_aiming', anonymous=True)
		#self.cur_gimbal = GimbalAngle()
		self._gimbal_pub    = rospy.Publisher('enemy_pos',EnemyPos,queue_size=5)
		self._shoot_pub     = rospy.Publisher('shoot', UInt8MultiArray, queue_size=1)

		#self.cur_gimbal.yaw          = 0.74
		#self.cur_gimbal.pitch        = 0.1
		self.shoot_msg = UInt8MultiArray(data=[0, 0, 1, 54])


		
		rospy.Subscriber('target_point',Point, self._transform_callback)
		
		
		rate 				= rospy.Rate(30) # 30hz
				


		rate.sleep()
		rospy.spin()

	def _transform_callback(self,msg):
		self.goal_gimbal.enemy_dist = 3
		h = msg.z
		alpha = (h-240) /480 * 0.785398
		z = 0.130 * math.cos(alpha) 
		self.goal_gimbal.enemy_yaw  = (0.5-msg.x)  
		self.goal_gimbal.enemy_pitch= math.atan2(z,0.3)
		if h > 0.01:
			self.shoot_msg.data = [1, 0, 1, 54]
			self._gimbal_pub.publish(self.goal_gimbal)
		else:
			self.shoot_msg.data = [0, 0, 1, 54]
		#self._shoot_pub.publish(self.shoot_msg)


if __name__ == '__main__':
	try:
		
		GimbalAiming()
		
	except rospy.ROSInterruptException:
		pass













