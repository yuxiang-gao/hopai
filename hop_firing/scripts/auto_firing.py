import rospy
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import UInt8MultiArray
from hop_msgs.msg import *
from collections import Counter
import math

class GimbalAiming:
	def __init__(self):
		rospy.init_node('gimbal_aiming', anonymous=True)
                 self.init_detection()
        rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self._transform_callback)
        rospy.Subscriber('odom',Odometry,self._velocity_callback)
        self._gimbal_pub 	= rospy.Publisher('gimbal_angle', GimbalAngle, queue_size=1)
        self._shoot_pub 	= rospy.Publisher('shoot', UInt8MultiArray, queue_size=1)
        rate 				= rospy.Rate(20) # 20hz
        
        self._cnt 			= 0
        self._out_cnt 		= 0
        self._last_id 		= 0
        self._shoot_dict 	= Counter()

        self.gim_yaw 		= 0
        self.gim_pitch 		= 0        
        self.enable_shoot 	= False

        self.velocity_x		= 0
        self.velocity_y		= 0

        print 'init...'
        self._init_cnt 		= 0
        while not rospy.is_shutdown():
            gimbal_angle 	= GimbalAngle()
            shoot_msg 		= UInt8MultiArray(data=[0, 0, 1, 100])
            
            if self._init_cnt < 50 and self.enable_shoot:
                self._init_cnt += 1
                self._gimbal_pub.publish(gimbal_angle)
                self._shoot_pub.publish(shoot_msg)
                rate.sleep()
                continue
            
            if self.enemy_id == 0:
                self._cnt = 0
                self._out_cnt += 1
                if self._out_cnt > 20
                	self.gim_yaw = 0
                   	self.gim_pitch = 0
            else:
                self._cnt = self._cnt + 1
                if self.enemy_z > 5.0 or self.enemy_z == 0 or self._cnt < 5 or self._shoot_dict[str(self.enemy_id)] > 10:
                    pass
                else:
                    self._out_cnt = 0
                     #self._shoot_dict[str(self.enemy_id)] += 1
                    self.gravity_comp()
                    self._calc_gimbal(self.enemy_x, self.enemy_y, self.enemy_z)
                    shoot_msg.data = [1, 0, 1, 100]
            
            gimbal_angle.pitch = float(self.gim_pitch)
            gimbal_angle.yaw = float(self.gim_yaw)

            print gimbal_angle, shoot_msg
            self._gimbal_pub.publish(gimbal_angle)
            if not self.enable_shoot:
                shoot_msg.data = [ 0, 0, 0, 0]
                self._shoot_pub.publish(shoot_msg)
            
            rate.sleep()

    
    def init_detection(self):
        self.enemy_id = 0 
        self.enemy_z = 0 
        self.enemy_y = 0 
        self.enemy_x = 0 


    def gravity_comp(self):
    	distance = math.sqrt(self.enemy_x**2 + self.enemy_y**2 + self.enemy_z**2)
    	time_hit = distance/18;
    	
    	height_comp = 1 / 2 * 9.8 * (time_hit**2)
    	self.enemy_z += height_comp


   	def _velocity_callback(self, msg):
   		distance = math.sqrt(self.enemy_x**2 + self.enemy_y**2 + self.enemy_z**2)
    	time_hit = distance/18;
    	self.velocity_x = msg.twist.twist.translation.x
    	self.velocity_y = msg.twist.twist.translation.y
    	self.enemy_x -= time_hit*self.velocity_x
    	self.enemy_y -= time_hit*self.velocity_y


    def _transform_callback(self, msg):
        array_length = len(msg.transforms)
        if array_length > 0:
            for i in range(array_length):
                if msg.transforms[i] == self._last_id:
                    tag_id = i 
                else:
                    tag_id = 0 
                self.enemy_id 	= msg.transforms[tag_id].fiducial_id
                self.enemy_z 	= msg.transforms[tag_id].transform.translation.z
                self.enemy_y 	= msg.transforms[tag_id].transform.translation.y
                self.enemy_x 	= msg.transforms[tag_id].transform.translation.x
                self._last_id 	= self.enemy_id
        else:
            self.init_detection()


    def _calc_gimbal(self, x, y, z): 
        yaw 		= math.atan2(-y, x)
        pitch 		= math.atan2(z, math.sqrt(self.enemy_x**2 + self.enemy_y**2))
        self.gim_pitch = pitch
        self.gim_yaw   = yaw

if __name__ == '__main__':
    try:
        GimbalAiming()
    except rospy.ROSInterruptException:
        pass













