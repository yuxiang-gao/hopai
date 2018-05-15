#!/usr/bin/env python

import rospy
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import UInt8MultiArray
from hai_msg.msg import EnemyPos
from collections import Counter
import math

class GimbalAiming:
    def __init__(self):
        rospy.init_node('gimbal_aiming', anonymous=True)
        self.init_detection()
        rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self._transform_callback)
        self._gimbal_pub = rospy.Publisher('enemy_pos', EnemyPos, queue_size=1)
        self._shoot_pub = rospy.Publisher('shoot', UInt8MultiArray, queue_size=1)
        rate = rospy.Rate(20) # 10hz
        self._cnt = 0
        self._out_cnt = 0
        self._last_id = 0
        self._shoot_dict = Counter()
        self.enemy_dist = 0
        self.enemy_pitch = 0
        self.enemy_yaw = 0
        self.enable_shoot = False
        print 'init...'
        self._init_cnt = 0
        while not rospy.is_shutdown():
            enemy_pos = EnemyPos()
            shoot_msg = UInt8MultiArray(data=[0, 0, 1, 100])
            if self._init_cnt < 50 and self.enable_shoot:
                self._init_cnt += 1
                self._gimbal_pub.publish(enemy_pos)
                self._shoot_pub.publish(shoot_msg)
                rate.sleep()
                continue
            if self.enemy_id == 0:
                self._cnt = 0
                self._out_cnt += 1
                if self._out_cnt > 20:
                    self.enemy_dist = 0
                    self.enemy_pitch = 0
                    self.enemy_yaw = 0
            else:
                self._cnt = self._cnt + 1
                if self.enemy_z > 5.0 or self.enemy_z == 0 or self._cnt < 5 or self._shoot_dict[str(self.enemy_id)] > 10:
                    pass
                else:
                    self._out_cnt = 0
                    #self._shoot_dict[str(self.enemy_id)] += 1
                    self.enemy_dist, self.enemy_pitch, self.enemy_yaw = self._calc_gimbal(self.enemy_x, self.enemy_y, self.enemy_z)
                    shoot_msg.data = [1, 0, 1, 100]
            enemy_pos.enemy_dist = float(self.enemy_dist)
            enemy_pos.enemy_pitch = float(self.enemy_pitch) 
            enemy_pos.enemy_yaw = float(self.enemy_yaw)
            print enemy_pos, shoot_msg
            self._gimbal_pub.publish(enemy_pos)
            if not self.enable_shoot:
                shoot_msg.data = [ 0, 0, 0, 0]    
            self._shoot_pub.publish(shoot_msg)
            rate.sleep()

    
    def init_detection(self):
        self.enemy_id = 0
        self.enemy_z = 0
        self.enemy_y = 0
        self.enemy_x = 0

    def _transform_callback(self, msg):
        array_length = len(msg.transforms)
        if array_length > 0:
            for i in range(array_length):
                if msg.transforms[i] == self._last_id:
                    tag_id = i
                else:
                    tag_id = 0
                self.enemy_id = msg.transforms[tag_id].fiducial_id
                self.enemy_z = msg.transforms[tag_id].transform.translation.z
                self.enemy_y = msg.transforms[tag_id].transform.translation.y
                self.enemy_x = msg.transforms[tag_id].transform.translation.x
                self._last_id = self.enemy_id
        else:
            self.init_detection()


    def _calc_gimbal(self, x, y, z):
        yaw = math.atan2(-x, z)
        pitch = math.atan2(0.16 + y, z)
        dist = math.sqrt(x**2 + y**2 + z**2)
        return (dist, pitch, yaw)

if __name__ == '__main__':
    try:
        GimbalAiming()
    except rospy.ROSInterruptException:
        pass

