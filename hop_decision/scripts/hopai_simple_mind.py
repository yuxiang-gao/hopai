#!/usr/bin/env python

import rospy
import tf
import random
import math
from transitions import Machine, State
from geometry_msgs.msg import Pose2D, Point, PoseStamped
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry
from hop_msgs.msg import *

##################################################
# The is the simple version of the decision module 
# of the robot using a FSM in python. pytransitions 
# is the package used for FSM implementation. 
# For more info:
# https://github.com/pytransitions/transitions
#
# Version: 0.1
# Author: Rui
##################################################
class Hopai(object):
	health   = 3000
	#bonus_size = 0.58 # bonus zone side length in meter
	cur_pose     =[0.5,0.5,0.5] # x, y, theta 
	states   = [
		State(name = 'start'),
		State(name = 'to_mid', on_enter=['to_mid_cb']),
		#State(name = 'take_buff', on_enter=['take_buff_cb']),
		State(name = 'engage', on_enter=['engage_cb'], on_exit=['disengage_cb']),
		State(name = 'patrol', on_enter=['patrol_cb']),
		State(name = 'runhome', on_enter=['runhome_cb'])
		]
	
	transitions = [
		['game_on','start','to_mid'],
		#['reached_mid','to_mid','take_buff'],
		['enemy_missing','engage','to_mid'],
		['enemy_missing','engage','patrol'], #conditions='has_bonus'],
		['enemy_detected','to_mid','engage'],
		#['enemy_detected','take_buff','engage'],
		['enemy_detected','patrol','engage'],
		#['have_buff','take_buff','patrol'],
		['taking_damage','to_mid','engage'],
		#['taking_damage','take_buff','engage'],
		['taking_damage','patrol','engage']
		]
	
	# Temporary goal publisher; should be a client to actionlib localization 
	goal_pub   = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
	mode_pub   = rospy.Publisher('chassis_mode',Int32, queue_size=1)

	def __init__(self,name='Hopai'):
		# obervation 
		self.name       = name
		#self.bonus      = False
		self.enemy      = False  # Robot
		self.ene_camera = 2
		self.target     = False  # Armor
		self.enemy_dist = 3
		self.being_hit  = False 
		self.hit_armor  = 0 
		self.goal_pose  = [0,0,0] # x, y, theta
		# fsm
		self.machine    = Machine(model=self, states=Hopai.states, transitions=Hopai.transitions, initial='to_mid')
		print('machine initialized')
		# update observation
		self.pos_listener  = tf.TransformListener()
		try:
			trans = [0.5,0.5,0]
			rot   = [0,0,0]
			(trans,rot) = self.pos_listener.lookupTransform('base_link', 'map', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
		self.cur_pose   = [trans[0],trans[1],rot[2]]
		print('pose updated')
		self.hp_sub     = rospy.Subscriber('health_point',Ref,self.self_hp_update)
		self.enemy_sub0 = rospy.Subscriber('enemy_left',Bool,self.enemy_update0)
		self.enemy_sub1 = rospy.Subscriber('enemy_right',Bool,self.enemy_update1)
		#self.enemy_subp  = rospy.Subscriber('enemy_prim',,self.enemy_update2)
		self.target_sub2= rospy.Subscriber('target_mid',EnemyPose,self.target_update2)
		self.hit_sub    = rospy.Subscriber('armor_id',Id,self.hit_update)
		# update fsm
		self.update()
		print('cycle finished')
		# spin
		rospy.spin()


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

	def finished(vec1,vec2):
		diff1 = math.fabs(vec1[0]-vec2[0])
		diff2 = math.fabs(vec1[1]-vec2[1])
		diff3 = math.fabs(vec1[2]-vec2[2])
		if diff1 < 0.1 and diff2 < 0.2 and diff3 < 0.174533:
			return False

    # Conditional Check
    #def has_bonus(self):
    #    return self.bonus


	def if_enemy_detected(self):
		return (self.enemy or self.target)

        
    # if self is in the bonus zone
    #def if_reached_mid(self):
    #    if self.cur_pose(0) >= 4-bonus_size && self.cur_pose(0) <= 4+bonus_size:
    #        if self.cur_pose(1) >= 3-bonus_size && self.cur_pose(1) <= 3+bonus_size:
    #            return True
    #    else:
    #        return False


	def if_taking_damage(self):
		return self.being_hit

    # Action units

	def navigate(self):
		goal  = self.goal_pose
		post_stamp_goal = to_pose_stamped(goal)
		self.goal_pub.publish(post_stamp_goal)
		while not self.finished(goal,self.cur_pose):
			rospy.sleep(1)
		
	def idle(self):
		pass


####################################################
####   TO  DO 
####################################################
	def turn_to_target(self):
		self.goal_pose = self.cur_pose
		if not self.enemy:
			if ene_camera == 0:
				self.goal_pose[2] += 0.785398
			if ene_camera == 1:
				self.goal_pose[2] -= 0.785398

		elif self.being_hit:
			if self.hit_armor == 1:
				pass
			elif self.hit_armor == 0:
				self.goal_pose[2] -= 1.5708
			elif self.hit_armor == 2:
				self.goal_pose[2] += 1.5708
			elif self.hit_armor == 3:
				self.goal_pose[2] += 3.1416
		self.navigate()


	def start_dodge(self):
		self.mode_pub.publish(4)

    
	def stop_dodge(self):
		self.mode_pub.publish(5)
##################################################
    # Obervation Callbacks

	def self_hp_update(self,msg):
		self.health = msg

	def enemy_update0(self,msg):
		if msg:
			self.ene_camera = 0
			self.enemy      = True

	def enemy_update1(self,msg):
		if msg:
			self.ene_camera = 1
			self.enemy      = True

	def target_update2(self,msg):
		self.target = True
		self.dist_target = msg.enemy_dist
         
###############################################
###############################################
    # State Behavior
    
	def to_mid_cb(self):
		self.goal_pose = [2.65,1.69,1.5708]
		self.navigate()

    #def take_buff_cb(self):
    #    self.goal_pose = [4.5,3.5,0]
    #    self.navigate()
    #    self.idle()


	def engage_cb(self):
		self.turn_to_target()

		self.start_dodge()

	def disengage_cb(self):
		self.stop_dodge()

######################################
######################################
#####        TO DO
#####     DECIDE WAY POINT
#########################################
	def patrol_cb(self):
		time1 = rospy.Time(5)
		time2 = rospy.Time(7)
		# goal 1
		while not rospy.is_shutdown(): 
			self.goal_pose = [2.6,1.6,1.5708]
			self.navigate()
			time1.sleep()
        # goal 2
			self.goal_pose = [4.5,2.5,-0.5]
			self.navigate()
			time1.sleep()
        # goal 3
			self.goal_pose = [5.5,3.4,1.5708]
			self.navigate()
			time1.sleep()
        # goal 4
			self.goal_pose = [7,0.75,-3.14]
			self.navigate()
			time1.sleep()

			self.goal_pose = [0.5,0.5,0.5]
			self.navigate()
			time2.sleep()
        # ...

	def runhome_cb(self):
		# goal 1
		self.goal_pose = [0.5,0.5,0.5]
		self.navigate()
        # goal ...
#############################################

    # Update
	def update(self):
		# 
		if self.is_start():
			self.game_on()
		if not self.is_engage():
			if self.if_enemy_detected():
				self.enemy_detected()
		else:
			if not self.if_enemy_detected():
				self.enemy_missing()

        #if self.has_buff():
        #    self.have_buff()        
		if self.if_taking_damage():
			self.taking_damage()

		if self.health < 1000:
			self.to_runhome()


if __name__ == '__main__':
	rospy.init_node('hop_simple_decision', anonymous=True)
	try:
		hopai= Hopai()
		
	except rospy.ROSInterruptException:
		pass


 
	
