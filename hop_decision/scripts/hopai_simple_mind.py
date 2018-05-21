import rospy
import tf
import random
import math
import actionlib
from transitions import Machine
from geometry_msgs.msg import Pose2D, Point

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
class Hopai(object):
    health   = 3000
    bonus_size = 0.58 # bonus zone side length in meter
    pose     =[0,0,0] # x, y, theta 
    states   = [
        State(name = 'to_mid', on_enter=['to_mid_cb']),
        State(name = 'take_buff', on_enter=['take_buff_cb']),
        State(name = 'engage', on_enter=['engage_cb']),
        State(name = 'patrol', on_enter=['patrol_cb']),
        State(name = 'runhome', on_enter=['runhome_cb'])
        ]
    transitions = [
        ['reached_mid','to_mid','take_buff'],
        ['enemy_missing','engage','to_mid'],
        ['enemy_missing','engage','patrol', conditions='has_bonus'],
        ['enemy_detected','to_mid','engage'],
        ['enemy_detected','take_buff','engage'],
        ['enemy_detected','patrol','engage'],
        ['have_buff','take_buff','patrol'],
        ['taking_damage','to_mid','engage'],
        ['taking_damage','take_buff','engage'],
        ['taking_damage','patrol','engage']
    ]

    # Temporary goal publisher; should be a client to actionlib localization 
    goal_pub   = rospy.Publisher('goal_topic', Point, queue_size=10)
    
    
    def __init__(self,name='Hopai'):
        # obervation 
        self.name       = name
        self.bonus      = False
        self.enemy      = False  # Robot
        self.ene_camera = []
        self.target     = False  # Armor
        self.tar_camera = [] 
        self.being_hit  = False 
        self.hit_armor  = [] 
        self.goal_pose  = [0,0,0] # x, y, theta

        # fsm
        self.machine    = Machine(model=self, states=Hopai.states, transitions=Hopai.transitions, initial='to_mid')
        
        # update observation
        
        self.pos_listener    = tf.TransformListener()
        try:
            (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        self.pose       = [trans[0],trans[1],rot[2]]
        self.hp_sub     = rospy.Subscriber('self_hp',int,self.self_hp_update)
        
        #######################################################
        #### TO DO   MESSAGE TYPE !!!!! more subscribption ######
        self.enemy_sub  = rospy.Subscriber('enemy',,self.enemy_update)
        self.target_sub = rospu.Subscriber('target',,self.target_update)
        ###################################################



        # update fsm
        self.update()

        # spin
        rospy.spin()

    # Conditional Check
    def has_bonus(self):
        return self.bonus

    def if_enemy_detected(self):
        return self.enemy
        
    # if self is in the bonus zone
    def if_reached_mid(self):
        if self.pose(0) >= 4-bonus_size && self.pose(0) <= 4+bonus_size:
            if self.pose(1) >= 3-bonus_size && self.pose(1) <= 3+bonus_size:
                return True
        else:
            return False


#################################################
#####    TO DO
###################################################
    def if_taking_damage(self):
        pass #TO DO
####################################################
##################################################
#####      TO DO
###############################################
    def navigate(self):
        client = actionlib.SimpleActionClient('goal_pose',LocalizationAction)
        client.wait_for_server()
        goal = 
        client.send_goal(goal)
#######################################################
########################################################

    # Action Unit
    def idle(self):
        pass

    def dodge(self):
        pass


    # Obervation Callbacks

    def self_hp_update(self,msg):
        self.health = msg

###############################################
#####     TO DO
###############################################
    def enemy_update(self,msg):
        pass
    
    def target_update(self,msg):
        pass
###############################################
###############################################

    # State Behavior
    
    def to_mid_cb(self):
        self.goal_pose = [4.5,3.5,0]
        self.navigate()

    
    def take_buff_cb(self):
        self.goal_pose = [4.5,3.5,0]
        self.navigate()
        self.idle()


######################################
######      TO DO 
######################################
    def engage_cb(self):
        if 
        
######################################
######################################
    def patrol_cb(self):
        # goal 1
        self.goal_pose = [4.5,3.5,0]
        self.navigate()
        # goal 2
        self.goal_pose = [4.5,3.5,0]
        self.navigate()
        # goal 3
        self.goal_pose = [4.5,3.5,0]
        self.navigate()
        # goal 4
        self.goal_pose = [4.5,3.5,0]
        self.navigate()
        # ...

    def runhome_cb(self):
        # goal 1
        self.goal_pose = [0,0,0]
        self.navigate()
        # goal 2
        self.goal_pose = [0,0,0]
        self.navigate()
        # goal 3
        self.goal_pose = [0,0,0]
        self.navigate()
        # goal ...



    # Update
    def update(self):
        # update pose
        
        # 
        if self.if_enemy_detected():
            self.enemy_detected()
        else:
            self.enemy_missing() 

        if self.if_reached_mid():
            self.reached_mid()

        if self.has_buff():
            self.have_buff()
        
        if self.if_taking_damage():
            self.taking_damage()

        if self.health < 500:
            self.to_runhome()


if __name__ == '__main__':
    rospy.init_node('hop_simple_decision', anonymous=True)
    try:
        Hopai()
    except rospy.ROSInterruptException:
        pass


 