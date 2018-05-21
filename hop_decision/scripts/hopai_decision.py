import rospy
import tf
import random
from transitions import Machine
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from hop_msgs.msg import *
##################################################
# The is the decision module of the robot using
# a FSM in python. pytransitions is the package 
# used for FSM implementation. FOr more info:
# https://github.com/pytransitions/transitions


# Version: 0.1
# Author: Rui
##################################################
class Hopai(object):
    position = [0,0,0]
    velocity = [0,0,0]
    health   = 3000
    bonus    = False
    enemy    = False
    states   = ['to_mid','take_buff','engage','disengage','patrol','ambush']
    transitions = [
        ['enemy_missing','to_mid','take_buff'],
        ['enemy_found','to_mid','engage'],
        ['enemy_found','take_buff','engage'],
        ['have_buff','take_buff','patrol'],
        ['enemy_found','patrol','engage'],
        ['enemy_missing','engage','patrol'],
        ['two_enemy','engage','disengage'],
        ['low_health','engage','disengage'],
        ['','disengage','ambush'],
        [],
        []
    ]


    goal_pub   = rospy.Publisher('goal_topic', Point, queue_size=10)
    
    
    def __init__(self,name='Hopai'):
        self.name       = name
        self.machine    = Machine(model=self, states=Hopai.states, transitions=Hopai.transitions, initial='to_mid')


    # Conditional Check

    def if_enemy_found(self):
        pass



    def if_enemy_lost(self):
        pass



    # Action Unit
    def idle(self):
        pass



    def navigate(self,goal):
        pass
    


    def dodge(self):
        pass


    # Callbacks
    def pos_update(self):
        pass

    def vel_update(self,Odometry):
        pass

    def self_hp_update(self,int):
        pass
    
    def on_enter_patrol(self):
        pass


    def on_exit_patrol(self):
        pass


    def on_enter_engage(self):
        pass


    def on_exit_engage(self):
        pass


    # Update
    def update(self):
        if self.is_to_mid()

        if self.is_







self.pos_sub    = rospy.Subscriber('odometry',Odometry,self.pos_update)
        self.odom_sub   = rospy.Subscriber('odometry',Odometry,self.vel_update)
        self.hp_sub     = rospy.Subscriber('self_hp',int,self.self_hp_update)
        self.enemy_sub  = rospy.Subscriber('enemy',,self.enemy_update)


        self.update()
        rospy.spin()
 