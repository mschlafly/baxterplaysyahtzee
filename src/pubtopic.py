#!/usr/bin/python2
#-*- coding: utf-8 -*-
"""
Import
"""
import os
import rospy
import time 
from PIL import Image, ImageDraw, ImageFont
from baxterplaysyahtzee.msg import GameState 


class pubtopic():
	def __init__(self):
		rospy.init_node('pubtopic')
		r=rospy.Rate(.1)
		pub = rospy.Publisher('/statetopic', GameState, queue_size=10)
		pretend=GameState()
		pretend.state="Dice Read"
		pretend.turn=5
		pretend.roll=2
		pretend.dice1=1
		pretend.dice2=2
		pretend.dice3=3
		pretend.dice4=4
		pretend.dice5=5
		pretend.dice1color='b'
		pretend.dice2color='b'
		pretend.dice3color='r'
		pretend.dice4color='bl'
		pretend.dice5color='y'
		while not rospy.is_shutdown():
			pub.publish(pretend)
			#rospy.loginfo(1)
			r.sleep()

#def main():
	
if __name__=='__main__':		
	try:
		pubtopic()
	except:
		rospy.loginfo("Done")
