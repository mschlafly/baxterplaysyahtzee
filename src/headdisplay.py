#!/usr/bin/python2

"""
Import
"""
import os
import rospy
from PIL import Image, ImageDraw, ImageFont

#statetopic=/statetopic;
#stateformat=

#In stateformat
#String for state
#Ints for  turn, roll, score, 

turnnum=1
rollnum=1
score=0

dir_path = os.path.dirname(os.path.realpath(__file__))
rospy.loginfo(dir_path+'/startingdisplay.png')
class headdisplay():
	def __init__(self):
		rospy.init_node('headdisplay')
		# Publish Starting Screen
		img = Image.new('RGB', (1024, 600), color = (229, 0, 11))
		font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 90, encoding="unic")
		fontsm = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 30, encoding="unic")
		d = ImageDraw.Draw(img)
		d.text((150,250), "Yahtzee Project!", font=font,fill=(255, 255, 255))
		d.text((10,10), "Turn: "+str(turnnum), font=fontsm,fill=(0, 0, 0))
		d.text((10,50), "Roll: "+str(rollnum), font=fontsm,fill=(0, 0, 0))
		d.text((10,90), "Score: "+str(score), font=fontsm,fill=(0, 0, 0))
		
		img.save(dir_path+'/startingdisplay.png')	
		rospy.loginfo(1)
		run=os.system("rosrun baxterplaysyahtzee xdisplay_image.py -f "+dir_path+'/startingdisplay.png')
		
		#rospy.wait_for_service(statetopic)
		#disp = rospy.ServiceProxy(statetopic,stateformat)
		
		#When a state gets published-> screen displays state name+ ...

		#When scores are published (1) display the dice (2) determine next move and publishes to a topic

		
try:
	headdisplay()
except:
	rospy.loginfo("Done")
