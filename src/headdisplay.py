#!/usr/bin/python2
#-*- coding: utf-8 -*-
"""
Import
"""
import os
import rospy
import numpy
import time
from PIL import Image, ImageDraw, ImageFont
from baxterplaysyahtzee.msg import GameState,KeepDice
from collections import Counter

score=0

dice_vals = ['0']*5


dir_path = os.path.dirname(os.path.realpath(__file__))
rospy.loginfo(dir_path+'/headdisplay.png')

class headdisplay():
	def __init__(self):
		rospy.init_node('headdisplay')
		r=rospy.Rate(10)
		rospy.loginfo(1)
		self.dice=[0]*5
		self.numdice=[0]*5
		self.combination = 'combo'
		rospy.loginfo(1)

		# Publish Starting Screen
		img = Image.new('RGB', (1024, 600), color = (229, 0, 11))
		font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 90, encoding="unic")
		fontsm = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 30, encoding="unic")
		fontdice = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf", 24)
		d = ImageDraw.Draw(img)
		d.text((150,250), "Yahtzee Project!", font=font,fill=(255, 255, 255))
		d.text((10,10), "Turn: "+str(1), font=fontsm,fill=(0, 0, 0))
		d.text((10,50), "Roll: "+str(1), font=fontsm,fill=(0, 0, 0))
		d.text((10,90), "Score: "+str(score), font=fontsm,fill=(0, 0, 0))
		d.text((10,110), "⚀ ⚁", font=fontdice,fill=(0, 0, 0))
		#img.save(dir_path+'/headdisplay.png')	
		# Call the xdisplay_image.py node to display the image on baxter's head
		#run=os.system("rosrun baxterplaysyahtzee xdisplay_image.py -f "+dir_path+'/headdisplay.png')

		
		#time.sleep(100)
		# When a new state is published, callback is called
		rospy.loginfo(1)
		rospy.Subscriber("/statetopic", GameState, self.callback)
		time.sleep(100)
		#r.sleep()

	def callback(self,message):
		# Update display with new game information

		rospy.loginfo(1)
		rospy.loginfo(message.state)
		if message.state=="Dice Read":
			rospy.loginfo(1)
			self.nextmove(message)
		else:
			rospy.loginfo(1)
			img = Image.new('RGB', (1024, 600), color = (229, 0, 11))
			font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 90, encoding="unic")
			fontsm = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 30, encoding="unic")
			img.paste(new_im,(50,50))
			d = ImageDraw.Draw(img)
			d.text((150,250), message.state+"...", font=font,fill=(255, 255, 255))
			d.text((10,10), "Turn: "+str(message.turn), font=fontsm,fill=(0, 0, 0))
			d.text((10,50), "Roll: "+str(message.roll), font=fontsm,fill=(0, 0, 0))
			d.text((10,90), "Score: "+str(score), font=fontsm,fill=(0, 0, 0))
			img.save(dir_path+'/headdisplay.png')	
			# Call the xdisplay_image.py node to display the image on baxter's head
			run=os.system("rosrun baxterplaysyahtzee xdisplay_image.py -f "+dir_path+'/headdisplay.png')
	def nextmove(self,message):
		#Display the dice 
		rospy.loginfo(1)
		img = Image.new('RGB', (1024, 600), color = (229, 0, 11))
		font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 90, encoding="unic")
		fontsm = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 30, encoding="unic")

		#Image pasting from Vikas
		dice_vals[0] = dir_path+"/DicePictures/" + message.dice1color + str(message.dice1) + ".jpg"
		dice_vals[1] = dir_path+"/DicePictures/" + message.dice2color + str(message.dice2) + ".jpg"
		dice_vals[2] = dir_path+"/DicePictures/" + message.dice3color + str(message.dice3) + ".jpg"
		dice_vals[3] = dir_path+"/DicePictures/" + message.dice4color + str(message.dice4) + ".jpg"
		dice_vals[4] = dir_path+"/DicePictures/" + message.dice5color + str(message.dice5) + ".jpg"

		images = map(Image.open, dice_vals);

		widths, heights = zip(*(i.size for i in images))
		total_width = sum(widths)
		max_height = max(heights)

		new_im = Image.new('RGB', (total_width, max_height))
		x_offset = 0
		for im in images:
		    new_im.paste(im, (x_offset,0))
		    x_offset += im.size[0]
		draw = ImageDraw.Draw(new_im)
		#new_im.save('test.jpg')
		#maxsize = (600, 600)
		#new_im = new_im.thumbnail(maxsize)
		new_im=new_im.resize((677,200), resample=0, box=None)
		img.paste(new_im,(200,350))

		#me
		d = ImageDraw.Draw(img)
		d.text((150,250), "Current Roll", font=font,fill=(255, 255, 255))
		d.text((10,10), "Turn: "+str(message.turn), font=fontsm,fill=(0, 0, 0))
		d.text((10,50), "Roll: "+str(message.roll), font=fontsm,fill=(0, 0, 0))
		d.text((10,90), "Score: "+str(score), font=fontsm,fill=(0, 0, 0))
		img.save(dir_path+'/headdisplay.png')	

		# if turn is 1 or 2
		# Determine the number with the most dice
		self.dice[0]=message.dice1
		self.dice[1]=message.dice2
		self.dice[2]=message.dice3
		self.dice[3]=message.dice4
		self.dice[4]=message.dice5
		self.numdice[0]=self.dice.count(1)
		self.numdice[1]=self.dice.count(2)
		self.numdice[2]=self.dice.count(3)
		self.numdice[3]=self.dice.count(4)
		self.numdice[4]=self.dice.count(5)
		rospy.loginfo(self.numdice)
		maxnum=self.numdice.index(max(self.numdice))+1
		rospy.loginfo(maxnum)

		#Publish the dice that are kept and left in topic
		pub = rospy.Publisher('/reroll', KeepDice, queue_size=10)
		di=KeepDice()
		if message.dice1==maxnum:
			di.dice1=0
		else:
			di.dice1=1
		if message.dice2==maxnum:
			di.dice2=0
		else:
			di.dice2=1
		if message.dice3==maxnum:
			di.dice3=0
		else:
			di.dice3=1
		if message.dice4==maxnum:
			di.dice4=0
		else:
			di.dice4=1
		if message.dice5==maxnum:
			di.dice5=0
		else:
			di.dice5=1
		pub.publish(di)

		counted = Counter(self.dice)
		rospy.loginfo(counted)
		rospy.loginfo(self.dice)

		#is it done? then display score ubuntu
		#if not, calculate next move + display 
		
		#When a state gets published-> screen displays state name+ ...

		#When scores are published (1) display the dice (2) determine next move and publishes to a topic

#def main():
	
if __name__=='__main__':		
	try:
		headdisplay()
	except:
		rospy.loginfo("Done")
