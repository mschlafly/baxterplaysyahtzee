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

#Global Variables
dice_vals = ['0']*5
POINTS_FULL_HOUSE = 25
POINTS_SMALL_STRAIGHT = 30
POINTS_LARGE_STRAIGHT = 40
POINTS_FIVE_OF_KIND = 50
LARGE_STRAIGHT_FORM = [1,1,1,1]
SMALL_STRAIGHT_FORM_PRE = [1,1,1,0]
SMALL_STRAIGHT_FORM_POST= [0,1,1,1]


dir_path = os.path.dirname(os.path.realpath(__file__))

class headdisplay():
	def __init__(self):
		rospy.init_node('headdisplay')
		self.dice=[0]*5
		self.numdice=[0]*6
		self.combination = 'none'
		#self.scorecard = [0]*13
		self.cardbool = [0]*14
		self.score=0
		#This is the order of the items in self.scorecard and self.cardbool
			#NONE = 0
			#ONES = 1
			#TWOS = 2
			#THREES = 3
			#FOURS = 4
			#FIVES = 5
			#SIXES = 6
			#THREE_OF_KIND = 7
			#FOUR_OF_KIND = 8
			#CHANCE = 9
			#FULL_HOUSE = 10
			#SMALL_STRAIGHT = 11
			#LARGE_STRAIGHT = 12
			#FIVE_OF_KIND = 13

		# Publish Starting Screen
		img = Image.new('RGB', (1024, 600), color = (229, 0, 11))
		font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 90, encoding="unic")
		fontsm = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 30, encoding="unic")
		d = ImageDraw.Draw(img)
		d.text((150,250), "Yahtzee Project!", font=font,fill=(255, 255, 255))
		d.text((10,10), "Turn: "+str(1), font=fontsm,fill=(0, 0, 0))
		d.text((10,50), "Roll: "+str(1), font=fontsm,fill=(0, 0, 0))
		d.text((10,90), "Score: "+str(self.score), font=fontsm,fill=(0, 0, 0))
		#img.save(dir_path+'/headdisplay.png')	
		# Call the xdisplay_image.py node to display the image on baxter's head
		#run=os.system("rosrun baxterplaysyahtzee xdisplay_image.py -f "+dir_path+'/headdisplay.png')

		
		#time.sleep(100)
		# When a new state is published, callback is called
		rospy.Subscriber("/statetopic", GameState, self.callback)
		rospy.spin()		
#time.sleep(100)
		#r.sleep()

	def callback(self,message):
		# Update display with new game information
		if message.state=="Dice Read":
			self.nextmove(message)
		else:
			img = Image.new('RGB', (1024, 600), color = (229, 0, 11))
			font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 90, encoding="unic")
			fontsm = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 30, encoding="unic")
			img.paste(new_im,(50,50))
			d = ImageDraw.Draw(img)
			d.text((150,250), message.state+"...", font=font,fill=(255, 255, 255))
			d.text((10,10), "Turn: "+str(message.turn), font=fontsm,fill=(0, 0, 0))
			d.text((10,50), "Roll: "+str(message.roll), font=fontsm,fill=(0, 0, 0))
			d.text((10,90), "Score: "+str(self.score), font=fontsm,fill=(0, 0, 0))
			img.save(dir_path+'/headdisplay.png')	
			# Call the xdisplay_image.py node to display the image on baxter's head
			run=os.system("rosrun baxterplaysyahtzee xdisplay_image.py -f "+dir_path+'/headdisplay.png')
	def nextmove(self,message):
		#Display the dice 
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
		d.text((10,90), "Score: "+str(self.score), font=fontsm,fill=(0, 0, 0))
		img.save(dir_path+'/headdisplay.png')	

		# If the turn is either 1 or 2, determine the 
		if (message.roll==1) or (message.roll==2):
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
			self.numdice[5]=self.dice.count(6)
			maxnum=self.numdice.index(max(self.numdice))+1

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
		else: 
		# Display box on scorecard to fill and update score
			# Determine the combination that gives baxter the most points
			self.dice.sort()
			counted = Counter(self.dice)
			# These first sections LIKELY provide the most points regardless
			if(len(counted) == 1 and self.cardbool[13] == 0):
				self.combination = 'YAHTZEE!!'		
				points = POINTS_FIVE_OF_KIND
				self.cardbool[13]=1
			elif(len(counted) == 2 and self.cardbool[10] == 0):
				self.combination = 'Full House'		
				points = POINTS_FULL_HOUSE
				self.cardbool[10]=1
			elif(self.straight() == 'LARGE_STRAIGHT' and self.cardbool[12] == 0):
				self.combination = 'Large Straight'		
				points = POINTS_LARGE_STRAIGHT
				self.cardbool[12]=1
			elif(self.straight() == 'SMALL_STRAIGHT' and self.cardbool[11] == 0):
				self.combination = 'Large Straight'		
				points = POINTS_SMALL_STRAIGHT
				self.cardbool[11]=1
			else:
				listpoints=[0]*10
				listname=['None','Ones','Twos','Threes','Fours','Fives','Sixes','Three of a Kind','Four of a Kind','Chance']
				# Determine the number with the most dice
				self.dice[0]=message.dice1
				self.dice[1]=message.dice2
				self.dice[2]=message.dice3
				self.dice[3]=message.dice4
				self.dice[4]=message.dice5
				self.numdice[0]=self.dice.count(1)
				if (self.numdice[0]==3) and (self.cardbool[7] == 0):
					listpoints[7]=self.numdice[0]*1
				elif (self.numdice[0]==4) and (self.cardbool[8] == 0):
					listpoints[8]=self.numdice[0]*1
				self.numdice[1]=self.dice.count(2)
				if (self.numdice[1]==3) and (self.cardbool[7] == 0):
					listpoints[7]=self.numdice[1]*1
				elif (self.numdice[0]==4) and (self.cardbool[8] == 0):
					listpoints[8]=self.numdice[1]*1
				self.numdice[2]=self.dice.count(3)
				if (self.numdice[2]==3) and (self.cardbool[7] == 0):
					listpoints[7]=self.numdice[2]*1
				elif (self.numdice[0]==4) and (self.cardbool[8] == 0):
					listpoints[8]=self.numdice[2]*1
				self.numdice[3]=self.dice.count(4)
				if (self.numdice[3]==3) and (self.cardbool[7] == 0):
					listpoints[7]=self.numdice[3]*1
				elif (self.numdice[0]==4) and (self.cardbool[8] == 0):
					listpoints[8]=self.numdice[3]*1
				self.numdice[4]=self.dice.count(5)
				if (self.numdice[4]==3) and (self.cardbool[7] == 0):
					listpoints[7]=self.numdice[4]*1
				elif (self.numdice[0]==4) and (self.cardbool[8] == 0):
					listpoints[8]=self.numdice[4]*1
				self.numdice[5]=self.dice.count(6)
				if (self.numdice[5]==3) and (self.cardbool[7] == 0):
					listpoints[7]=self.numdice[5]*1
				elif (self.numdice[0]==4) and (self.cardbool[8] == 0):
					listpoints[8]=self.numdice[5]*1

				if (self.cardbool[1] == 0):
					listpoints[1]=self.numdice[0]*1
				if (self.cardbool[2] == 0):
					listpoints[2]=self.numdice[1]*2
				if (self.cardbool[3] == 0):
					listpoints[3]=self.numdice[2]*3
				if (self.cardbool[4] == 0):
					listpoints[4]=self.numdice[3]*4
				if (self.cardbool[5] == 0):
					listpoints[5]=self.numdice[4]*5
				if (self.cardbool[6] == 0):
					listpoints[6]=self.numdice[5]*6
				if (self.cardbool[7] == 0):
					listpoints[7]=sum(self.dice)
				maxindex=listpoints.index(max(listpoints))
				points=listpoints[maxindex]
				self.combination=listname[maxindex]
				self.cardbool[maxindex]=1
			#Update score
			self.score=self.score+points
			#Display image
			img = Image.new('RGB', (1024, 600), color = (229, 0, 11))
			font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 90, encoding="unic")
			fontsm = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 30, encoding="unic")
			d = ImageDraw.Draw(img)
			d.text((150,250), "Chosen Catagory: "+str(self.combination) , font=font,fill=(255, 255, 255))
			d.text((10,10), "Turn: "+str(message.turn), font=fontsm,fill=(0, 0, 0))
			d.text((10,50), "Roll: "+str(message.roll), font=fontsm,fill=(0, 0, 0))
			d.text((10,90), "Score: "+str(self.score), font=fontsm,fill=(0, 0, 0))
			img.save(dir_path+'/headdisplay.png')	
			# Call the xdisplay_image.py node to display the image on baxter's head
			run=os.system("rosrun baxterplaysyahtzee xdisplay_image.py -f "+dir_path+'/headdisplay.png')
		def straight(self):
			d = [self.dice[i+1]-self.dice[i] for i in range(len(self.dice)-1)]
			if(d == LARGE_STRAIGHT_FORM):
				return 'LARGE_STRAIGHT'
			if(d == SMALL_STRAIGHT_FORM_PRE or d == SMALL_STRAIGHT_FORM_POST):
				return 'SMALL_STRAIGHT'
		
if __name__=='__main__':		
	try:
		headdisplay()
	except:
		rospy.loginfo("Done")
