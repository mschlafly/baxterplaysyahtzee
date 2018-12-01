#!/usr/bin/env python

#
# Vikas Gupta
#

import rospy
from std_msgs.msg import String

import sys
import rospy
import numpy 
import operator
from std_msgs.msg import String
import roslib
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3 

"""
DEFINES
"""

OUTOFBOUNDS = -1
DONTMOVE = -2
Vision = True 
VIZ_ONLY = True
SKIP = 15
DOT_DETECTION = False
frames = 0

RATE = 5
IMAGE_X = 640
IMAGE_Y= 480
CENTER = [IMAGE_X/2, IMAGE_Y/2]
SEGMENTS = 4
MINWIDTH = 0.1
DEADPASSRADIUS= IMAGE_Y/4
QSIZE = 10

##dots##
POS = 255
NEG = 0
SHOW_NEG = False
MAX_SIZE_FILTER = 200# pixel count of blob to filter
MIN_SIZE_FILTER = 10# pixel count of blob to filter
DEBUG_SIZE = 1000

##


xqueue = []
yqueue = []
move = True
(send_x, send_y) = (IMAGE_X/2,IMAGE_Y/2)

prev_x = 0
prev_y = 0
prev_max_count = 0
prev_s = 0
prev_e = 0
timer = 0

def camera_info_callback(camera_info):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', camera_info)
    pass

def image_raw_callback(image):
    global move, send_x, send_y, frames

    from cv_bridge import CvBridge, CvBridgeError
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

    frames += 1

    if(DOT_DETECTION and frames % SKIP != 0):
        return

    obj_x, obj_y, mr = coords_for_object(image, cv_image)

    if(DOT_DETECTION):
        dots(cv_image)

    if(VIZ_ONLY):
        #cv2.imshow("images", cv_image)
        #cv2.waitKey(3)
        return
    
    new_x = obj_x 
    new_y = obj_y 
    cp = euclidean_distance(CENTER, [new_x, new_y])
    rospy.loginfo("DIST FROM CP %d", cp)
    if(mr < MINWIDTH):
        rospy.loginfo("DONT MOVE MINWIDTH")
        send_x = DONTMOVE
        send_y = DONTMOVE
        return
    elif(cp < DEADPASSRADIUS):
        rospy.loginfo("DONT MOVE WITHINRANGE")
        send_x = DONTMOVE 
        send_y = DONTMOVE 
    else: 
        mpx, mpy, mpd = midpoint(CENTER, [new_x, new_y])
        while(True):
            mpx, mpy, mpd = midpoint(CENTER, [mpx, mpy])
            rospy.loginfo("MP dis %d", mpd)
            rospy.loginfo("OK MOVE from %d %d to %d %d", obj_x, obj_y, mpx, mpy)
            rospy.loginfo("MP dis %d", mpd)

            while(mpd > mr/2):
                send_x = DONTMOVE
                send_y = DONTMOVE
    
            new_x = mpx
            new_y = mpy
            send_x = new_x
            send_y = new_y

def coords_for_object(image, cv_image):
    global prev_x, prev_y
    print"===========COORDS FOR OBJECT=============="

    # define the list of boundaries for BGR
    boundaries = [
        #[b , g, r]
	#([10, 0, 0], [255, 240, 55]), #orig_blue

	([90, 0, 0], [255,  100, 80]), #orig_blue
    ]
    # loop over the boundaries
    for (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries
	lower = numpy.array(lower, dtype = "uint8")
	upper = numpy.array(upper, dtype = "uint8")
 
	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(cv_image, lower, upper)
        output = cv2.findNonZero(mask)

        # Maximum Run Start End X, Y
        y = 0
        ymax = 0
        for i in range(len(mask)):
            isum = numpy.sum(mask[i])
            if(isum > ymax):
                ymax = isum
                y = i

        max_count = 0
        s = 0
        run_count = 0

        #determine common color
        #bx = 400
        #by = 600
        #rospy.loginfo("%s %s b %d g %d r %d" , type(cv_image), cv_image.shape, cv_image[bx][by][0], cv_image[bx][by][1], cv_image[bx][by][2] )

        for i in range(1, len(mask[y])):
            if(mask[y][i] == 0):
                run_count = 0
                continue
            run_count += 1

            #rospy.loginfo("%s %s b %d g %d r %d" , type(cv_image), cv_image.shape, cv_image[y][i][0], cv_image[y][i][1], cv_image[y][i][2] )

            if(run_count > max_count):
                max_count = run_count
                s = i - max_count

        r = max_count / 2
        e = s+max_count
        x = (s+e)/2

        #fx, fy = filter(x, y)
        #if(fx == OUTOFBOUNDS or fy == OUTOFBOUNDS):
            #prev_x = x
            #prev_y = y  
            #return fx, fy

        #x, y, s, e , max_count = simple_filter(x, y, s, e, max_count)
        #x, y, = simple_filter(x, y)

        #r = max_count / 2
        #e = s+max_count
        #x = (s+e)/2
       
        rospy.loginfo("y %d ymax: %d, mr %d s %d e %d x %d r %d ", y, ymax, max_count, s, e, x, r)
	output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        cv2.circle(output, (s,y), r/4, (0,0,255), 4)
        cv2.circle(output, (x,y), r, (0, 255, 0), 4)
        cv2.circle(output, (e,y), r/4, (0, 0, 255), 4)
 
    # show images
    cv2.imshow("cv_images", numpy.hstack([cv_image, output]))
    cv2.waitKey(3)
   
    return x, y, max_count


def midpoint (p1, p2):
    x = (p1[0] + p2[0])/2
    y = (p1[1] + p2[1])/2
    d = euclidean_distance(p1, [x, y])
    #print "midpoint C to obj cx", p1[0], "cy", p1[1], "x", x, "y", y, "d", d
    return x, y, d

#def simple_filter(x, y, s, e, max_count):
def simple_filter(x, y):
    global prev_x, prev_y

    if(prev_x != 0 and prev_y != 0):
        if((abs(x - prev_x) > 0.2 * prev_x) or (abs(y - prev_y) > prev_y - 0.2)):
            prev_x = x
            prev_y = y
            #prev_s = s
            #prev_e = e 
            #prev_max_count = max_count
            #return x, y, s, e, max_count
            return x, y
#
    d = euclidean_distance([prev_x, prev_y], [x, y])
    if(d < 10):
        prev_x = x
        prev_y = y
        #prev_s = s
        #prev_e = e 
        #prev_max_count = max_count
        #return x, y, s, e, max_count
        return x, y

    #return x, y, s, e, max_count
    return x, y

    """
    x = (x + prev_x) / 2
    prev_x = x
    
    y = (y + prev_y) / 2
    prev_y = y
    return x, y
    """

def euclidean_distance(p1, p2):
    from scipy.spatial import distance
    dist = distance.euclidean(p1, p2) 
    return dist

def filter(x, y):
    global xqueue 
    global yqueue 

    if(len(xqueue) > QSIZE):
        rospy.loginfo("deleting")
        del xqueue[0]
    xqueue.append(x)
    avg = numpy.sum(xqueue) / QSIZE
    rospy.loginfo(rospy.get_caller_id() + "x %d avg %d qlen %d", x, avg, len(xqueue))
    if(x < 0.8*avg  or x > 1.2*avg):
        x = OUTOFBOUNDS

    if(len(yqueue)+1 > QSIZE):
        del yqueue[0]
    xqueue.append(y)
    avg = numpy.sum(xqueue) / QSIZE
    if(y < 0.8*avg  or y > 1.2*avg):
        y = OUTOFBOUNDS
    return x, y


lut = {}

def dots(img):
    current_label = 0
    global lut

    img_l = init_img_label_arr(img)

    for i in range(0, img.shape[0]):
        for j in range(0, img.shape[1]):
            if(cp(img, i, j, True) == NEG): # background
                add_label(img_l, i, j, NEG)
                continue

            ul = ap(img_l, i, j, False)
            ll = pp(img_l, i, j, False)

            if(i == 0): # first row
                if(j == 0): # first column
                    add_label(img_l, i, j, current_label)
                elif(ll > NEG):
                    add_label(img_l, i, j, ll)
            elif(j == 0): # not first row, first column
                if(ul > NEG):
                    add_label(img_l, i, j, ul)
            # normal case
            if(ul > NEG and ll > NEG and ul == ll): # same label
               add_label(img_l, i, j, ul)
            elif((ul != ll) and (ul == NEG or ll == NEG)): # either is 0
                add_label(img_l, i, j, max(ul, ll))
            #elif(ul > NEG and ll > NEG):
               #add_label(img_l, i, j, ul)
            else: # no neighbors new label
                current_label += 1
                lut[current_label] = current_label
                #print "current labels", current_label
                add_label(img_l, i, j, current_label)
    lut_pass(img_l)
    l = labels()

    lc = print_label_count(img_l)

    if(MIN_SIZE_FILTER > 0):
        filter_rest(lc, img_l)
        print_label_count(img_l)

    #print img_l

    if(MIN_SIZE_FILTER > 0):
        filter_rest(lc, img_l)
        print_label_count(img_l)

    #print img_l
    colorize(img_l, img)

    cv2.imshow("hmm", img)
    cv2.waitKey(3)

def colorize(img_l, img):
    lc = label_count(img_l)
    neg_lookup = 0
    for i in range (0, len(img_l)):
        for j in range(0, len(img_l[i])):
            cl = cp(img_l, i, j, False)
            if(cl > NEG):
                if(cl in lc.keys()):
                    #print cl
                    img[i][j][0] = 240
                    img[i][j][1] = cl/2
                    img[i][j][2] = cl/3

def print_label_count(img_l):
    lc = label_count(img_l)
    rospy.loginfo("count %d",len(lc))
    return lc

def filter_rest(lc, img_l):
    sorted_lc = sorted(lc.items(), key=operator.itemgetter(1))
    #print "remove <", SIZE_FILTER, "from:", len(sorted_lc[:-1]), len(sorted_lc[:-1]), sorted_lc[:-1]
    #rospy.loginfo("keep SIZE %d - %d ", MIN_SIZE_FILTER,MAX_SIZE_FILTER)

    for i in range (0, len(img_l)):
        for j in range(0, len(img_l[i])):
            cl = cp(img_l, i, j, False)
            if(cl > NEG):
                #print "filter: check cl"
                e = equiv(cl)
                for l in sorted_lc:
                    if(l[1] > MIN_SIZE_FILTER and l[1] < MAX_SIZE_FILTER):
                        #print "for l", l, "keep e: ", e, "of size", l[1]
                        continue
                    #print "for l", l, "remove e: ", e
                    if(e == l[0]):
                        add_label(img_l, i, j, NEG)

def label_count(img_l):
    p = {}
    for i in range (0, len(img_l)):
        for j in range(0, len(img_l[i])):
            cl = cp(img_l, i, j, False)
            if(cl > NEG):
                l = equiv(cl)
                if(l in p.keys()):
                   p[l] += 1
                else:
                   p[l] = 1
            elif(SHOW_NEG):
                if(cl in p.keys()):
                   p[cl] += 1
                else:
                   p[cl] = 1
    return p

def labels():
    global lut
    l = {}
    for key in lut:
        if(lut[key] in l):
           l[lut[key]] += 1
        else:
           l[lut[key]] = 1
    return l

def relabel(eql, minl):
    global lut
    lut[eql] = minl
    label = equiv(eql)
    #print "relabeling", eql, "to", label
    lut[eql] = label

def equiv(eql):
    global lut
    if(eql == NEG):
        return NEG
    if(eql == lut[eql]):
        #print "return", eql
        return eql
    return equiv(lut[eql])

def check_label(img_l, i, j):
    cl = cp(img_l, i, j, False)
    ul = ap(img_l, i, j, False)
    ll = pp(img_l, i, j, False)

    if(cl == NEG):
        return

    cl = equiv(cl)
    ul = equiv(ul)
    ll = equiv(ll)

    if(ul > NEG and ll > NEG and ul == ll): # same label
        add_label(img_l, i, j, ul)
    elif((ul != ll) and (ul == NEG or ll == NEG)): # either is 0
        add_label(img_l, i, j, max(ul, ll))
    elif(ul > NEG and ll > NEG and ul != ll): # both > NEG
        ul = equiv(ul)
        ll = equiv(ll)

        minl = NEG
        if(ul < ll):
            minl = ul
            relabel(ll, ul)
        elif (ll < ul):
            minl = ll
            relabel(ul, ll)
        add_label(img_l, i, j, minl)

def lut_pass(img_l):
    for i in range (0, len(img_l)):
        for j in range(0, len(img_l[i])):
            check_label(img_l, i, j)

def init_img_label_arr(img):
    img_l = []
    for i in range(0, img.shape[0]):
        new = []
        for j in range(0, img.shape[1]):
             new.append(0)
        img_l.append(new)
    return img_l

def add_label(arr, i, j, label):
    arr[i][j] = label
    return arr

def cp(arr, i, j, rgb):
    if(rgb):
        return color2bw(arr[i][j][0])
    return arr[i][j]

def pp(arr, i, j, rgb):
    if(rgb):
        return color2bw(arr[i][j-1][0])
    return arr[i][j-1]

def ap(arr, i, j, rgb):
    if(rgb):
        return color2bw(arr[i-1][j][0])
    return arr[i-1][j]

def color2bw(val):
    if(val < 100):
        return POS
    else:
        return NEG



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    global move
    pub = rospy.Publisher('batter', Vector3, queue_size=10)
    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        global send_x, send_y 

        if(Vision):
            v = Vector3(send_x, send_y, 0)
            pub.publish(v)
        rate.sleep()
    
def listener():
    #rospy.Subscriber('/usb_cam/camera_info', CameraInfo, camera_info_callback)
    rospy.Subscriber('/usb_cam/image_raw', Image, image_raw_callback)

def loop():
    rospy.init_node('tracker', anonymous=True)
    listener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__ == '__main__':
    loop()

