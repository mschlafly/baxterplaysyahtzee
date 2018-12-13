#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from PIL import Image
import cv2
import numpy as np
from lib_main import *
from random import random
import numpy
from matplotlib import pyplot as plt


# def labeled_img = color_seg(img0):
# def res_ellipse, res_rects = find_squares(labeled_img):
def calc_distance(x1,y1,x2,y2):
    return sqrt( (x1-x2)**2+(y1-y2)**2)


def extract_rect(rect):
    center_x=np.mean(rect[:,0])
    center_y=np.mean(rect[:,1])
    
    radius_x=calc_distance(rect[0,0],rect[0,1],rect[1,0],rect[1,1])
    radius_y=calc_distance(rect[1,0],rect[1,1],rect[2,0],rect[2,1])

    angle=np.arctan2(rect[1,1]-rect[0,1],rect[1,0]-rect[0,0] )
    return (center_x, center_y, radius_x, radius_y, angle)

def extract_ellipse(ellipse):
    center=ellipse[0]
    angle=ellipse[2]
    x=center[0]
    y=center[1]
    return x, y, angle

def rander_color(labeled_img):
    (rows,cols)=labeled_img.shape
    random_color = lambda: (int(random()*255), int(random()*255), int(random()*255))
    colors = [random_color() for i in xrange(rows*cols)]

    color_image = np.zeros((rows,cols,3), np.uint8)
    for y in np.arange(rows):
        for x in np.arange(cols):
            color_image[y, x, :] = colors[labeled_img[y,x]]
    return color_image

def pil2cv(pil_image):
    open_cv_image = np.array(pil_image) 
    # Convert RGB to BGR 
    open_cv_image = open_cv_image[:, :, ::-1].copy() 
    return open_cv_image

def cv2pil(img):
    cv2_im = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    pil_im = Image.fromarray(cv2_im)
    return pil_im

def get_labeled_image(forest, width, height):
    random_color = lambda: (int(random()*255), int(random()*255), int(random()*255))
    colors = [random_color() for i in xrange(width*height)]

    label_dict={}
    cnt=0

    rows=width # 240, PIL is opposite to cv2
    cols=height # 320,
    labeled_img = np.zeros((rows, cols), np.uint8) 

    for y in xrange(height):
        for x in xrange(width):
            comp = forest.find(y * width + x)
            if comp not in label_dict:
                label_dict[comp]=cnt
                cnt+=1
            labeled_img[x, y] = label_dict[comp]
    print "label_dict, total labels = ", cnt
    return labeled_img

def equalize_image(img, filter_size=5):
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
    # convert the YUV image back to RGB format
    img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    
    kernel = np.ones((filter_size,filter_size),np.float32)/(filter_size*filter_size)
    img_output = cv2.filter2D(img_output,-1,kernel)

    return img_output

# Segment image, return the labeled_img
def color_seg(img0,  
        neighbor = 8,
        sigma = 0.5,
        K = 1000.0,
        min_size = 100):
    size = (img0.shape[1], img0.shape[0])
    
    img=pil2cv(img0)
    # img.show()

    try:
        neighbor = int(sys.argv[2])
        if neighbor != 4 and neighbor!= 8:
            print 'Invalid neighborhood choosed. The acceptable values are 4 or 8.'
            print 'Segmenting with 4-neighborhood...'
        sigma = float(sys.argv[1])
        K = float(sys.argv[3])
        min_size = int(sys.argv[4])
    except:
        pass

    grid = gaussian_grid(sigma)

    b,g,r = cv2.split(img)

    r = filter_image(r, grid)
    g = filter_image(g, grid)
    b = filter_image(b, grid)


    smooth = (r, g, b)
    diff = diff_rgb
    
    # print "before building graph"

    graph = build_graph(smooth, size[1], size[0], diff, neighbor == 8)
    forest = segment_graph(graph, size[0]*size[1], K, min_size, threshold)

    # seg_result_image = generate_image(forest, size[1], size[0])
    labeled_img = get_labeled_image(forest, size[1], size[0])

    # print 'Number of components: %d' % forest.num_sets

    return labeled_img

# input a image with multiple labels (mask s), find the squares inside them
def find_squares(labeled_img):

    # output:
    res_rects=list() # each rect: [ [181, 124], [280, 62], [339, 155], [240, 218]]

    # res_ellipse=list() # each ellipse: (center, axes, angle)       
    # res_xs=list()
    # res_ys=list()
    # res_angles=list()

    # ------------------------ start ----------------
    rows, cols=labeled_img.shape

    n_labels=labeled_img.max()
    # print n_labels

    # Fit ellipse to determine the rectangles.
    ws_bincolor=rander_color(labeled_img)
    cnt=0

    for l in np.arange(1, n_labels + 1):
        mask_object=labeled_img == l
        rect = find_square(mask_object)
        if rect is None:
            continue

        # return
        cnt+=1
        res_rects.append(rect)

        # res_ellipse.append(ellipse) # (center, axes, angle)
        # center=ellipse[0]
        # angle=ellipse[2]
        # x=center[0]
        # y=center[1]
        # res_xs.append(x)
        # res_ys.append(y)
        # res_angles.append(angle)

    # return res_ellipse, res_rects, res_xs, res_ys, res_angles
    return res_rects



# Input a mask, output the rectangular containing the mask.
# This also checks the property of the rectangular, to see if it's a real rect.
def find_square(mask_object,
        MIN_AREA=100,
        ERROR_HOW_SQUARE=0.5, # len/width
        ERROR_TOTAL_AREA=0.5, # total_area/(len*width)
        ERROR_TOTAL_AREA2=0.5, # total_area/(the logical "and" area of square and segmentation result)
    ):
    # output
    flag_find=False
    ellipse=None
    rect=None

    # start
    mask_object=mask_object.astype(np.uint8)*255
    kernel = np.ones((5,5),np.uint8)
    mask_object = cv2.erode(mask_object, kernel, iterations = 1)
    mask_object = cv2.dilate(mask_object, kernel, iterations = 1)

    rows, cols=mask_object.shape[0:2]
    yx = numpy.dstack(numpy.nonzero(mask_object)).astype(numpy.int64)
    xy = numpy.roll(numpy.swapaxes(yx, 0, 1), 1, 2)

    cnt=0
    while cnt<1: # just for easier break
        cnt=cnt+1

        if len(xy) < MIN_AREA:  # Too small.
            break

        # get rectangular area
        USE_METHOD_1=False
        if USE_METHOD_1:
            # get ellipse
            ellipse = cv2.fitEllipse(xy) # ((42.13095474243164, 124.8644027709961), (16.14134407043457, 36.2296142578125), 18.161659240722656)
            center, axes, angle = ellipse
            # axes: the axes whole length, not half
            # center (x to right, y to down)
            rect_area = radius_x * radius_y
            rect = numpy.round(numpy.float64(
                        cv2.boxPoints(ellipse)
                    )).astype(numpy.int64)
        else:
            # method2: https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html
            
            im2, contours, hierarchy = cv2.findContours(mask_object, 1, 2)
            rect = cv2.minAreaRect(contours[0])
            rect = cv2.boxPoints(rect)
            rect = np.int0(rect)

            (center_x, center_y, radius_x, radius_y, angle)=extract_rect(rect)
            rect_area = radius_x * radius_y

            # rect= [[ 29 140]
            # [ 40 105]
            # [ 55 110]
            # [ 44 145]]
            # print "rect=",rect
            # print "ellipse=",ellipse

        # C0: length == width
        if 1:
            if abs(radius_x / radius_y -1 )>=ERROR_HOW_SQUARE:
                break

        # C1: simple check total area
        if 1:
            if abs(rect_area / float(len(xy))-1) >= ERROR_TOTAL_AREA:
                break
        
        # C2: get the square and then check the area again
        if 1:
            mask_square=np.zeros((rows, cols), np.uint8) 
            cv2.fillPoly(mask_square, [rect], 255)
            mask_square = mask_square==255
            # print mask_square

            tmp = np.logical_and(mask_object, mask_square )
            same_area = sum(sum( tmp ))
            tmp = np.logical_xor(mask_object, mask_square )
            not_same_area=sum(sum( tmp ))
            score_of_square = (same_area-not_same_area)/rect_area
            # print "score_of_square",score_of_square

            if abs(score_of_square - 1) >= ERROR_TOTAL_AREA2:
                break
        flag_find=True

    if flag_find==True:
        return rect
    else:
        return None



# This calls: equalize_image, color_seg, find_squares, to find all the squares(objects)
def find_all_objects(img0,
        FLAG_DRAW_SQURAE_TO_IMAGE=False,
        IMAGE_RESIZE_SCALE=0.5,
    ):

    img=img0.copy()

    # --------------------------------------------------
    img = cv2.resize(img, (0,0), fx=IMAGE_RESIZE_SCALE, fy=IMAGE_RESIZE_SCALE)

    height, width, depth = img.shape

    # pre processing
    img=equalize_image(img)

    # Segment image, and then label them
    # print "start segmenting image"
    labeled_img=color_seg(img,neighbor = 8,
        sigma = 0.5,
        K = 800.0,
        min_size = 100)
    

    # Find the squares(object)
    res_rects = find_squares(labeled_img)

    # draw
    if FLAG_DRAW_SQURAE_TO_IMAGE:

        n_rects=len(res_rects)
        print "Find ", n_rects, " rectangulars !"

        colored_image= rander_color(labeled_img)
        cv2.imshow("colored label image", colored_image)
        cv2.waitKey(10)

        for i in range(n_rects):

            rect=res_rects[i]
            (x, y, radius_x, radius_y, angle)=extract_rect(rect)

            color=[0,0,255]
            cv2.drawContours(img, [rect], 0, color, 2)
            print "x={%.2f}, y={%.2f}, angle={%.2f}\n"%(x, y, angle)

        cv2.imshow("img with objects marked in rects", img)
        cv2.waitKey()
        cv2.destroyAllWindows()

    # Restore the scale of the rects
    res_rects = [rect/IMAGE_RESIZE_SCALE for rect in res_rects]
    return res_rects, labeled_img

# draw the colored_image with marked rectangulars of objects
def find_all_objects_then_draw(rects, labeled_img, IF_PRINT=True, IMAGE_RESIZE_SCALE=0.5):
    n_rects=len(rects)
    if IF_PRINT:
        print "Find ", n_rects, " rectangulars !"
    colored_image= rander_color(labeled_img)
    # cv2.imshow("colored label image", colored_image)
    # cv2.waitKey(1000)
    for i in range(n_rects):
        rect=rects[i]*IMAGE_RESIZE_SCALE
        rect=rect.astype(int)
        (x, y, radius_x, radius_y, angle)=extract_rect(rect)
        # print (x, y, radius_x, radius_y, angle)
        color=[0,0,255]
        # print "debug print rect: ", rect
        cv2.drawContours(colored_image, [rect], 0, color, 2)
        if IF_PRINT:
            print "x={%.2f}, y={%.2f}, angle={%.2f}\n"%(x, y, angle)
    return colored_image

# This function inputs the rectangular that object might be in, 
#   and then use "grabcut" algorithm to find the extact mask of the object
def refine_image_mask(img0, rect, disextend=20):
    
    img=img0.copy()

    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)

    # print rect
    # rect=np.array(rect)
    minx=min(rect[:,0])
    maxx=max(rect[:,0])
    maxwidthx=maxx-minx

    miny=min(rect[:,1])
    maxy=max(rect[:,1])
    maxheighty=maxy-miny

    rectxywidth = (minx,miny,maxwidthx,maxheighty)
    # print "rectxywidth",rectxywidth

    mask = np.zeros(img.shape[:2],np.uint8)
    mask[miny-disextend:maxy+disextend,minx-disextend:maxx+disextend]=1
    img[mask==0,:]=[0,0,0]
    # plt.imshow(img),plt.colorbar(),plt.show()

    cv2.grabCut(img,mask,rectxywidth,bgdModel,fgdModel,10,cv2.GC_INIT_WITH_RECT)
    # cv2.grabCut(img,mask,None,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_MASK)

    mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    mask_3dim=mask2[:,:,np.newaxis]
    img_res = img*mask_3dim

    return mask_3dim[:,:,0], img_res


# This function uses the middle of the image as the rectangular (assume object in the middle), 
#   and then calles function "refine_image_mask()" to obtain the object pos
def find_object_in_middle(img0, ratio_RADIUS_TO_CHECK=3, disextend=50):
    img=img0.copy()
    for i in range(2):
        if i==1:
            # print "trying HSV"
            img=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # --------------
        rows,cols=img.shape[:2]
        RADIUS_TO_CHECK=(rows/ratio_RADIUS_TO_CHECK)/2
        xmid=cols/2
        ymid=rows/2
        rect=[
            [xmid-RADIUS_TO_CHECK,ymid-RADIUS_TO_CHECK],
            [xmid+RADIUS_TO_CHECK,ymid-RADIUS_TO_CHECK],
            [xmid+RADIUS_TO_CHECK,ymid+RADIUS_TO_CHECK],
            [xmid-RADIUS_TO_CHECK,ymid+RADIUS_TO_CHECK],
        ]
        rect=np.array(rect)
        image_for_display_object=cv2.drawContours(img.copy(), [rect], 0, [0,1,0], 2)
        
        # plt.imshow(image_for_display_object),plt.colorbar(),plt.show()
        # print "the rect before grabcut, ", rect

        mask,img_res = refine_image_mask(img, rect, disextend=100)
        
        # print "the image after grabcut, "
        # plt.imshow(img_res),plt.colorbar(),plt.show()
        
        res_rect = find_square(mask)
        # print "the rect after grabcut and find_square, ", res_rect
        if res_rect is not None:
            break

    # If display or not
    if 0 and res_rect is not None:
        (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(res_rect)
        image_for_display_object=cv2.drawContours(img, [res_rect], 0, [0,0,1], 2)
        plt.imshow(image_for_display_object),plt.colorbar(),plt.show()
    return mask, res_rect

# get the median color
def get_color_median(img, mask, rect):
    # img=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    minx=min(rect[:,0])
    maxx=max(rect[:,0])
    miny=min(rect[:,1])
    maxy=max(rect[:,1])
    cb=list()
    cg=list()
    cr=list()
    for i in range(miny,maxy):
        for j in range(minx,maxx):
            # print mask[i,j]
            if mask[i,j]==0:
                continue
            cb.append(img[i,j,0])
            cg.append(img[i,j,1])
            cr.append(img[i,j,2])
    cr=np.array(cr)
    cb=np.array(cb)
    cg=np.array(cg)
    color=[np.median(cr),np.median(cg),np.median(cb)]

    # print "the color is ",color
    return color

def crop_image(img0, rect):
    img=img0.copy()
    minx=min(rect[:,0])
    maxx=max(rect[:,0])
    miny=min(rect[:,1])
    maxy=max(rect[:,1])
    return img[miny:maxy, minx:maxx]

def create_blob_detector(blob_min_area=12, 
                        blob_min_int=0.0, blob_max_int=1.0, blob_th_step=5):
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = blob_min_area
    params.maxArea = 30
    params.filterByCircularity = False
    params.filterByColor = False
    params.filterByConvexity = False
    params.filterByInertia = False
    # blob detection only works with "uint8" images.
    params.minThreshold = int(blob_min_int*255)
    params.maxThreshold = int(blob_max_int*255)
    params.thresholdStep = blob_th_step
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        return cv2.SimpleBlobDetector(params)
    else:
        return cv2.SimpleBlobDetector_create(params) 

def detect_dots(img, mask, rect):
    imgmask=img.copy()
    imgmask[mask==0]=0
    small = crop_image(imgmask, rect)
    # small = equalize_image(small, filter_size=2)
    detector = create_blob_detector()
    keypoints = detector.detect(small)

    IF_DRAW=False
    if IF_DRAW:
        im_with_keypoints = cv2.drawKeypoints(small, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        print "number of dots", len(keypoints)
    
    return len(keypoints)