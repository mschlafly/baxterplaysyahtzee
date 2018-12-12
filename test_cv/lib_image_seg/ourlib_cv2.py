
#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from PIL import Image
import cv2
import numpy as np
from main import *
from random import random
import numpy

# def labeled_img = color_seg(img0):
# def res_ellipse, res_rects = find_squares(labeled_img):


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


def equalize_image(img):
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
    # convert the YUV image back to RGB format
    img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    return img_output


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

def find_squares(labeled_img):

    # output:
    res_rects=list()
    # each rect: [ [181, 124], [280, 62], [339, 155], [240, 218]]

    res_ellipse=list()
    # each ellipse: (center, axes, angle)       
    res_xs=list()
    res_ys=list()
    res_angles=list()

    # ------------------------ start ----------------
    rows, cols=labeled_img.shape

    n_labels=labeled_img.max()
    # print n_labels

    # Fit ellipse to determine the rectangles.
    ws_bincolor=rander_color(labeled_img)
    cnt=0

    for l in np.arange(1, n_labels + 1):
        mask_object=labeled_img == l
        ellipse, rect = find_square(mask_object)
        if ellipse is None:
            continue

        # return
        cnt+=1
        res_ellipse.append(ellipse) # (center, axes, angle)
        res_rects.append(rect)

        center=ellipse[0]
        angle=ellipse[2]
        x=center[0]
        y=center[1]
        
        res_xs.append(x)
        res_ys.append(y)
        res_angles.append(angle)

    return res_ellipse, res_rects, res_xs, res_ys, res_angles

def ellipse2xyangle(ellipse):
    # if ellipse is not None:
    center=ellipse[0]
    angle=ellipse[2]
    x=center[0]
    y=center[1]
    return x, y, angle
    # else:
        # return None, None, None

def find_square(mask_object,
        MIN_AREA=100,
        ERROR_HOW_SQUARE=0.2, # len/width
        ERROR_TOTAL_AREA=0.2, # total_area/(len*width)
        ERROR_TOTAL_AREA2=0.3, # total_area/(the logical "and" area of square and segmentation result)
    ):
    # output
    flag_find=False
    ellipse=None
    rect=None

    # start
    rows, cols=mask_object.shape[0:2]
    yx = numpy.dstack(numpy.nonzero(mask_object)).astype(numpy.int64)
    xy = numpy.roll(numpy.swapaxes(yx, 0, 1), 1, 2)

    cnt=0
    while cnt<1: # just for easier break
        cnt=cnt+1

        if len(xy) < MIN_AREA:  # Too small.
            break

        # get ellipse
        ellipse = cv2.fitEllipse(xy) # ((42.13095474243164, 124.8644027709961), (16.14134407043457, 36.2296142578125), 18.161659240722656)
        center, axes, angle = ellipse
        # axes: the axes whole length, not half
        # center (x to right, y to down)
        rect_area = axes[0] * axes[1]

        # get rectangular area
        rect = numpy.round(numpy.float64(
                    cv2.boxPoints(ellipse)
                )).astype(numpy.int64)
        # rect= [[ 29 140]
        # [ 40 105]
        # [ 55 110]
        # [ 44 145]]
        # print "rect=",rect
        # print "ellipse=",ellipse

        # C0: length == width
        if 1:
            if abs(axes[0] / axes[1] -1 )>=ERROR_HOW_SQUARE:
                break

        # C1: simple check total area
        if 1:
            if abs(rect_area / float(len(xy))-1) >= ERROR_TOTAL_AREA:
                break
            # method2: https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html
        
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
        return ellipse, rect
    else:
        return None, None


def refine_image_mask(img, rect, disextend=20):
    mask = np.zeros(img.shape[:2],np.uint8)

    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)

    # print rect
    minx=min(rect[:,0])-disextend
    maxwidthx=max(rect[:,0])-minx+disextend*2
    miny=min(rect[:,1])-disextend
    maxheighty=max(rect[:,1])-miny+disextend*2

    rect = (minx,miny,maxwidthx,maxheighty)
    cv2.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)

    mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    mask_3dim=mask2[:,:,np.newaxis]
    img = img*mask_3dim

    return mask_3dim[:,:,0]