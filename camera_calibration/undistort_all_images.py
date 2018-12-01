

import numpy as np
import cv2
import glob

CAMERA_TYPES=["head","lefthand","righthand"]
CAMERA_TYPE=CAMERA_TYPES[0]
FOLDER_OF_ORIGINAL_IMAGES=CAMERA_TYPE+"_original_images"
FOLDER_TO_OUTPUT_UNDISTORTED_IMAGES=CAMERA_TYPE+"_undistorted_images"

# camera parameters (copied and pasted from the output of calibrate_camera.py)
# CHESSBOARD_SQUARE_SIZE=0.017
projection_error=2.1841066616
camera_intrinsics=np.array([
         [410.84409982,   0,         629.02144102],
         [  0,         410.77681442, 468.79624491],
         [  0,           0,           1        ]
         ])
# camera_intrinsics[0,0]*=CHESSBOARD_SQUARE_SIZE
# camera_intrinsics[1,1]*=CHESSBOARD_SQUARE_SIZE
camera_distortion_coefs=np.array([ 0.02039285, -0.04876712, -0.00017574, -0.00083992,  0.01135468])

# undistort
cnt_img=0
images = glob.glob(FOLDER_OF_ORIGINAL_IMAGES+'/*.jpg')
print(images)

for fname in images:
    cnt_img+=1
    print("processing {}/{} image ...".format(cnt_img, len(images)))
    img = cv2.imread(fname)
    
    # set param
    if cnt_img==1:    
        h, w=img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_intrinsics,camera_distortion_coefs,(w,h),1,(w,h))

    # undistort
    undistort_image = cv2.undistort(img, camera_intrinsics, camera_distortion_coefs, None, newcameramtx)
    
    # crop the image
    CROP_IMAGE=False
    if CROP_IMAGE:
        x,y,w,h = roi
        undistort_image = undistort_image[y:y+h, x:x+w]
        
    # resize
    # https://stackoverflow.com/questions/4195453/how-to-resize-an-image-with-opencv2-0-and-python2-6
    # undistort_image = cv2.resize(undistort_image, (0,0), fx=0.5, fy=0.5) # resize to half the size
    undistort_image = cv2.resize(undistort_image, (w,h)) 

    # put two images into one
    image_out = np.concatenate((img, undistort_image), axis=1)
    
    # show
    cv2.imshow('image_out',image_out)
    cv2.imwrite(FOLDER_TO_OUTPUT_UNDISTORTED_IMAGES+'/calib_'+fname,image_out)
    cv2.waitKey(1)