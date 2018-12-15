import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
from geometry_msgs.msg import Pose, Point
import cv2

# transformations
def form_T(R,p):
    T=np.identity(4)
    T[0:3,0:3]=R
    try:
        T[0:3,3:4]=p[0:3,0:1]
    except:
        T[0,3]=p[0]
        T[1,3]=p[1]
        T[2,3]=p[2]
    return T

def get_Rp_from_T(T):
    R=T[0:3,0:3]
    p=T[0:3,3:4]
    return (R,p)   


def Rp_to_pose(R,p):
    pose=Pose()

    R_vec, _ = cv2.Rodrigues(R)
    q=quaternion_from_euler(R_vec[0],R_vec[1],R_vec[2])    
    pose.orientation.w=q[0]
    pose.orientation.x=q[1]
    pose.orientation.y=q[2]
    pose.orientation.z=q[3]

    pose.position.x=p[0]
    pose.position.y=p[1]
    pose.position.z=p[2]
    return pose


# Baxter's camera
# reference:
# http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html

class BaxterCamera_Head(object):
    def __init__(self):
        # height: 800
        # width: 1280
        # distortion_model: "plumb_bob"
        # D: [0.0, 0.0, 0.0, 0.0, 0.0]
        # K: [410.0, 0.0, 640.0, 0.0, 410.0, 400.0, 0.0, 0.0, 1.0]
        # R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # P: [-410.0, -0.0, 639.0, 0.0, 0.0, -410.0, 399.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.height=800
        self.width=1280

        self.distortion=[ 0.02039285, -0.04876712, -0.00017574, -0.00083992,  0.01135468]
        self.distortion=np.array(self.distortion)

        # self.intrinsic_matrix_array_form=[410.0, 0.0, 640.0, 0.0, 410.0, 400.0, 0.0, 0.0, 1.0]
        # self.intrinsic_matrix=np.reshape(self.intrinsic_matrix_array_form, (3,3))

        self.intrinsic_matrix=np.array([
                [410.84409982,   0,         629.02144102],
                [  0,         410.77681442, 468.79624491],
                [  0,           0,           1        ]
                ])

class BaxterCamera_LeftHand(object):
    def __init__(self):
        # height: 400
        # width: 640
        # distortion_model: "plumb_bob"
        # D: [0.02909140543965944, -0.08374659479659467, -0.0010871751762388689, 0.002550696113092468, 0.03253240050176302]
        # K: [403.1498765559809, 0.0, 651.3100088479307, 0.0, 403.9409358133219, 405.82095914289783, 0.0, 0.0, 1.0]
        # R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # P: [403.1498765559809, 0.0, 331.3100088479307, 0.0, 0.0, 403.9409358133219, 205.82095914289783, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.height=400
        self.width=640

        BAXTER_ON_THE_RIGHT=True
        if BAXTER_ON_THE_RIGHT:
            # ----------------------- The baxter on the right -----------------
            self.distortion=[0.02909140543965944,-0.08374659479659467, -0.0010871751762388689, 0.002550696113092468, 0.03253240050176302]
            self.distortion=np.array(self.distortion)    

            self.intrinsic_matrix_array_form=[403.1498765559809, 0.0, 651.3100088479307/2,
                    0.0, 403.9409358133219, 405.82095914289783/2,
                    0.0, 0.0, 1.0]
            self.intrinsic_matrix=np.reshape(self.intrinsic_matrix_array_form, (3,3))
        else:
            # ----------------------- The baxter on the left -----------------
            self.distortion=[0.0184964633113, -0.0560618936713, 0.000269508536766, 0.000436460185745, 0.0147597067544]
            self.distortion=np.array(self.distortion)    

            self.intrinsic_matrix_array_form= [405.329601488, 0.0, 656.494624497/2,
                    0.0, 405.329601488, 425.656789027/2,
                    0.0, 0.0, 1.0]
            self.intrinsic_matrix=np.reshape(self.intrinsic_matrix_array_form, (3,3))

  

"""
not implemented
"""
class BaxterCamera_RightHand(object):
    def __init__(self):
        pass
        # height: 400
        # width: 640
        # distortion_model: "plumb_bob"
        # D: [0.02909140543965944, -0.08374659479659467, -0.001087175176238868
