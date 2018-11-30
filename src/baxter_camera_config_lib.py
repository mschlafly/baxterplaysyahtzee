import numpy as np

# transformations
def form_T(R,p):
    T=np.identity(4)
    T[0:3,0:3]=R
    T[0:3,3:4]=p[0:3,0:1]
    return T

def get_Rp_from_T(T):
    R=T[0:3,0:3]
    p=T[0:3,3:4]
    return (R,p)   


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
        self.distortion=[0.0, 0.0, 0.0, 0.0, 0.0] 
        
        # !!! This distoration params are definitly wrong.
        # !!! Collect data again, it should have distortion
        
        self.distortion=np.array(self.distortion)
        self.intrinsic_matrix_array_form=[410.0, 0.0, 640.0, 0.0, 410.0, 400.0, 0.0, 0.0, 1.0]
        self.intrinsic_matrix=np.reshape(self.intrinsic_matrix_array_form, (3,3))

        None

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
        self.distortion=[0.02909140543965944,-0.08374659479659467, -0.0010871751762388689, 0.002550696113092468, 0.03253240050176302]
        self.distortion=np.array(self.distortion)    

        self.intrinsic_matrix_array_form=[403.1498765559809, 0.0, 651.3100088479307/2,
                0.0, 403.9409358133219, 405.82095914289783/2,
                0.0, 0.0, 1.0]
        # self.intrinsic_matrix_array_form=[403.1498765559809, 0.0, 651.3100088479307, 0.0, 403.9409358133219, 405.82095914289783, 0.0, 0.0, 1.0]
        
        # !!! The intrinsic matrix parameters are wrong. 
        # !!! The above "/2" in "intrinsic_matrix_array_form" was manually added by me, not its original data.
        # !!! I should collect this info again.
        
        self.intrinsic_matrix=np.reshape(self.intrinsic_matrix_array_form, (3,3))

class BaxterCamera_RightHand(object):
    def __init__(self):
        pass
        # height: 400
        # width: 640
        # distortion_model: "plumb_bob"
        # D: [0.02909140543965944, -0.08374659479659467, -0.001087175176238868

