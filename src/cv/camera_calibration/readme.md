
This is the folder for calibrating Baxter's head camera.  
(We did this step because (a) we thought we needed Baxter's head camera, and (b) Baxter's "head_camera/camera_info" looks wrong -- the distortion coefs are all zero.)

The calibration result is camera_parameters.txt (or xxx_camera_parameters.pkl, which can be read in by Python using pickle)

Two Python scripts:
1. calibrate_camera.py
2. undistort_all_images.py


