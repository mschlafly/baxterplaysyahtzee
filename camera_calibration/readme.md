
This is the folder for calibrating Baxter's head camera.
(Because we can't get its camera_info from it ros topic)

The calibration result is camera_parameters.txt (or xxx_camera_parameters.pkl, which can be read in by Python using pickle)

Two Python scripts:
1. calibrate_camera.py
2. undistort_all_images.py


