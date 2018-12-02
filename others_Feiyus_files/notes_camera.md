
reference:
https://github.com/Laurenhut/ME495-final-project

## open camera

ls -ltrh /dev/video*

sudo apt-get install ros-melodic-uvc-camera

1. <!-- rosrun usb_cam usb_cam_node _video_device:=/dev/ttyACM0 -->
rosrun usb_cam usb_cam_node _video_device:=/dev/video0

2. rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv _camera_name:=tracker_camera

3. rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=uvc _camera_name:=tracker_camera
   

## calibrate:
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/usb_cam/image_raw camera:=/usb_cam


## Geometry/Math
http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Pose.html
geometry_msgs/Pose.msg

## tag tracking
https://github.com/ablarry91/ros-tag-tracking

if bug: cv::Scaler

download:
https://github.com/ros-perception/ar_track_alvar

$ roslaunch tag_tracking ar_track.launch
$ rosrun rviz rviz

## open camera

* list camera
rosrun baxter_tools camera_control.py -l

* close right hand camera
rosrun baxter_tools camera_control.py -c right_hand_camera

* open head camera
rosrun baxter_tools camera_control.py -o head_camera -r 1280x800

* open left-hand camera
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800


* display camera
rosrun image_view image_view image:=/cameras/head_camera/image



