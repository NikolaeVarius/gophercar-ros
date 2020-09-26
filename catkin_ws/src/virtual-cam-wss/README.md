# https://github.com/jgoppert/ros-virtual-cam


rosrun image_transport repuish compressed in:=/output/image_raw out:=/output/image
rosrun virtual_cam stream _device:=/dev/video1 _width:=640 _height:=480 _fourcc:=YV12 image:=/output/image
