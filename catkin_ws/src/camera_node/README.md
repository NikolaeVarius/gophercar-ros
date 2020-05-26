# CSI-Camera
Based on https://github.com/JetsonHacksNano/CSI-Camera


## To Start Camera
`gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! nvvidconv flip-method=2 ! nvoverlaysink`

