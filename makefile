send:
	scp vision.py nvidia@192.168.8.1:~/catkin_ws/src/color_tracking/src/vision.py
	scp control.py nvidia@192.168.8.1:~/catkin_ws/src/race/src/control.py

download:
	scp nvidia@192.168.8.1:~/catkin_ws/src/color_tracking/src/vision.py vision.py
	scp nvidia@192.168.8.1:~/catkin_ws/src/race/src/control.py control.py
