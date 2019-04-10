
Kalman filter 

Currently implemented a Kalman filter based on the standard equations found https://en.wikipedia.org/wiki/Kalman_filter. 

In the file kalman_config_param.yaml you will find all tuneable parameters, making it easy to change parameters when running the program. 


The node is launched by the kalman.launch file.

This will make the node subscribe to the topic: "mavros/mocap/pose" (position measurement source), and publish to topic "/mavros/vision_pose/pose". If different topics are to be used, these has to be changed in the main.cpp (line 19) and Kalman.cpp (line 13), respectively.
