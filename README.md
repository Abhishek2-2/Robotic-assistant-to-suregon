# Robotic-assistant-to-suregon
Robotic assistant to surgeon

This project aims to develop a solution to assist surgeons in understaffed hospitals by means of an intelligent robot arm.

The robotic arm to be developed would act as an additional hand to the doctor while performing any surgery by providing the requested surgical instrument on his/her request via speech command.

We simulate the problem statement in gazebo using a UR5 robotic arm and a kinect 3D camera.
For Image processing yolov5 is used to identify surgical instuments and is intergrated with ros using the darknet-ros package.
For Speech processing we use the ros-vosk package.
