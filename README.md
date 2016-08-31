# wholebody_ik
A library for whole-body inverse kinematics for the humanoid robot Walk-Man.

====
to run com_ik_test

5 terminals:

1 - roscore
2 - yarpserver --write
3 - rviz (add RobotModel display, TF display (e_0 frame is the target) and set fixed_frame=l_sole)
4 - roslaunch bigman_urdf bigman.launch
5 - com_ik_test