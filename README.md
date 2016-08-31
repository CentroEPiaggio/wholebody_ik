# wholebody_ik
A library for whole-body inverse kinematics for the humanoid robot Walk-Man.

to run com_ik_test
---

5 terminals:

- roscore
- yarpserver --write
- rviz (add RobotModel display, TF display (e_0 frame is the target) and set fixed_frame=l_sole)
- roslaunch bigman_urdf bigman.launch
- com_ik_test
