# YuMi Python Interface
## AutoLab, UC Berkeley

This package provides a python interface for control and communication with ABB's Yumi. Currently the interface is still undergoing development, so changes will occur to this repo. For best results please be on the newest commit of the master branch before installing and using.

Full documentation can be found at [https://berkeleyautomation.github.io/yumipy/](https://berkeleyautomation.github.io/yumipy/)

### Installation
See the instructions [here](https://berkeleyautomation.github.io/yumipy/install/install.html)

### Usage
Simple example to import and use the YuMi interface (make sure the YuMi is in Auto mode and has the server running):
```python
from yumipy import YuMiRobot
# starting the robot interface
y = YuMiRobot()
# getting the current pose of the right end effector
pose = y.right.get_pose()
# move right arm forward by 5cm using goto_pose
pose.translation[0] += 0.05
y.right.goto_pose(pose)
# move right arm back by 5cm using move delta
y.right.goto_pose_delta((-0.05,0,0))
```

The control and RAPID server code is inspired by the [open-abb-driver](https://github.com/robotics/open_abb).
