# YuMi Python Interface
## AutoLab, UC Berkeley

This package provides a python interface for control and communication with ABB's Yumi. Currently the interface is still undergoing development, so changes will occur to this repo. For best results please be on the newest commit of the master branch before installing and using.

Full documentation can be found at [https://berkeleyautomation.github.io/yumipy/](https://berkeleyautomation.github.io/yumipy/)

UPDATE: As of commit f4be489 on June 18, 2017, `yumipy` depends on [`autolab_core`](https://berkeleyautomation.github.io/autolab_core/), a renamed version of `core`.
If this affects you (you have installed yumipy before June 18th, 2017), please do the following:
* Pull the latest commit from `core`
* Re-install using pip or setup.py following the instructions [here](https://berkeleyautomation.github.io/autolab_core/install/install.html)
* Pull the lastest commit from `yumipy`

### Installation
Step 1: Install the alan YuMi python interface on the client computer that will communicate with the YuMi:
```sh
$ python setup.py develop
```
Step 2: Upload SERVER_LEFT.mod and SERVER_RIGHT.mod under `src/alan/control/` to the left and right arms of YuMi through RobotStudio.
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

### Support
Please request for support and report issues with errors and bugs on the repository Issues tab - that is the best way for us to respond.
For other inquiries please contact Jeff Mahler at jmahler@berkeley.edu and Jacky Liang at jackyliang@berkeley.edu
