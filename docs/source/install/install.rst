ROS Installation
~~~~~~~~~~~~~~~~
To use the yumi-over-ros functionality, yumipy must be installed as a catkin_package.
The `yumipy` package has been tested for ROS Jade and may not work in other versions.

1. Install AUTOLAB dependencies
"""""""""""""""""""""""""""""""
Install the `AUTOLAB autolab_core`_ module by following `the installation instructions`_.

.. _AUTOLAB autolab_core: https://github.com/BerkeleyAutomation/autolab_core
.. _the installation instructions: https://berkeleyautomation.github.io/autolab_core/install/install.html

2. Clone the repository
"""""""""""""""""""""""
First `create a catkin workspace`_ if you do not already have one.
Then clone the repository in your workspace ::

  $ cd {PATH_TO_YOUR_CATKIN_WORKSPACE}/src  
  $ git clone https://github.com/BerkeleyAutomation/yumipy.git

3. Build the catkin package
"""""""""""""""""""""""""""
Now build the package and re-source the ROS bash environment ::
  $ cd {PATH_TO_YOUR_CATKIN_WORKSPACE}
  $ catkin_make
  $ . devel/setup.bash

and yumipy will be active as a catkin package. Note that this DOES NOT persist across terminal sessions.

.. _create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Python-Only Installation
~~~~~~~~~~~~~~~~~~~~~~~~
The yumipy package can also be used without ROS.
Note that the `yumipy` module is known to work for Python 2.7 and has not been tested for Python 3.

1. Install AUTOLAB dependencies
"""""""""""""""""""""""""""""""
Install the `AUTOLAB autolab_core`_ module by following `the installation instructions`_.

.. _AUTOLAB autolab_core: https://github.com/BerkeleyAutomation/autolab_core
.. _the installation instructions: https://berkeleyautomation.github.io/autolab_core/install/install.html

2. Clone the repository
"""""""""""""""""""""""
You can clone or download our source code from `Github`_. ::

    $ git clone https://github.com/BerkeleyAutomation/yumipy.git

.. _Github: https://github.com/BerkeleyAutomation/yumipy

3. Run Python installation script
"""""""""""""""""""""""""""""""""
To install `yumipy` in your current Python environment, simply
change directories into the `yumipy` repository and run ::

    $ python setup.py install

or ::

    $ pip install -r requirements.txt

Alternatively, you can run ::

    $ pip install /path/to/yumipy

to install `yumipy` from anywhere.

RAPID Server Installation
~~~~~~~~~~~~~~~~~~~~~~~~~
To install the RAPID Server on the ABB YuMi:

1. Connect YuMi (XP23 Service Port) to your machine via an Ethernet cable
2. Open ABB Robot Studio > Controller Tab
3. Add Controller > One Click Connect
4. Switch YuMi to Manual Mode
5. In the controller tab, Request Write Access
6. Unzip RAPID/YuMi_Backup_Stable.zip
7. Backup down arrow > Restore from Backup > Select the unzipped folder
8. Proceed with restoring backup

Quick Start
~~~~~~~~~~~
First install yumipy is as either as a ROS catkin package or standalone in Python, and install the RAPID server. 
Turn the robot on and make sure that your computer is connected to the service port of the YuMi via Ethernet.

1. Start the RAPID Server
"""""""""""""""""""""""""
To run the RAPID server, through the FlexPendant:

1. Turn Motors On
2. Change to AUTO Mode
3. Set Program Pointer to Main
4. Click "Play" to run server

Note that if you use our provided backup, you can use the custom buttons on the
FlexPendant to do the following:

- Three bars - toggle motors on/off
- Two bars - toggle auto/manual mode
- One bar - set program pointer to main

You only need to run the RAPID server if you're using the YuMiArm or YuMiRobot
objects. You can run YuMiSubcriber anytime as long as the YuMi is on and connected
to your machine.

You can edit the server code in SERVER_LEFT and SERVER_RIGHT.mod in RobotStudio.
To edit the logger code that provide logging capabilities to the YuMiSubscriber,
you must change their respective tasks from semi-static to static, restart
the robot, make your edits, change them back to semi-static, then restart
the robot again.

2. Calibrate the Grippers
"""""""""""""""""""""""""
Make sure the grippers are calibrated before using the arms ::

  $ python tools/calibrate_grippers.py

3. Test the Installation
""""""""""""""""""""""""
To test your installation, run::

  $ python setup.py test

with the robot on and calibrated.

We highly recommend testing before using the module.

4. Start the Arms Service (ROS Installation Only)
"""""""""""""""""""""""""""""""""""""""""""""""""
After doing this, in order to run the local server, run ::
  
  $ rosrun yumipy yumi_arms.launch

This will start servers for the two arms.

After doing this, we can initialize a yumi remote interface:

.. code-block:: python

  from yumipy import YuMiRobot
  y = YuMiRobot(arm_type='remote')

5. Run a Python Example
"""""""""""""""""""""""
Now let's move the right arm 5cm forward and 5cm backward to the starting point.

First, create a YuMiRobot object that communicates over ROS:

.. code-block:: python

  from yumipy import YuMiRobot
  # start the robot interface
  y = YuMiRobot(arm_type='remote')

If you installed the Python-only version then you can spin up an arm as follows:

.. code-block:: python

  from yumipy import YuMiRobot
  # start the robot interface
  y = YuMiRobot()

Now move the arms!
.. code-block:: python
  
  # getting the current pose of the right end effector
  pose = y.right.get_pose()

  # move right arm forward by 5cm using goto_pose
  pose.translation[0] += 0.05
  y.right.goto_pose(pose)

  # move right arm back by 5cm using move delta
  y.right.goto_pose_delta((-0.05,0,0))

5. Run a Python Example


Dependencies
~~~~~~~~~~~~
The `yumipy` module depends on the Berkeley AutoLab's `autolab_core`_ module,
which can be installed using `pip install` on the source repo.

.. _autolab_core: https://github.com/BerkeleyAutomation/autolab_core

Any other dependencies will be installed automatically when `yumipy` is
installed with `pip`.

To use the remote yumi functionality, `ROS`_ is needed.

.. _ROS: http://wiki.ros.org/

This package was tested in ROS Jade. Other versions may or may not work.

Documentation
~~~~~~~~~~~~~

Building
""""""""
Building `yumipy`'s documentation requires a few extra dependencies --
specifically, `sphinx`_ and a few plugins.

.. _sphinx: http://www.sphinx-doc.org/en/1.4.8/

To install the dependencies required, simply run ::

    $ pip install -r docs_requirements.txt

Then, go to the `docs` directory and run `make` with the appropriate target.
For example, ::

    $ cd docs/
    $ make html

will generate a set of web pages. Any documentation files
generated in this manner can be found in `docs/build`.

Deploying
"""""""""
To deploy documentation to the Github Pages site for the repository,
simply push any changes to the documentation source to master
and then run ::

    $ . gh_deploy.sh

from the `docs` folder. This script will automatically checkout the
``gh-pages`` branch, build the documentation from source, and push it
to Github.

