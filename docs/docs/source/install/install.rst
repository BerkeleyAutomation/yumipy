Installation Instructions
=========================

Dependencies
~~~~~~~~~~~~
The `yumipy` module depends on the Berkeley AutoLab's `core`_ module,
which can be installed using `pip install` on the source repo.

.. _core: https://github.com/mmatl/core

Any other dependencies will be installed automatically when `yumipy` is
installed with `pip`.

To use the remote yumi functionality, `ROS`_ is needed.

.. _ROS: http://wiki.ros.org/

This package was tested in ROS Jade. Other versions may or may not work.

Cloning the Repository
~~~~~~~~~~~~~~~~~~~~~~
You can clone or download our source code from `Github`_. ::

    $ git clone git@github.com:jacky-liang/yumipy.git

.. _Github: https://github.com/jacky-liang/yumipy

Installation
~~~~~~~~~~~~
To install `yumipy` in your current Python environment, simply
change directories into the `yumipy` repository and run ::

    $ pip install -e .

or ::

    $ pip install -r requirements.txt

Alternatively, you can run ::

    $ pip install /path/to/yumipy

to install `yumipy` from anywhere.

Testing
~~~~~~~
To test your installation, run ::

    $ python setup.py test

with the robot on and calibrated.

We highly recommend testing before using the module.

Building Documentation
~~~~~~~~~~~~~~~~~~~~~~
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

Using ROS functionality
~~~~~~~~~~~~~~~~~~~~~~~
To use the yumi-over-ros functionality, first install yumipy as a catkin package

To do this, `create a catkin workspace`_, then clone yumipy into the src folder

.. _create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Now, in the catkin workspace directory, run ::

    catkin_make
    . devel/setup.bash

and yumipy will be active as a catkin package. Note that this DOES NOT persist across terminal sessions.


After doing this, in order to run the local server, run ::

    rosrun yumipy yumi_arms.launch

This will start servers for the two arms.

After doing this, we can initialize a yumi remote interface ::

    from yumipy import YuMiArmFactory
    yumi_arm_left = YuMiArmFactory.YuMiArm('remote', 'left')
    yumi_arm_right = YuMiArmFactory.YuMiArm('remote', 'right')

yumi_arm_left and yumi_arm right can be used the same way YuMiArm objects are (functions only, properties cannot be accessed)
