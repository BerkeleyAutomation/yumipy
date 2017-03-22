Installation Instructions
=========================

Dependencies
~~~~~~~~~~~~
The `yumipy` module depends on the Berkeley AutoLab's `core`_ module,
which can be installed using `pip install` on the source repo.

.. _core: https://github.com/BerkeleyAutomation/core

Any other dependencies will be installed automatically when `yumipy` is
installed with `pip`.

Cloning the Repository
~~~~~~~~~~~~~~~~~~~~~~
You can clone or download our source code from `Github`_. ::

    $ git clone https://github.com/BerkeleyAutomation/yumipy.git

.. _Github: https://github.com/BerkeleyAutomation/yumipy

Installation of YuMiPy
~~~~~~~~~~~~~~~~~~~~~~
To install `yumipy` in your current Python environment, simply
change directories into the `yumipy` repository and run ::

    $ pip install -e .

or ::

    $ pip install -r requirements.txt

Alternatively, you can run ::

    $ pip install /path/to/yumipy

to install `yumipy` from anywhere.

Installation of RAPID Server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To install the RAPID Server on the ABB YuMi:

1. Connect YuMi (XP23 Service Port) to your machine via an Ethernet cable
2. Open ABB Robot Studio > Controller Tab
3. Add Controller > One Click Connect
4. Switch YuMi to Manual Mode
5. In the controller tab, Request Write Access
6. Unzip RAPID/YuMi_Backup_Stable.zip
7. Backup down arrow > Restore from Backup > Select the unzipped folder
8. Proceed with restoring backup

Running RAPID Server
~~~~~~~~~~~~~~~~~~~~
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
