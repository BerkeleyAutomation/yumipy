.. yumipy documentation master file, created by
   sphinx-quickstart on Wed Oct 26 18:16:00 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Berkeley AutoLab yumipy Documentation
=========================================
Welcome to the documentation for the Berkeley AutoLab's `yumipy` module!
This module is a Python interface to commanding the `ABB YuMi robot`_.
The `yumipy` module depends directly on AutoLab's `autolab_core`_ module.

.. _ABB YuMi robot: http://new.abb.com/products/robotics/industrial-robots/yumi
.. _autolab_core: https://github.com/BerkeleyAutomation/autolab_core

This project is still under development.
For technical support please submit an issue under `Github Issues`_.
For other inquiries please contact Jeff Mahler (jmahler@berkeley.edu) and Jacky Liang (jackyliang@berkeley.edu).

.. _Github Issues: https://github.com/BerkeleyAutomation/yumipy/issues

Core Rename Update
~~~~~~~~~~~~~~~~~~
As of commit `f4be489`_ on June 18, 2017, yumipy depends on `autolab_core`_, a renamed version of core.
If this affects you (you have installed yumipy prior to June 18th, 2017), please do the following:

.. _f4be489: https://github.com/BerkeleyAutomation/yumipy/commit/f4be489f911a6f189a1f5784daefc0b0a07d5571
.. _autolab_core: https://github.com/BerkeleyAutomation/autolab_core

1. Pull the latest commit from core
2. Re-install using pip or setup.py following the instructions here
3. Pull the lastest commit from yumipy

.. toctree::
   :maxdepth: 2
   :caption: Installation Guide

   install/install.rst

.. toctree::
   :maxdepth: 2
   :caption: API Documentation
   :glob:

   api/*

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
