"""
Setup of YuMiPy  codebase
Author: Jacky Liang
"""
from setuptools import setup

setup(name='yumipy',
      version='0.1.dev0',
      description='YuMi Python Interface by Berkeley AutoLab',
      author='Jacky Liang, Aimee Goncalves, Jeff Mahler',
      author_email='jackyliang@berkeley.edu',
      package_dir = {'': '.'},
      packages=['yumipy'],
      #test_suite='test'
     )

