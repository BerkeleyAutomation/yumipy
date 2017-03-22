Streaming Information from the Robot
====================================

YuMSubscriber
~~~~~~~~~~~~~
.. autoclass:: yumipy.YuMiSubscriber

YuMiSubcriber Example
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

  sub = YuMiSubcriber()
  sub.start()

  left_pose = sub.left.get_pose()
  right_pose = sub.right.get_pose()
  left_state = sub.left.get_state()
  right_state = sub.right.get_state()
  left_torque = sub.left.get_torque()
  right_torque = sub.right.get_torque()

  sub.stop()
