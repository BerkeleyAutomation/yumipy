'''
State Encapsulation for YuMi robot
Author: Jacky
'''
import numpy as np


class YuMiState(object):
    """Object that encapsulates a yumi arm joint angle configuration"""

    NUM_JOINTS = 7
    NAME = 'YuMi'

    def __init__(self, vals=[0] * NUM_JOINTS):
        """Construct a YuMiState from joint values in degrees"""
        if len(vals) != self.NUM_JOINTS:
            raise ValueError('A list of {} values must be given'.format(self.NUM_JOINTS))
        self._joint_names = []
        for i, val in enumerate(vals):
            joint_name = 'joint{}'.format(i + 1)
            _joint_name = '_' + joint_name
            self._joint_names.append(joint_name)
            # Add YuMiState._joint1 and similars, containing the value in deg.
            setattr(self, _joint_name, val)

    def __str__(self):
        return str(self.joints)

    def __repr__(self):
        return 'YuMiState({})'.format(self.joints)

    @property
    def in_radians(self):
        return [np.radians(t) for t in self.joints]

    @property
    def in_degrees(self):
        return self.joints

    @property
    def joint1(self):
        return self._joint1

    @joint1.setter
    def joint1(self, val):
        self._joint1 = val

    @property
    def joint2(self):
        return self._joint2

    @joint2.setter
    def joint2(self, val):
        self._joint2 = val

    @property
    def joint3(self):
        return self._joint3

    @joint3.setter
    def joint3(self, val):
        self._joint3 = val

    @property
    def joint4(self):
        return self._joint4

    @joint4.setter
    def joint4(self, val):
        self._joint4 = val

    @property
    def joint5(self):
        return self._joint5

    @joint5.setter
    def joint5(self, val):
        self._joint5 = val

    @property
    def joint6(self):
        return self._joint6

    @joint6.setter
    def joint6(self, val):
        self._joint6 = val

    @property
    def joint7(self):
        return self._joint7

    @joint7.setter
    def joint7(self, val):
        self._joint7 = val

    @property
    def joint_names(self):
        return self._joint_names

    @property
    def joints(self):
        return [getattr(self, jn) for jn in self.joint_names]

    def copy(self):
        return YuMiState(self.joints)

    def mirror(self):
        return YuMiState([
            -self.joint1,
            self.joint2,
            self.joint3,
            -self.joint4,
            self.joint5,
            self.joint6,
            -self.joint7,
        ])

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        return False

    def __ne__(self, other):
        if isinstance(other, self.__class__):
            return not self.__eq__(other)
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted(self.__dict__.items())))
