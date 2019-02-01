'''
State Encapsulation for YuMi robot
Author: Jacky
'''
import numpy as np


class YuMiState:
    """ Object that encapsulates a yumi arm joint angle configuration.
    """

    NUM_JOINTS = 7
    NAME = "YuMi"

    def __init__(self, vals=[0] * NUM_JOINTS):
        self._joint_names = []
        for i, val in enumerate(vals):
            joint_name = 'joint{}'.format(i+1)
            _joint_name = '_' + joint_name
            self._joint_names.append(joint_name)
            # Add YuMiState._joint1 and similars, containing the value in deg.
            setattr(self, _joint_name, val)
            # Add YuMiState.joint1 and similars as property.
            setattr(YuMiState, joint_name,
                    getx=lambda self: getattr(self, _joint_name),
                    setx=lambda self, val: setattr(self, _joint_name, val))

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
