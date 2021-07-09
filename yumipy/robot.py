import abc
import six

@six.add_metaclass(abc.ABCMeta)
class Robot(object):
    """Abstract class for physical or simulated robots.

    Attributes
    ----------
    name : str
        Unique identifier for the robot.
    pose : :obj:`autolab_core.RigidTransform`
        Pose of the robot base with respect to the world coordinate system.
    grippers : list of :obj:`Gripper`
        List of available grippers on the robot.
    """

    def __init__(self, name, pose, grippers=None):
        self.name = name
        self.pose = pose
        self.grippers = grippers
        if self.grippers is None:
            self.grippers = []

    @abc.abstractmethod
    def execute(self, action):
        """Execute an action on the robot.

        Parameters
        ----------
        action : :obj:`Action`
            Action to execute.

        Returns
        -------
        success : bool
            Whether or not the action was executed successfully.
        metadata : dict
            Metadata about the execution.
        """
        pass


class SimRobot(Robot):
    """Abstract class for simulated robots, which require a physics engine.
    """

    @abc.abstractmethod
    def execute(self, action, physics_engine):
        """Execute an action with the robot in the given physics engine.

        Parameters
        ----------
        action : :obj:`Action`
            The action to execute.
        physics_engine : :obj:`PhysicsEngine`
            The physics engine to use.

        Returns
        -------
        success : bool
            Whether or not the action was executed successfully.
        metadata : dict
            Metadata about the execution.
        """
        pass
