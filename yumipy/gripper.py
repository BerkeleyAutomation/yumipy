import numpy as np

class Gripper(object):
    """A robot gripper.

    Parameters
    ----------
    name : str
        Unique identifier for the gripper.
    geometry : :obj:`trimesh.Trimesh`
        Triangular mesh representation of the gripper's geometry.
        The z-axis of the gripper's frame of reference should point
        along its approach direction for grasps.
    tooltips : list of :obj:`Tooltip`
        A list of tooltips associated with this gripper.
    tooltip_poses : list of :obj:`autolab_core.RigidTransform`
        A list of poses for each tooltip relative to the gripper frame.
    collision_geometry : :obj:`trimesh.Trimesh`, optional
        Simplified geometry for faster collision checking.
    """
    def __init__(self, name, geometry, tooltips, tooltip_poses, collision_geometry=None):
        self.name = name
        self.geometry = geometry
        self.tooltips = tooltips
        self.tooltip_poses = tooltip_poses
        self.collision_geometry = collision_geometry

        # Rename tooltips to unique names as needed
        names = {}
        for t in tooltips:
            if not t.name in names:
                names[t.name] = [t]
            else:
                names[t.name].append(t)
        for name in names:
            num_dups = len(names[name])
            if num_dups > 1:
                for i, t in enumerate(names[name]):
                    t.name = '{}_{}'.format(t.name, i)

    def approach(self, pose=None):
        """Get the grasp approach in world frame, which is a vector
        pointing out of the gripper's wrist towards the grasp center.

        Note
        ----
        Override this as needed in gripper subclasses.

        Parameters
        ----------
        pose : :obj:`autolab_core.RigidTransform`
            The gripper pose. If None, assume identity.

        Returns
        -------
        approach : (3,) float
            The grasp approach in world frame.
        """
        # Gripper approaches along its own z axis
        if pose is None:
            R, t = np.eye(3), np.zeros(3)
        else:
            R, t = pose.rotation, pose.translation
        return R[:,2]

    def find_contacts(self, objects, pose, mask=None):
        """Compute a list of valid contacts induced on the set of objects
        when the gripper is in the given pose.

        Parameters
        ----------
        objects : list of :obj:`GraspableObject`
            A set of graspable objects in the world.
        pose : :obj:`autolab_core.RigidTransform`
            The pose of this gripper in the world.
        mask : (n,) bool
            A boolean mask for which tooltips to activate.
            If not provided, all tooltips are activated.

        Returns
        -------
        list of :obj:`Contact`
            All valid contacts induced by the gripper.
        """
        all_contacts = self.find_all_contacts(objects, pose)
        contacts = [c for c in all_contacts if c.is_valid]
        return contacts

    def find_all_contacts(self, objects, pose, mask=None):
        """Compute a list of all contacts induced on the set of objects
        when the gripper is in the given pose.

        Parameters
        ----------
        objects : list of :obj:`GraspableObject`
            A set of graspable objects in the world.
        pose : :obj:`autolab_core.RigidTransform`
            The pose of this gripper in the world.
        mask : (n,) bool
            A boolean mask for which tooltips to activate.
            If not provided, all tooltips are activated.

        Returns
        -------
        list of :obj:`Contact`
            All contacts induced by the gripper.
        """
        contacts = []
        if mask is None:
            mask = np.ones(len(self.tooltips), dtype=np.bool)
        for tooltip, tooltip_pose, activated in zip(self.tooltips, self.tooltip_poses, mask):
            if not activated:
                continue
            tt_contacts = tooltip.find_contacts(objects, pose.dot(tooltip_pose))
            if len(tt_contacts) > 0:
                contacts += tt_contacts
        return contacts
