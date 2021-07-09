import numpy as np

from autolab_core import RigidTransform, transformations

from .gripper import Gripper
from ...actions import Grasp3D

class ParallelJawGripper(Gripper):
    """A parallel-jaw robot gripper.

    This gripper has two finger-based tooltips that actuate linearly
    against each other.

    The gripper's frame should be defined so that the gripper-frame z-axis
    points out of the gripper's wrist towards its finger tips and is orthogonal
    to the tool mount point.

    The tooltip poses should be such that their origins are the positions of the
    inner pads of the fingers at maximum opening width and their z-axes point towards
    each other.

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
    max_width : float
        The max opening width of the gripper (computed automatically).
    """
    def __init__(self, name, geometry, tooltips, tooltip_poses, collision_geometry=None):
        super(ParallelJawGripper, self).__init__(name, geometry, tooltips, tooltip_poses,
                                                 collision_geometry)

        ep0, ep1 = self.endpoints()
        self.max_width = np.linalg.norm(ep1 - ep0)

    def axis(self, pose=None):
        """Get the grasp axis in world frame, which points
        from one fingertip to the other.

        Parameters
        ----------
        pose : :obj:`autolab_core.RigidTransform`
            The gripper pose. If None, assume identity.

        Returns
        -------
        axis : (3,) float
            The grasp axis in world frame.
        """
        ep0, ep1 = self.endpoints(pose)
        axis = ep1 - ep0
        axis = axis / np.linalg.norm(axis)
        return axis

    def endpoints(self, pose=None):
        """Get the grasp endpoints in world frame, which are the
        fingertip points at the gripper's maximum opening width.

        Parameters
        ----------
        pose : :obj:`autolab_core.RigidTransform`
            The gripper pose. If None, assume identity.

        Returns
        -------
        endpoints : (2,3) float
            The endpoints in world frame.
        """
        if pose is None:
            R, t = np.eye(3), np.zeros(3)
        else:
            R, t = pose.rotation, pose.translation
        ep0 = R.dot(self.tooltip_poses[0].translation) + t
        ep1 = R.dot(self.tooltip_poses[1].translation) + t
        return np.array([ep0, ep1])

    def center(self, pose=None):
        """Get the grasp center in world frame, which is halfway
        between the two fingertip points.

        Parameters
        ----------
        pose : :obj:`autolab_core.RigidTransform`
            The gripper pose. If None, assume identity.

        Returns
        -------
        center : (3,) float
            The grasp center in world frame.
        """
        ep0, ep1 = self.endpoints(pose)
        return (ep0 + ep1) / 2.0

    def grasp_from_center_axis(self, center, axis):
        """Compute a grasp or a set of grasps from
        world-frame centers and axes.

        Note
        ----
        The grasp poses created force the approach of the gripper
        to be as vertical as possible in world frame.

        Parameters
        ----------
        center : (3,) or (N,3) float
            A grasp center or list of centers in world frame.
        axis : (3,) or (N,3) float
            A grasp axis or list of axes in world frame.

        Returns
        -------
        grasp : :obj:`Grasp3D` or list of :obj:`Grasp3D`
            The associated grasps.
        """
        if center.ndim == 1:
            center = np.array([center])
            axis = np.array([axis])

        poses = self._poses_from_center_axis(center, axis)

        grasps = [Grasp3D(self, pose) for pose in poses]

        if len(grasps) == 1:
            return grasps[0]
        return grasps

    def _poses_from_center_axis(self, centers, axes):
        """Create a set of poses from a list of centers and axes.

        The approach of the generated grasp pose will be as close to vertical
        as possible.
        """
        # Compute grasp bases:
        #   - x axis points from finger 0 to finger 1,
        #   - z axis points along gripper approach,
        #   - y axis completes the basis
        down_z = np.array([0.0, 0.0, -1.0])
        xs = axes
        zs = down_z - np.dot(xs, down_z)[:,np.newaxis] * xs
        znorms = np.linalg.norm(zs, axis=1)
        zs[znorms == 0.0] = np.array([1.0, 0.0, 0.0])
        znorms[znorms == 0.0] = 1.0
        zs = (zs.T / znorms).T
        ys = np.cross(zs, xs)

        # Choose pose mappings:
        #   - x to provided axis
        #   - z to projection of -z (world) onto plane orthogonal to x
        #   - y to complete the basis
        Rs = np.empty((len(axes), 3, 3))
        Rs[:,:,0] = xs
        Rs[:,:,1] = ys
        Rs[:,:,2] = zs

        # Pre-rotate by amount around z axis to take actual grasp axis to x
        # axis.
        actual_axis = self.axis()
        rot_angle = np.arccos(actual_axis[0])
        if actual_axis[1] < 0.0:
            rot_angle = -rot_angle
        pR = transformations.rotation_matrix(rot_angle, np.array([0.0, 0.0, 1.0]))[:3,:3]
        Rs = np.matmul(Rs, pR)
        ts = centers - np.matmul(Rs, self.center())

        poses = []
        for i in range(len(Rs)):
            poses.append(RigidTransform(rotation=Rs[i], translation=ts[i],
                                        from_frame='gripper', to_frame='world'))
        return poses
