import numpy as np

from autolab_core import RigidTransform, transformations

from .gripper import Gripper
from ...actions import Grasp3D

class SuctionCupGripper(Gripper):
    """A suction-cup gripper with a single cup that is fixed.

    The tooltip pose should be such that its origin is at the base of the center
    of the suction cup and its z-axis points out towards the object about the axis
    of symmetry.

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
        super(SuctionCupGripper, self).__init__(name, geometry, tooltips,
                                                tooltip_poses,
                                                collision_geometry)

    def tip(self, pose=None, idx=None):
        """Get the gripper tip in world frame, which is the point at
        the center of the first suction cup tooltip and in the plane of its edge.

        Parameters
        ----------
        pose : :obj:`autolab_core.RigidTransform`
            The gripper pose. If None, assume identity.

        Returns
        -------
        tip : (3,) float
            The grasp tip point in world frame.
        """
        if pose is None:
            R, t = np.eye(3), np.zeros(3)
        else:
            R, t = pose.rotation, pose.translation
        if idx is None:
            idx = 0
        tip = R.dot(self.tooltip_poses[idx].translation) + t
        return tip

    def approach(self, pose=None, idx=None):
        """Get the grasp approach in world frame, which is a vector
        pointing out of the first suction cup towards the target.

        Parameters
        ----------
        pose : :obj:`autolab_core.RigidTransform`
            The gripper pose. If None, assume identity.

        Returns
        -------
        approach : (3,) float
            The grasp approach in world frame.
        """
        if pose is None:
            R = np.eye(3)
        else:
            R = pose.rotation
        if idx is None:
            idx = 0
        return R.dot(self.tooltip_poses[idx].rotation[:,2])

    def grasp_from_tip_approach(self, tip, approach, idx=None):
        """Compute a grasp or a set of grasps from
        world-frame tips and approaches.

        Parameters
        ----------
        tip : (3,) or (N,3) float
            A grasp tip or list of tips in world frame.
        approach : (3,) or (N,3) float
            A grasp approach or list of approaches in world frame.

        Returns
        -------
        grasp : :obj:`Grasp3D` or list of :obj:`Grasp3D`
            The associated grasps.
        """
        if tip.ndim == 1:
            tip = np.array([tip])
            approach = np.array([approach])
            if idx is None:
                idx = 0

        poses = self._poses_from_tip_approach(tip, approach, idx)

        grasps = [Grasp3D(self, pose) for pose in poses]

        if len(grasps) == 1:
            return grasps[0]
        return grasps

    def _poses_from_tip_approach(self, tips, approaches, idx):
        """Create a set of poses from a list of tips and approaches.

        Choose gripper pose such that approach matches up with each approach
        """
        approach_g = self.approach(idx=idx)
        tip_g = self.tip(idx=idx)

        s_t = approach_g + approaches

        nonzero = np.sum(np.abs(s_t) > 1e-12, axis=1) > 0
        nonzero_rows = np.where(nonzero)[0]
        zero_rows = np.where(~nonzero)[0]

        Rs = np.empty((len(approaches), 3, 3))

        # For approaches that are close to -approach_g, substitute default
        # rotation matrix
        vperp = np.array([approach_g[1], -approach_g[0], 0.0])
        if np.linalg.norm(vperp) == 0.0:
            vperp = np.array([1.0, 0.0, 0.0])
        else:
            vperp = vperp / np.linalg.norm(vpert)
        R = transformations.rotation_matrix(np.pi, vperp)[:3,:3]

        if len(zero_rows) > 0:
            Rs[zero_rows] = R

        # For approaches that aren't bad, use formula
        if len(nonzero_rows) > 0:
            s = approach_g[np.newaxis,:] + approaches[nonzero_rows]
            Rs[nonzero_rows] = 2 * (np.matmul(s[:,:,np.newaxis], s[:,np.newaxis,:]) /
                            np.einsum('ij,ij->i', s, s)[:,np.newaxis,np.newaxis]) - np.eye(3)

        # Compute translations
        ts = tips - np.matmul(Rs, tip_g)

        # Create poses
        poses = []
        for i in range(len(Rs)):
            poses.append(RigidTransform(rotation=Rs[i], translation=ts[i],
                                        from_frame='gripper', to_frame='world'))
        return poses
