"""
Motion planner for the YuMi
Author: Jeff Mahler
"""
import IPython
import logging
import numpy as np
try:
    import geometry_msgs
    import moveit_commander
    import moveit_msgs
    import rospy
except ImportError:
    logging.error("Unable to load ROS! Will not be able to use client-side motion planner!")

from autolab_core import RigidTransform
from .yumi_state import YuMiState
from .yumi_trajectory import YuMiTrajectory
from .yumi_constants import YuMiConstants as ymc

class YuMiMotionPlanner:
    """ Client-side motion planner for the ABB YuMi, based on MoveIt!
    """
    
    def __init__(self, arm, ee_link_name, planner='RRTstar',
                 goal_pos_tol=0.001,
                 planning_time=0.1,
                 eef_delta=0.01,
                 jump_thresh=0.0):
        self._arm = 'right_arm'
        if arm.lower() == 'left':
            self._arm = 'left_arm'
        self._goal_pos_tol = goal_pos_tol
        self._planning_time = planning_time
        self._ee_link=ee_link_name

        self._init_planning_interface()
        self._update_planning_params()
        self.set_planner(planner)

    def _init_planning_interface(self):
        """ Initializes the MoveIn planning interface """
        self._robot_comm = moveit_commander.RobotCommander()
        self._planning_group = moveit_commander.MoveGroupCommander(self._arm)
        self._planning_group.set_pose_reference_frame(ymc.MOVEIT_PLANNING_REFERENCE_FRAME)

    def _update_planning_params(self):
        """ Updates the parameters of the planner """
        self._planning_group.set_goal_position_tolerance(self._goal_pos_tol)
        self._planning_group.set_planning_time(self._planning_time)
        self._planning_group.set_end_effector_link(self._ee_link)

    def set_planner(self, planner):
        """ Sets the motion planner for the YuMi """
        if planner not in ymc.MOVEIT_PLANNER_IDS.keys():
            raise ValueError('Planner %s not supported' %(planner))
        self._planner_id = ymc.MOVEIT_PLANNER_IDS[planner]
        self._planning_group.set_planner_id(self._planner_id)
        
    @property
    def goal_position_tolerance(self):
        return self._goal_pos_tol

    @property
    def planning_time(self):
        return self._planning_time

    @property
    def planner(self):
        return self._planner

    @property
    def planner(self):
        return self._planner

    @goal_position_tolerance.setter
    def goal_position_tolerance(self, tol):
        self._goal_pos_tol = tol
        self._update_planning_params()

    @planning_time.setter
    def planning_time(self, planning_time):
        self._planning_time = planning_time
        self._update_planning_params()

    @planner.setter
    def planner(self, planner):
        self.set_planner(planner)

    def plan_waypoints(self,start_state,waypoints,eef_delta=0.01, jump_thresh=0.00):
        #returns a plan from the current state to each of the waypoints with the 
        #end effector pose linearly interpolated
        if not isinstance(start_state, YuMiState):
            raise ValueError('Start state must be specified')

        # check valid pose types
        if not isinstance(waypoints[0], RigidTransform):
            raise ValueError('Start and goal poses must be specified as RigidTransformations')
        # check valid frames
        if waypoints[0].from_frame != 'gripper' or (waypoints[0].to_frame != 'world' and waypoints[0].to_frame != 'base'):
            raise ValueError('Start and goal poses must be from frame \'gripper\' to frame \{\'world\', \'base\'\}')
            
        # set start state of planner
        start_state_msg = moveit_msgs.msg.RobotState()
        start_state_msg.joint_state.name = ['yumi_joint_%d_r' %i for i in range(1,8)]
        if self._arm == 'left_arm':
            start_state_msg.joint_state.name = ['yumi_joint_%d_l' %i for i in range(1,8)]
        start_state_msg.joint_state.position = start_state.in_radians
        self._planning_group.set_start_state(start_state_msg)
     
        # convert poses to the planner's reference frame
        # get waypoints
        pose_traj = [w*ymc.T_GRIPPER_HAND for w in waypoints]
        waypoints = [t.pose_msg for t in pose_traj]

        # plan plath
        plan, fraction = self._planning_group.compute_cartesian_path(waypoints, eef_delta, jump_thresh)

        if fraction >= 0.0 and fraction < 1.0:
            logging.warning('Failed to plan full path.')
        if fraction < 0.0:
            logging.warning('Error while planning path.')
            return None

        # convert from moveit joint traj in radians 
        joint_names = plan.joint_trajectory.joint_names
        joint_traj = []
        for t in plan.joint_trajectory.points:
            joints_and_names = zip(joint_names, t.positions)
            joints_and_names.sort(key = lambda x: x[0])
            joint_traj.append(YuMiState([np.rad2deg(x[1]) for x in joints_and_names]))

        return fraction,YuMiTrajectory(joint_traj)

    def plan_shortest_path(self, start_state, goal_pose):
        """
        Plans the shortest path in joint space between the start and goal pose from the initial joint configuration. The poses should be specified in the frame of reference of the end effector tip. Start state must correspond to the start pose - right now this is up to the user.
        
        Parameters
        ----------
        start_state : YuMiState
            initial state of the arm
        start_pose : RigidTransform
            initial pose of end effector tip
        goal_pose : RigidTransform
            goal pose of end effector tip

        Returns
        -------
        traj : YuMiTrajectory
            the planned trajectory to execute
        """
        # check valid state types
        if not isinstance(start_state, YuMiState):
            raise ValueError('Start state must be specified')

        # check valid pose types
        if not isinstance(goal_pose, RigidTransform):
            raise ValueError('Start and goal poses must be specified as RigidTransformations')

        # check valid frames
        if goal_pose.from_frame != 'gripper' or (goal_pose.to_frame != 'world' and goal_pose.to_frame != 'base'):
            raise ValueError('Start and goal poses must be from frame \'gripper\' to frame \{\'world\', \'base\'\}')
            
        # set start state of planner
        start_state_msg = moveit_msgs.msg.RobotState()
        start_state_msg.joint_state.name = ['yumi_joint_%d_r' %i for i in range(1,8)]
        if self._arm == 'left_arm':
            start_state_msg.joint_state.name = ['yumi_joint_%d_l' %i for i in range(1,8)]
        start_state_msg.joint_state.position = start_state.in_radians
        self._planning_group.set_start_state(start_state_msg)
     
        # convert end pose to the planner's reference frame
        goal_pose_hand = goal_pose * ymc.T_GRIPPER_HAND

        # plan plath
        self._planning_group.set_pose_target(goal_pose_hand.pose_msg)
        plan = self._planning_group.plan()
        if len(plan.joint_trajectory.points) == 0:
            logging.warning('Failed to plan path.')
            return None

        # convert from moveit joint traj in radians 
        joint_names = plan.joint_trajectory.joint_names
        joint_traj = []
        for t in plan.joint_trajectory.points:
            joints_and_names = zip(joint_names, t.positions)
            joints_and_names.sort(key = lambda x: x[0])
            joint_traj.append(YuMiState([np.rad2deg(x[1]) for x in joints_and_names]))

        return YuMiTrajectory(joint_traj)
