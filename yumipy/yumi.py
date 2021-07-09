from robot import Robot, SimRobot
from parallel_jaw_gripper import ParallelJawGripper
from suction_cup_gripper import SuctionCupGripper
from action import NoAction
from yumipy import YuMiRobot, YuMiKinematics, YuMiState
from yumipy import YuMiCommException, YuMiControlException
from yumipy import YuMiConstants as YMC


from abc import ABCMeta, abstractmethod
import cPickle as pkl
import datetime
from gym import spaces
from multiprocessing import Process
import numpy as np
import os
import time
from copy import deepcopy

from autolab_core import RigidTransform, Logger
import autolab_core.utils as utils

# Setup imports for physical robot
import moveit_commander
from moveit_msgs.msg import RobotState



# minimum rotation necessary to actually attempt execution
MIN_ROTATION_DEG = 1
MIN_TRANS_METERS = 0.01
POSITION_REACHED_TOL = 0.01
MM_TO_M = 1.0 / 1000
class NoAction(object):
    pass
class RobotExecutionResult(object):
    SUCCESS = 0
    KINEMATIC_ERROR = 1
    COMM_ERROR = 2

class PathNotFeasibleException(Exception):
    def __init__(self, start_state, goal_pose,
                 *args, **kwargs):
        self.start_state = start_state
        self.goal_pose = goal_pose
        Exception.__init__(self, *args, **kwargs)

class YuMiSim(SimRobot):

    def execute(self, action, physics_engine):
        pass

class YuMiDriver(object):

    def __init__(self, arm_type, unreachable_poses, reset_on_start, 
                 approach_dist, approach_height, drop_height, timeout,
                 standard_velocity, standard_zoning, approach_velocity,
                 approach_zoning, max_speed_dist_ratio,drop_mode, 
                 feasibility_check_mode, eef_step, jump_threshold, 
                 min_plan_fraction, traj_len, ik_timeout, left_config, 
                 right_config, debug=False):
        
        self.arm_type = arm_type
        self.unreachable_poses = unreachable_poses
        self.reset_on_start = reset_on_start
        self.approach_dist = approach_dist
        self.approach_height = approach_height
        self.drop_height = drop_height
        self.timeout = timeout
        self.standard_velocity = standard_velocity
        self.standard_zoning = standard_zoning
        self.approach_velocity = approach_velocity
        self.approach_zoning = approach_zoning
        self.max_speed_dist_ratio = max_speed_dist_ratio
        self.drop_mode = drop_mode
        self.feasibility_check_mode = feasibility_check_mode
        self.eef_step = eef_step
        self.jump_threshold = jump_threshold
        self.min_plan_fraction = min_plan_fraction
        self.traj_len = traj_len
        self.ik_timeout = ik_timeout
        self.left_grippers = left_config['grippers']
        left_min = np.array(left_config['kinematically_feasible_region']['min'])
        left_max = np.array(left_config['kinematically_feasible_region']['max'])
        self.left_feasibility_region = spaces.Box(left_min, left_max)
        self.right_grippers = right_config['grippers']
        right_min = np.array(right_config['kinematically_feasible_region']['min'])
        right_max = np.array(right_config['kinematically_feasible_region']['max'])
        self.right_feasibility_region = spaces.Box(right_min, right_max)
        self.debug = debug

class YuMi(Robot):

    """ Wrapper class for easy use of the YuMi robot through yumipy. """
    def __init__(self, name, pose, grippers, driver, vis_state_callback=None):

        super(YuMi, self).__init__(name, pose, grippers)
        
        # set up logger
        self._logger = Logger.get_logger(self.__class__.__name__)
        self._vis_state_callback = vis_state_callback
       
        # open data store
        self.driver = driver

        self.tools = {} 
        self.robot = None

        # start robot
        self.start()                          

    @property
    def left(self):
        return self.robot.left

    @property
    def right(self):
        return self.robot.right
    
    def arm(self, name):
        if name == 'left':
            return self.robot.left
        return self.robot.right
    
    def start(self):
        """ Connect to the robot and reset to the home pose. """
        # iteratively attempt to initialize the robot
        initialized = False
        self.robot = None
        while not initialized:
            try:
                # open robot
                self.robot = YuMiRobot(debug=self.driver.debug,
                                       arm_type=self.driver.arm_type)
                
                # reset the arm poses
                self.robot.set_z(self.driver.standard_zoning)
                self.robot.set_v(self.driver.standard_velocity)

                # reset the tools
                gripper_names = [g.name for g in self.grippers]
                for gripper in self.driver.left_grippers:
                    gripper_obj = self.grippers[gripper_names.index(gripper)]                    
                    if gripper == 'yumi_metal_spline':
                        self.tools['jaws'] = ParallelJawYuMiTool(
                            self.robot, self.pose, 'left', 
                            gripper_obj, self.driver)
                    elif gripper == 'yumi_suction':
                        self.tools['suction'] = SuctionYuMiTool(
                            self.robot, self.pose, 'left', 
                            gripper_obj, self.driver)
                
                for gripper in self.driver.right_grippers:
                    gripper_obj = self.grippers[gripper_names.index(gripper)]
                    if gripper == 'yumi_metal_spline':
                        self.tools['jaws'] = ParallelJawYuMiTool(
                            self.robot, self.pose, 'right', 
                            gripper_obj, self.driver)
                    elif gripper == 'yumi_suction':
                        self.tools['suction'] = SuctionYuMiTool(
                            self.robot, self.pose, 'right', 
                            gripper_obj, self.driver)

                for name,tool in self.tools.iteritems():
                    if self.driver.reset_on_start:
                        tool.reset()
                    if isinstance(tool, YuMiGraspingTool):
                        tool.open_gripper()
                
                # mark initialized
                initialized = True

            except YuMiControlException as e:
                # stop robot (collision probably occurred)
                self.stop()
                
                # log errors
                self._logger.warning('Failed to initialize YuMi: unreachable pose')
                self._logger.warning('Jog YuMi into a safe pose')
                human_input = raw_input('Hit [ENTER] when YuMi is ready')

            except YuMiCommException as e:
                self._logger.warning(str(e))

                # stop robot (collision probably occurred)
                self.stop()
                
                # log errors
                self._logger.warning('Failed to initialize YuMi: robot communication interface died')
                self._logger.warning('Check the YuMi hardware and look for errors on the FlexPendant')
                human_input = raw_input('Hit [ENTER] when YuMi is ready')
            
    def stop(self):
        """ Stop the robot. """
        if self.robot is not None:
            self.robot.stop()

    def reset(self):
        """ Reset the robot. """
        self.stop()
        self.start()

    def select_tool(self, action):
        """ Select the tool corresponding to the action. """
        tool = None
        if isinstance(action.gripper, ParallelJawGripper):
            tool = self.tools['jaws']
        elif isinstance(action.gripper, SuctionCupGripper):
            tool = self.tools['suction']
        else:
            raise ValueError('Gripper type %s not supported! Only ParallelJawGripper and SuctionCupGripper can be used.' %(type(action.gripper)))
        return tool

    def store_control_exception(self, tool, exception):
        """ Stores a control exception to disk. """
        if self.driver.unreachable_poses is not None and isinstance(exception, YuMiControlException):
            save_dir = os.path.join(self.driver.unreachable_poses, tool.arm_name)
            if not os.path.exists(save_dir):
                os.mkdir(save_dir)
            timestamp = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H:%M:%S')
            start_state_filename = os.path.join(save_dir, 'start_state_%s.pkl' %(timestamp))
            goal_pose_filename = os.path.join(save_dir, 'goal_pose_%s.pkl' %(timestamp))
            action_filename = os.path.join(save_dir,
                                           'action_%s.pkl' %(timestamp))
            pkl.dump(exception.start_state, open(start_state_filename, 'wb'))
            pkl.dump(exception.goal_pose, open(goal_pose_filename, 'wb'))
            pkl.dump(action, open(action_filename, 'wb'))            
                    
    def handle_control_exception(self, tool, exception):
        """ Handles control exceptions for a specified tool on the YuMi. """
        try:
            # store control exception
            self.store_control_exception(tool, exception)
            raise Exception

            # attempt to reset the tool
            tool.reset()
            T_robot_world = tool.arm.get_pose()
            goal_trans = YMC.R_BIN_HOME_POSE.translation
            if tool.arm_name == 'left':
                goal_trans = YMC.L_BIN_HOME_POSE.translation
            while np.linalg.norm(T_robot_world.translation - goal_trans) > POSITION_REACHED_TOL:
                time.sleep(0.01)
                T_robot_world = tool.arm.get_pose()
            self._logger.info('Reset succeeded! The error was likely an unreachable pose')
        except:
            # restart the control interface
            self.robot = None
            self._logger.warning('Failed to handle control exception: robot communication interface died')
            self._logger.warning('Check the YuMi hardware and look for errors on the FlexPendant')
            human_input = raw_input('Hit [ENTER] when YuMi is ready')
            self.start()
            return False
        return True
    
    def execute(self, action):
        """ Execute an action on the physical robot.

        Parameters
        ----------
        action : :obj:`Action`
            action to execute

        Returns
        -------
        :obj:`RobotExecutionResult`
            result of executing the action
        """
        if isinstance(action, NoAction):
            return RobotExecutionResult.SUCCESS
        
        # attempt to execute the action using the specified tool
        tool = self.select_tool(action)
        try:
            tool.execute(action)
        except PathNotFeasibleException as e:
            # handle path feasbility errors
            if self._vis_state_callback is not None:
                self._vis_state_callback('error_recovery')
            self._logger.warning('Failed to execute action: path not feasible')
            self._logger.warning(str(e))
            tool.reset()
            T_robot_world = tool.arm.get_pose()
            goal_trans = YMC.R_BIN_HOME_POSE.translation
            if tool.arm_name == 'left':
                goal_trans = YMC.L_BIN_HOME_POSE.translation
            while np.linalg.norm(T_robot_world.translation - goal_trans) > POSITION_REACHED_TOL:
                time.sleep(0.01)
                T_robot_world = tool.arm.get_pose()
            return RobotExecutionResult.KINEMATIC_ERROR
        except YuMiControlException as e:
            # handle path control exceptions (unreachable poses)
            if self._vis_state_callback is not None:
                self._vis_state_callback('error_recovery')
            self._logger.warning('Failed to execute action: unreachable pose')
            self._logger.warning(str(e))
            self.handle_control_exception(tool, e)
            return RobotExecutionResult.KINEMATIC_ERROR
        except YuMiCommException as e:
            # handle communication errors by restarting the control interface
            # these are usually due to collision and require human intervention
            if self._vis_state_callback is not None:
                self._vis_state_callback('error_recovery')
            self.robot = None
            self.store_control_exception(tool, e)
            self._logger.warning('Failed to execute action: robot communication interface died')
            self._logger.warning('Check the YuMi hardware and look for errors on the FlexPendant')
            human_input = raw_input('Hit [ENTER] when YuMi is ready')
            self.start()
            self._logger.warning(str(e))
            return RobotExecutionResult.COMM_ERROR
        return RobotExecutionResult.SUCCESS
        
    def finish(self, action):
        """ Executes finishing motions for a given action on the physical robot.

        Parameters
        ----------
        action : :obj:`Action`
            action to execute

        Returns
        -------
        bool
            True if the action was successfully executed
        """
        if isinstance(action, NoAction):
            return RobotExecutionResult.SUCCESS
    
        # attempt to finish the action using the specified tool
        tool = self.select_tool(action) 
        try:
            tool.finish()
        except PathNotFeasibleException as e:
            if self._vis_state_callback is not None:
                self._vis_state_callback('error_recovery')
            self._logger.warning('Failed to execute action: path not feasible')
            # self._logger.warning(str(e))
            tool.reset()
            T_robot_world = tool.arm.get_pose()
            goal_trans = YMC.R_BIN_HOME_POSE.translation
            if tool.arm_name == 'left':
                goal_trans = YMC.L_BIN_HOME_POSE.translation
            while np.linalg.norm(T_robot_world.translation - goal_trans) > POSITION_REACHED_TOL:
                time.sleep(0.01)
                T_robot_world = tool.arm.get_pose()
            return RobotExecutionResult.KINEMATIC_ERROR
        except YuMiControlException as e:
            if self._vis_state_callback is not None:
                self._vis_state_callback('error_recovery')
            self._logger.warning('Failed to execute action: unreachable pose')
            self._logger.warning(str(e))
            self.handle_control_exception(tool, e)
            return RobotExecutionResult.KINEMATIC_ERROR
        except YuMiCommException as e:
            if self._vis_state_callback is not None:
                self._vis_state_callback('error_recovery')
            # restart the control interface
            self.robot = None
            self.store_control_exception(tool, e)
            self._logger.warning('Failed to execute action: robot communication interface died')
            self._logger.warning('Check the YuMi hardware and look for errors on the FlexPendant')
            human_input = raw_input('Hit [ENTER] when YuMi is ready')
            self.start()
            self._logger.warning(str(e))
            return RobotExecutionResult.COMM_ERROR
        return RobotExecutionResult.SUCCESS

class YuMiTool(object):
    """ Abstract class for tools on the YuMi. """
    __metaclass__ = ABCMeta

    def __init__(self, robot, T_robot_world, 
                 arm_name, gripper, 
                 tool_pose, driver):

        # set up logger
        self._logger = Logger.get_logger(self.__class__.__name__)

        # set params
        self.tool_pose = tool_pose

        # load arm
        self.arm_name = arm_name
        self.T_robot_world = T_robot_world
        self.gripper = gripper
        self.driver = deepcopy(driver)

        # read move group
        self.ik_srv = None
        if not self.driver.debug:
            self.ik_srv = YuMiKinematics(self.arm_name+'_arm',
                                         ik_timeout=self.driver.ik_timeout)
            self.move_group = moveit_commander.MoveGroupCommander(self.arm_name+'_arm')

        # set home pose
        if self.arm_name == 'left':
            self.arm = robot.left
            self.kinematically_feasible_region = self.driver.left_feasibility_region
            self.home_pose = YMC.L_BIN_HOME_POSE
            self.home_state = YMC.L_BIN_HOME_STATE
            self.singularity_avoidance_state = YMC.L_KINEMATIC_AVOIDANCE_STATE
        else:
            self.arm = robot.right
            self.kinematically_feasible_region = self.driver.right_feasibility_region
            self.home_pose = YMC.R_BIN_HOME_POSE
            self.home_state = YMC.R_BIN_HOME_STATE
            self.singularity_avoidance_state = YMC.R_KINEMATIC_AVOIDANCE_STATE
        self.arm.set_tool(self.tool_pose)


    def is_feasible(self, start_state, goal_pose):
        """ Check for a feasible cartesian path from the start state to the goal pose. """
        if self.move_group is None and self.ik_srv is None:
            return True

        if self.driver.feasibility_check_mode == 'ik':
            return self.is_feasible_ik_check(start_state, goal_pose)
        return self.is_feasible_cartesian_path(start_state, goal_pose)
        
    def is_feasible_ik_check(self, start_state, goal_pose):
        """ Checks feasibility by checking for the existence of inverse kinematic
        solutions along a cartesian end-effector trajectory. """
        # read the latest poses
        cur_pose = self.arm.get_pose().as_frames('gripper', 'world')

        # check all poses along the cartesian trajectory
        traj = cur_pose.linear_trajectory_to(goal_pose, self.driver.traj_len)
        for i, T_gripper_world in enumerate(traj):
            pose = self.ik_srv.pose_stamped_from_pose(T_gripper_world.pose_msg)    
            res = self.ik_srv.get_ik(pose, avoid_collisions=True)
            kinematically_feasible = False
            try:
                if res.error_code.val == 1:
                    kinematically_feasible = True
            except:
                pass
            if not kinematically_feasible:
                return False
        return True

    def is_feasible_cartesian_path(self, start_state, goal_pose):
        """ Checks feasibility by checking for a valid cartesian path from the
        start state to the goal pose.
        Enforces smooth paths in joint space.
        """
        # plan a cartesian path
        _, fraction = self.plan_cartesian_path(start_state, goal_pose)

        # check the fraction complete
        if fraction < self.driver.min_plan_fraction:
            self._logger.info('Plan fraction: {}'.format(fraction))
            return False
        return True

    def plan_cartesian_path(self, start_state, goal_pose):
        """ Plans a cartesian path from the start state to the goal pose. """
        # set start state
        start_robot_state = RobotState()
        start_robot_state.joint_state.name = self.move_group.get_joints()[:-1]
        start_robot_state.joint_state.position = start_state.moveit
        self.move_group.set_start_state(start_robot_state)

        # plan a cartesian path to the goal pose
        waypoints = [goal_pose.pose_msg]
        plan, fraction = self.move_group.compute_cartesian_path(waypoints,
                                                                eef_step=self.driver.eef_step,
                                                                jump_threshold=self.driver.jump_threshold)        
        joint_space_plan = []
        for joint_angles in plan.joint_trajectory.points:
            joint_angles_rad = joint_angles.positions
            joint_angles_deg = np.rad2deg(np.array([joint_angles_rad[0],
                                                    joint_angles_rad[1],
                                                    joint_angles_rad[3],
                                                    joint_angles_rad[4],
                                                    joint_angles_rad[5],
                                                    joint_angles_rad[6],
                                                    joint_angles_rad[2]]))
            state = YuMiState(joint_angles_deg)
            joint_space_plan.append(state)
        return joint_space_plan, fraction
        
    def in_kinematically_feasible_region(self, translation):
        """ Checks whether or not a pose is in the kinematically
        feasible region for the arm. """
        return self.kinematically_feasible_region.contains(translation)
    
    def move_to(self, T_grasp_world, wait_for_res=True, fine=True, check_path_feasibility=False):
        """ Go to a pose via delta poses: first translate, then rotate. """
        
        # form delta pose
        cur_state = self.arm.get_state()
        T_cur_robot = self.arm.get_pose().as_frames('gripper', 'world')
        # T_des_robot = self.T_robot_world.inverse() * T_grasp_world
        T_des_cur = T_cur_robot.inverse() * T_grasp_world

        # translate in robot frame
        delta_t = T_cur_robot.rotation.dot(T_des_cur.translation)
        try:
            if fine or np.linalg.norm(delta_t) > MIN_TRANS_METERS:
                # check feasibility
                if check_path_feasibility:
                    T_trans = T_cur_robot.copy()
                    T_trans.translation = T_trans.translation + delta_t
                    if not self.is_feasible(cur_state, T_trans):
                        self._logger.info('Translation not feasible')
                        raise PathNotFeasibleException(start_state=cur_state,
                                                       goal_pose=T_trans)
                
                # go to pose
                self.arm.goto_pose_delta(delta_t,
                                         wait_for_res=wait_for_res)
        except YuMiControlException as e:
            goal_pose = RigidTransform(rotation=T_cur_robot.rotation,
                                       translation=T_cur_robot.translation,
                                       from_frame=T_cur_robot.from_frame,
                                       to_frame=T_cur_robot.to_frame)
            raise YuMiControlException(e.req_packet,
                                       e.res,
                                       start_state=cur_state,
                                       goal_pose=goal_pose)

        # rotate in robot frame
        delta_rx, delta_ry, delta_rz = T_des_cur.euler
        if delta_rz > 90.0:
            while np.abs(delta_rz) > 90.0:
                delta_rz = delta_rz - 180.0
                delta_ry = -delta_ry
                delta_rx = -delta_rx
        elif delta_rz < -90.0:
            while np.abs(delta_rz) > 90.0:
                delta_rz = delta_rz + 180.0
                delta_ry = -delta_ry
                delta_rx = -delta_rx            
                
        try:
            # delta rotations
            if fine or np.abs(delta_rz) > MIN_ROTATION_DEG:
                try:
                    # check feasibility
                    if check_path_feasibility:
                        cur_state = self.arm.get_state()
                        T_cur_robot = self.arm.get_pose().as_frames('gripper', 'world')
                        Rz = RigidTransform.z_axis_rotation(np.deg2rad(delta_rz))
                        delta_Rz = RigidTransform(rotation=Rz,
                                                  from_frame='gripper',
                                                  to_frame='gripper')
                        T_rz = T_cur_robot * delta_Rz
                        if not self.is_feasible(cur_state, T_rz):
                            self._logger.info('Z rotation not feasible')
                            raise PathNotFeasibleException(start_state=cur_state,
                                                           goal_pose=T_trans)
                                        
                    # go to pose    
                    self.arm.goto_pose_delta([0,0,0],
                                             rotation=[0,0,delta_rz],
                                             wait_for_res=wait_for_res)
                except (YuMiControlException, PathNotFeasibleException) as e:
                    # reverse rotation
                    if delta_rz < 0:
                        delta_rz += 180.0
                    else:
                        delta_rz -= 180.0                        
                    delta_ry = -delta_ry
                    delta_rx = -delta_rx

                    # check feasibility
                    if check_path_feasibility:
                        cur_state = self.arm.get_state()
                        Rz = RigidTransform.z_axis_rotation(np.deg2rad(delta_rz))
                        delta_Rz = RigidTransform(rotation=Rz,
                                                  from_frame='gripper',
                                                  to_frame='gripper')
                        T_rz = T_cur_robot * delta_Rz
                        if not self.is_feasible(cur_state, T_trans):
                            self._logger.info('Z rotation not feasible')
                            raise PathNotFeasibleException(start_state=cur_state,
                                                           goal_pose=T_trans)
                    
                    # go to pose
                    self.arm.goto_pose_delta([0,0,0],
                                             rotation=[0,0,delta_rz],
                                             wait_for_res=wait_for_res)
            if fine or np.abs(delta_ry) > MIN_ROTATION_DEG:
                # check feasibility
                if check_path_feasibility:
                    cur_state = self.arm.get_state()
                    T_cur_robot = self.arm.get_pose().as_frames('gripper', 'world')
                    Ry = RigidTransform.y_axis_rotation(np.deg2rad(delta_ry))
                    delta_Ry = RigidTransform(rotation=Ry,
                                              from_frame='gripper',
                                              to_frame='gripper')
                    T_ry = T_cur_robot * delta_Ry
                    if not self.is_feasible(cur_state, T_ry):
                        self._logger.info('Y rotation not feasible')
                        raise PathNotFeasibleException(start_state=cur_state,
                                                       goal_pose=T_trans)

                # go to pose
                self.arm.goto_pose_delta([0,0,0],
                                         rotation=[0,delta_ry,0],
                                         wait_for_res=wait_for_res)
            if fine or np.abs(delta_rx) > MIN_ROTATION_DEG:
                # check feasibility
                if check_path_feasibility:
                    cur_state = self.arm.get_state()
                    T_cur_robot = self.arm.get_pose().as_frames('gripper', 'world')
                    Rx = RigidTransform.x_axis_rotation(np.deg2rad(delta_rx))
                    delta_Rx = RigidTransform(rotation=Rx,
                                              from_frame='gripper',
                                              to_frame='gripper')
                    T_rx = T_cur_robot * delta_Rx
                    if not self.is_feasible(cur_state, T_rx):
                        self._logger.info('X rotation not feasible')
                        raise PathNotFeasibleException(start_state=cur_state,
                                                       goal_pose=T_trans)
                # go to pose
                self.arm.goto_pose_delta([0,0,0],
                                         rotation=[delta_rx,0,0],
                                         wait_for_res=wait_for_res)

        except YuMiControlException as e:
            # read state again
            cur_state = self.arm.get_state()
            raise YuMiControlException(e.req_packet,
                                       e.res,
                                       start_state=cur_state,
                                       goal_pose=T_des_robot)
            
    def reset(self):
        """ Reset the tool to the home pose. """
        # move arm to the kinematic avoidance pose
        if self.arm_name == 'left':
            self.arm.goto_state(YMC.L_KINEMATIC_AVOIDANCE_STATE,
                                wait_for_res=True)
        else:
            self.arm.goto_state(YMC.R_KINEMATIC_AVOIDANCE_STATE,
                                wait_for_res=True)
            
        # move to home pose
        self.arm.goto_state(self.home_state)
        
    @abstractmethod
    def execute(self, action):
        """ Executes an action on a physical YuMi robot. """
        pass

    @abstractmethod
    def finish(self):
        """ Performs finishing motions for the tool (after executing an action). """
        pass

class YuMiGraspingTool(YuMiTool):
    """ Abstract class for grasping with YuMi. """
    __metaclass__ = ABCMeta
    
    def __init__(self, robot, T_robot_world, 
                 arm_name, gripper, 
                 tool_pose, driver):

        self._gripper_start = 0
        
        YuMiTool.__init__(self, robot, T_robot_world,
                          arm_name, gripper, 
                          tool_pose, driver)
                
    def plan(self, action):
        """ Plan the sequence of poses for a grasp. """
        # read current pose
        T_start_world = self.arm.get_pose().as_frames('gripper', 'world')
        
        # read grasp pose
        T_grasp_world = action.pose.copy()

        # compute approach pose
        t_grasp_approach = np.array([0,0,self.driver.approach_dist])
        T_grasp_approach = RigidTransform(translation=t_grasp_approach,
                                          from_frame=T_grasp_world.from_frame,
                                          to_frame=T_grasp_world.from_frame)
        T_approach_world = T_grasp_world * T_grasp_approach.inverse()

        # compute pregrasp pose
        T_pregrasp_world = RigidTransform(rotation=T_start_world.rotation,
                                          translation=T_approach_world.translation,
                                          from_frame=T_grasp_world.from_frame,
                                          to_frame=T_grasp_world.to_frame)
        T_pregrasp_world.translation[2] = self.driver.approach_height

        # compute lift pose
        T_lift_world = RigidTransform(rotation=T_start_world.rotation,
                                      translation=T_approach_world.translation,
                                      from_frame=T_approach_world.from_frame,
                                      to_frame=T_approach_world.to_frame)
        T_lift_world.translation[2] = self.driver.approach_height       

        # decide whether or not to avoid singularities
        avoid_singularities = True
        if self.in_kinematically_feasible_region(T_grasp_world.translation):
            avoid_singularities = False
        return T_pregrasp_world, T_approach_world, T_grasp_world, T_lift_world, avoid_singularities
                
    def finish(self):
        # set tool
        self.arm.set_tool(self.tool_pose)
        
        # read current pose
        T_start_world = self.arm.get_pose().as_frames('gripper', 'world')

        # compute drop pose
        T_drop_world = T_start_world.copy()
        T_drop_world.translation[2] = self.driver.drop_height

        # move to poses in sequence
        self.arm.set_speed(YuMiRobot.get_v(self.driver.standard_velocity))
        self.move_to(T_drop_world)
        self.open_gripper()
        self._logger.debug('Time between grasp and drop: %.3f ' %(time.time() - self._gripper_start))
        self.arm.goto_state(self.home_state)

    def reset(self):
        YuMiTool.reset(self)
        self.open_gripper()
        
    @abstractmethod
    def open_gripper(self, wait_for_res=True):
        """ Open the gripper. """
        pass

    @abstractmethod
    def close_gripper(self, wait_for_res=True):
        """ Close the gripper. """
        pass

class ParallelJawYuMiTool(YuMiGraspingTool):
    """ Tool for parallel jaw grasping with the YuMi. """
    def __init__(self, robot, T_robot_world, 
                 arm_name, gripper, driver):
        
        # read tool pose 
        # tool_pose = YuMi.YMC.TCP_DEFAULT_GRIPPER
        tool_pose = RigidTransform()
        YuMiGraspingTool.__init__(self, robot, T_robot_world, 
                                  arm_name, gripper, 
                                  tool_pose, driver)
        # self.driver.drop_height += YMC.TCP_DEFAULT_GRIPPER.translation[2]

        # check valid gripper
        if not isinstance(gripper, ParallelJawGripper):
            raise ValueError('ParallelJawYuMiTool can only be instantiated with a parallel jaw gripper')

    @property
    def width(self):
        return self.arm.get_gripper_width()
        
    def open_gripper(self, wait_for_res=True):
        self.arm.open_gripper(wait_for_res=wait_for_res)

    def close_gripper(self, wait_for_res=True):
        self.arm.close_gripper(wait_for_res=wait_for_res)

    def execute(self, action):
        """ Go to the grasp pose, lift, and return to the home pose. """
        # set tool
        self.arm.set_tool(self.tool_pose)

        # plan sequence of poses
        T_pregrasp_world, T_approach_world, T_grasp_world, T_lift_world, avoid_singularities = self.plan(action)

        # open gripper
        self.open_gripper(wait_for_res=False)

        # move to the approach pose
        self.arm.set_speed(YuMiRobot.get_v(self.driver.standard_velocity))
        self.arm.set_zone(YuMiRobot.get_z(self.driver.standard_zoning))
        if avoid_singularities:
            self._logger.info('Action is in a dangerous region. Moving to singularity avoidance state before approach pose.')
            self.arm.goto_state(self.singularity_avoidance_state)
            self.move_to(T_approach_world, fine=False, check_path_feasibility=True)
        else:
            try:
                self.move_to(T_approach_world, fine=False, check_path_feasibility=True)
            except (YuMiControlException, PathNotFeasibleException):
                self._logger.info('Approach path not feasible. Moving to singularity avoidance state.')
                T_cur_world = self.arm.get_pose()
                if np.linalg.norm(T_cur_world.translation - T_approach_world.translation) > POSITION_REACHED_TOL:
                    self.arm.goto_state(self.singularity_avoidance_state)
                self.move_to(T_approach_world, fine=False)

        # move to the grasp
        self.arm.set_speed(YuMiRobot.get_v(self.driver.approach_velocity))
        self.arm.set_zone(YuMiRobot.get_z(self.driver.approach_zoning))
        self.move_to(T_grasp_world, fine=False)
        self._gripper_start = time.time()
        self.close_gripper()

        # move to the lift pose
        self.arm.set_speed(YuMiRobot.get_v(self.driver.standard_velocity))
        self.arm.set_zone(YuMiRobot.get_z(self.driver.standard_zoning))
        if avoid_singularities:
            self._logger.info('Action is in a dangerous region. Moving to singularity avoidance state before home pose.')
            self.move_to(T_approach_world, fine=False)
            self.arm.goto_state(self.singularity_avoidance_state)
            self.arm.goto_state(self.home_state)
        else:
            try:
                self.move_to(T_approach_world, fine=False, check_path_feasibility=True)
                self.move_to(T_lift_world, check_path_feasibility=True)
                if self.driver.drop_mode == 'container':
                    self.arm.goto_state(self.home_state)
            except (YuMiControlException, PathNotFeasibleException):
                self._logger.info('Home path not feasible. Moving to singularity avoidance state.')
                self.arm.goto_state(self.singularity_avoidance_state)
                self.arm.goto_state(self.home_state)
        
class SuctionYuMiTool(YuMiGraspingTool):
    """ Tool for suction grasping with the YuMi. """
    def __init__(self, robot, T_robot_world, 
                 arm_name, gripper, driver):
        
        # read tool pose
        # tool_pose = YMC.TCP_SUCTION_STIFF
        tool_pose = RigidTransform()
        YuMiGraspingTool.__init__(self, robot, T_robot_world, 
                                  arm_name, gripper, 
                                  tool_pose, driver)

        # check valid gripper
        if not isinstance(gripper, SuctionCupGripper):
            raise ValueError('SuctionYuMiTool can only be instantiated with a suction gripper')

        # open suction
        self.arm.init_suction()
        
    def open_gripper(self, wait_for_res=True):
        self.arm.suction_off()

    def close_gripper(self, wait_for_res=True):
        self.arm.suction_on()

    def set_safe_velocity(self, goal_pose):
        """ Sets the fastest possible velocity, capping high accelerations to minimize drops. """
        # read cur pose
        cur_pose = self.arm.get_pose()

        # determine velocity
        dist = np.linalg.norm(goal_pose.translation - cur_pose.translation)
        speed_dist_ratio = (self.driver.standard_velocity * MM_TO_M) / dist
        slow_factor = int(speed_dist_ratio / self.driver.max_speed_dist_ratio)
        velocity = self.driver.standard_velocity / 2**slow_factor
        self._logger.debug('Slow factor: %d' %(slow_factor))
        self._logger.debug('Velocity: %.3f' %(velocity))
        
        # set speed
        self.arm.set_speed(YuMiRobot.get_v(velocity))
        self.arm.set_zone(YuMiRobot.get_z(self.driver.standard_zoning))
        
    def execute(self, action):
        """ Go to the grasp pose, lift, and return to the home pose. """
        # set tool
        self.arm.set_tool(self.tool_pose)

        # plan sequence of poses
        T_pregrasp_world, T_approach_world, T_grasp_world, T_lift_world, avoid_singularities = self.plan(action)

        # open gripper
        self.open_gripper(wait_for_res=False)

        # move to the approach pose
        self.arm.set_speed(YuMiRobot.get_v(self.driver.standard_velocity))
        self.arm.set_zone(YuMiRobot.get_z(self.driver.standard_zoning))
        if avoid_singularities:
            self._logger.info('Action is in a dangerous region. Moving to singularity avoidance state before approach pose.')
            self.arm.goto_state(self.singularity_avoidance_state)
            self.move_to(T_approach_world, fine=False, check_path_feasibility=True)
        else:
            try:
                self.move_to(T_approach_world, fine=False, check_path_feasibility=True)
            except (YuMiControlException, PathNotFeasibleException) as e:
                self._logger.info('Approach path not feasible. Moving to singularity avoidance state.')
                T_cur_world = self.arm.get_pose()
                if np.linalg.norm(T_cur_world.translation - T_approach_world.translation) > POSITION_REACHED_TOL:
                    self.arm.goto_state(self.singularity_avoidance_state)
                self.move_to(T_approach_world, fine=False)

        # move to the grasp
        self.arm.set_speed(YuMiRobot.get_v(self.driver.approach_velocity))
        self.arm.set_zone(YuMiRobot.get_z(self.driver.approach_zoning))
        self._gripper_start = time.time()
        self.close_gripper()
        try: 
            self.move_to(T_grasp_world, fine=False)
        except (YuMiControlException, PathNotFeasibleException):
            self._logger.info('Grasp pose not feasible. Moving to singularity avoidance state.')
            self.arm.goto_state(self.singularity_avoidance_state)
            self.set_safe_velocity(self.home_pose)
            self.move_to(self.home_pose)


        # move to the lift pose
        if avoid_singularities:
            self._logger.info('Action is in a dangerous region. Moving to singularity avoidance state before home pose.')
            self.move_to(T_approach_world, fine=False)
            self.arm.goto_state(self.singularity_avoidance_state)
            self.set_safe_velocity(self.home_pose)
            self.move_to(self.home_pose)
        else:
            try:
                self.move_to(T_approach_world, fine=False, check_path_feasibility=True)
                self.move_to(T_lift_world, check_path_feasibility=True)
                self.set_safe_velocity(self.home_pose)
                if self.drop_mode == 'container':
                    self.move_to(self.home_pose, check_path_feasibility=True)
            except (YuMiControlException, PathNotFeasibleException):
                self._logger.info('Home path not feasible. Moving to singularity avoidance state.')
                self.arm.goto_state(self.singularity_avoidance_state)
                self.set_safe_velocity(self.home_pose)
                self.move_to(self.home_pose)        
