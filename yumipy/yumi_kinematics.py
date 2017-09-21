#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
import geometry_msgs.msg

"""
    Important info: To use this module, you need to have ROS installed, and
    the yumi_moveit_config downloaded.  Before using this module, you must 
    start the move_group using the following commands (in their own terminals):
    roscore
    roslaunch yumi_moveit_config real_demo.launch
"""
class YumiKinematics(object):
    def __init__(self, group, ik_timeout=1.0, ik_attempts=0,
                 avoid_collisions=False):
        """
        A class to do IK calls thru the MoveIt!'s /compute_ik service.
        :param str group: MoveIt! group name
        :param float ik_timeout: default timeout for IK
        :param int ik_attempts: default number of attempts
        :param bool avoid_collisions: if to ask for IKs that take
        into account collisions
        """
        rospy.loginfo("Initalizing GetIK...")
        self.group_name = group
        self.ik_timeout = ik_timeout
        self.ik_attempts = ik_attempts
        self.avoid_collisions = avoid_collisions
        rospy.loginfo("Computing IKs for group: " + self.group_name)
        rospy.loginfo("With IK timeout: " + str(self.ik_timeout))
        rospy.loginfo("And IK attempts: " + str(self.ik_attempts))
        rospy.loginfo("Setting avoid collisions to: " +
                      str(self.avoid_collisions))
        self.ik_srv = rospy.ServiceProxy('/compute_ik',
                                         GetPositionIK)
        rospy.loginfo("Waiting for /compute_ik service...")
        self.ik_srv.wait_for_service()
        rospy.loginfo("Connected!")

    def get_ik(self, pose_stamped,
               group=None,
               ik_timeout=None,
               ik_attempts=None,
               avoid_collisions=None):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param str group: The MoveIt! group.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """
        if group is None:
            group = self.group_name
        if ik_timeout is None:
            ik_timeout = self.ik_timeout
        if ik_attempts is None:
            ik_attempts = self.ik_attempts
        if avoid_collisions is None:
            avoid_collisions = self.avoid_collisions
        req = GetPositionIKRequest()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(ik_timeout)
        req.ik_request.attempts = ik_attempts
        req.ik_request.avoid_collisions = avoid_collisions

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp
    
    def pose_from_pos_quat(self, pos, quat):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = quat[0]
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        return pose

    def pose_stamped_from_pose(self, pose):
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose = pose
        return pose_stamped

if __name__ == "__main__":
    group = "right_arm"
    ik_srv = YumiKinematics(group)

    pos = [.5, 0, .5]
    quat = [0,0,1,0]
    pose = ik_srv.pose_from_pos_quat(pos, quat)
    res = ik_srv.get_ik(pose, avoid_collisions=True)
    print res

    print "ending\n\n\n\n"