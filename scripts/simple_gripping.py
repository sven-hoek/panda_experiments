#!/usr/bin/env python2

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import shape_msgs.msg
import control_msgs.msg
from moveit_commander.conversions import pose_to_list
#from math import pi
#from std_msgs.msg import String

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a
    tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.gripper_action_client = actionlib.SimpleActionClient("franka_gripper/gripper_action", control_msgs.msg.GripperCommandAction)

        self.planning_frame = self.group.get_planning_frame()
        print "============ Reference frame: %s" % self.planning_frame
        self.eef_link = self.group.get_end_effector_link()
        print "============ End effector: %s" % self.eef_link
        self.group_names = self.robot.get_group_names()
        print "============ Robot Groups:", self.group_names
        #print "============ Printing robot state"
        #print robot.get_current_state()
        print ""

        # Define gripper open and closed postures
        self.gripper_open = trajectory_msgs.msg.JointTrajectory()
        self.gripper_open.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        self.gripper_open.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
        self.gripper_open.points[0].positions = [0.04, 0.04]
        self.gripper_open.points[0].time_from_start = rospy.Duration(0.5)

        self.gripper_closed = copy.deepcopy(self.gripper_open)
        self.gripper_closed.points[0].positions = [0.0, 0.0]

        self.grasp_pose = geometry_msgs.msg.Pose()
        self.grasp_pose.position.x = 0.5
        self.grasp_pose.position.y = 0.0
        self.grasp_pose.position.z = 0.13
        self.grasp_pose.orientation.w = -0.0000159
        self.grasp_pose.orientation.x = 0.926539
        self.grasp_pose.orientation.y = -0.376198
        self.grasp_pose.orientation.z = -0.0002354

        self.lift_pose = copy.deepcopy(self.grasp_pose)
        self.lift_pose.position.x = 0.52
        self.lift_pose.position.z = 0.2

    def add_collision_objects(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.026
        box_pose.pose.orientation.w = 1.0
        self.scene.add_box("cube", box_pose, size=(0.05, 0.05, 0.05))

    def remove_collision_objects(self):
        self.scene.remove_attached_object("panda_link8")
        rospy.sleep(0.1)
        self.scene.remove_world_object("cube")

    def go_to_pose_goal(self, pose, approval_required=True):
        self.group.set_pose_target(pose)
        plan = self.group.plan()
        print "Continue? (Press ENTER)"
        raw_input()
        self.group.execute(plan, wait = True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

        # For testing:
        current_pose = self.group.get_current_pose().pose
        return all_close(pose, current_pose, 0.01)


    def pick(self):
        grasps = [moveit_msgs.msg.Grasp()]
        grasps[0].grasp_pose.header.frame_id = "panda_link0"
        grasps[0].grasp_pose.pose.position.x = 0.5
        grasps[0].grasp_pose.pose.position.y = 0.0
        grasps[0].grasp_pose.pose.position.z = 0.13
        grasps[0].grasp_pose.pose.orientation.w = -0.0000159
        grasps[0].grasp_pose.pose.orientation.x = 0.926539
        grasps[0].grasp_pose.pose.orientation.y = -0.376198
        grasps[0].grasp_pose.pose.orientation.z = -0.0002354

        grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0"
        grasps[0].pre_grasp_approach.direction.vector.z = -1.0
        grasps[0].pre_grasp_approach.min_distance = 0.11
        grasps[0].pre_grasp_approach.desired_distance = 0.12

        grasps[0].post_grasp_retreat = grasps[0].pre_grasp_approach
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0

        grasps[0].pre_grasp_posture = self.gripper_open
        grasps[0].grasp_posture = self.gripper_closed

        self.group.pick("cube")
        #self.group.


    def move_gripper(self, position, max_effort=4, blocking=False):
        if self.gripper_action_client.wait_for_server(rospy.Duration(0.1)) is False:
            rospy.logwarn("Could not reach action server for Gripper!")
            return False
        else:
            goal = control_msgs.msg.GripperCommandGoal(command=control_msgs.msg.GripperCommand(position=position, max_effort=max_effort))
            self.gripper_action_client.send_goal(goal)
            if blocking is True:
                self.gripper_action_client.wait_for_result()
                return self.gripper_action_client.get_result()
            else:
                return True



def main():
    try:
        tutorial = MoveGroupPythonIntefaceTutorial()

        #print "============ Press `Enter` to add collision objects..."
        #raw_input()
        #tutorial.add_collision_objects()

        print "============ Press `Enter` to open gripper..."
        if raw_input() == "x": quit()
        tutorial.move_gripper(0.039)

        print "============ Press `Enter` to move to grasp pose..."
        if raw_input() == "x": quit()
        print "Success: " + str(tutorial.go_to_pose_goal(tutorial.grasp_pose))

        print "============ Press `Enter` to close gripper..."
        if raw_input() == "x": quit()
        tutorial.move_gripper(0.0)

        print "============ Press `Enter` to move to lift pose..."
        if raw_input() == "x": quit()
        print "Success: " + str(tutorial.go_to_pose_goal(tutorial.lift_pose))

        #print "============ Press `Enter` to move to grasp pose..."
        #raw_input()
        #print "Success: " + str(tutorial.go_to_pose_goal(tutorial.grasp_pose))

        print "============ Press `Enter` to open gripper..."
        if raw_input() == "x": quit()
        tutorial.move_gripper(0.039)

        #print "============ Press `Enter` to grasp..."
        #raw_input()
        #tutorial.pick()

        #print "============ Press `Enter` to remove collision objects..."
        #raw_input()
        #tutorial.remove_collision_objects()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
