#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveFrankaGazeboRobot(object):
    def __init__(self):
        super(MoveFrankaGazeboRobot, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_franka_gazebo_robot", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
         # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.move_group.set_end_effector_link("panda_hand_tcp")
        self.move_group.set_pose_reference_frame("panda_link0")
        
    def print_current_pose(self):
        ee_pose = self.move_group.get_current_pose().pose
        print (ee_pose)

    def go_to_joint_state(self):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.0 * (pi /180)
        joint_goal[1] = -29.0 * (pi /180)
        joint_goal[2] = 0.0 * (pi /180)
        joint_goal[3] = -92 * (pi /180)
        joint_goal[4] = 0 * (pi /180)
        joint_goal[5] = 62 * (pi /180)
        joint_goal[6] = 48 * (pi /180)

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
def main():
    try:
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveFrankaGazeboRobot()
        
        input(
                "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
       
        tutorial.go_to_joint_state() 
        tutorial.print_current_pose()
                
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
     
if __name__ == "__main__":
    main()
