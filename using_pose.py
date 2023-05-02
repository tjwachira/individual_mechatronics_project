#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


import tf.transformations as tf_transform
import tf
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
        self.move_group.set_end_effector_link("panda_hand_tcp")
        self.move_group.set_pose_reference_frame("panda_link0")
        self.display_trajectory_publisher = display_trajectory_publisher
        
        
      
    def print_current_pose(self):
        ee_pose = self.move_group.get_current_pose().pose
        print (ee_pose)
        
    def euler_to_quaternion(self, roll, pitch , yaw):
        quaternion = tf_transform.quaternion_from_euler(roll, pitch, yaw)
        return quaternion
        
    def go_to_pose_goal(self):

        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        
        #current_pose = self.move_group.get_current_pose().pose
        roll = -3.14
        pitch = 0.00
        yaw = -0.785
        quaternion = self.euler_to_quaternion(roll, pitch, yaw)

        pose_goal.orientation.w = quaternion[3]
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.position.x = 0.54
        pose_goal.position.y = 0.00
        pose_goal.position.z = 0.48
    

        move_group.set_pose_target(pose_goal) 
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
        
def main():
    try:
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveFrankaGazeboRobot()
        
        input(
                "============ Press `Enter` to execute a movement using a pose goal.;."
        )
       
        tutorial.go_to_pose_goal()
        tutorial.print_current_pose()
        
                
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
     
if __name__ == "__main__":
    main()
