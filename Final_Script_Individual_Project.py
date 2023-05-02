#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input
from math import pi, dist, fabs, cos, atan2
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from franka_msgs.msg import FrankaState
import tf.transformations as tf_transform
import numpy as np

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
        
        self.latest_tau_ext_hat_filtered = None
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
    
         # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.move_group.set_end_effector_link("panda_hand_tcp")
        self.move_group.set_pose_reference_frame("panda_link0")
        self.display_trajectory_publisher = display_trajectory_publisher
        self.contact_threshold = 1.0  # Set your desired threshold here, so that torque values above this threshold are printed
        
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
        pose_goal.position.y = 0.27
        pose_goal.position.z = 0.36
    
        move_group.set_pose_target(pose_goal) 
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.16 # this will make it move downwards by 20 cm 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.16
        #move back up the z axis
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  
        return plan, fraction

    def display_trajectory(self, plan): # this whole function is to display the trajectoey path in RVIZ as well. 

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

       
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        
    #new addition - Only print torqu values when contact with object is made  
    def is_in_contact(self):
        if self.latest_tau_ext_hat_filtered is not None:
            for torque_value in self.latest_tau_ext_hat_filtered:
                if abs(torque_value) > self.contact_threshold:
                    return True
        return False
        
    def euler_to_quaternion(self, roll, pitch , yaw):
        quaternion = tf_transform.quaternion_from_euler(roll, pitch, yaw)
        return quaternion
    
    def franka_state_callback(self, data):
        self.latest_tau_ext_hat_filtered = data.tau_ext_hat_filtered  #new additon 
    
    # convert 7 seg array to force vector; intro to jacobian
    def compute_jacobian(self):
        move_group = self.move_group
        current_joint_values = move_group.get_current_joint_values()
        jacobian = move_group.get_jacobian_matrix(current_joint_values)
        return jacobian
    
    def compute_jacobian_transpose(self):
        jacobian = self.compute_jacobian()
        jacobian_transpose = np.transpose(jacobian)
        return jacobian_transpose
    
    def compute_force_vector(self):
        jacobian_transpose = self.compute_jacobian_transpose()
        torque_values = np.array(self.latest_tau_ext_hat_filtered[:6])  
        # Only take the first 6 elements
        force_vector = np.dot(jacobian_transpose, torque_values)
        return force_vector[:3]  
    # Only return the first 3 elements (forces in x, y, z directions)
    # force vector 'force vector'
    
    def print_current_pose(self):
        ee_pose = self.move_group.get_current_pose().pose
        print (ee_pose)
    
    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)
        
        if self.is_in_contact():          
            force_vector = self.compute_force_vector()
            rospy.loginfo("Force vector: %s", force_vector)
            rospy.loginfo("Latest tau_ext_hat_filtered (torque) values: %s", self.latest_tau_ext_hat_filtered)
            
            if not self.is_contact_perpendicular(force_vector):
                rospy.loginfo("Contact is not perpendicular, adjusting orientation...")
                adjusted_orientation = self.compute_adjusted_orientation(force_vector)
                self.go_to_adjusted_orientation_goal(adjusted_orientation)
                
    def is_contact_perpendicular(self, force_vector):
        # threshold wheterh something is perpendicular. Should try and increase this
        threshold = 0.1

        # Assuming that the z-component of the force vector represents the perpendicular force
        force_magnitude = np.linalg.norm(force_vector)
        z_component_ratio = abs(force_vector[2]) / force_magnitude

        return z_component_ratio > (1 - threshold)
    
    def compute_adjusted_orientation(self, force_vector):
        # Calculate the angle using the x, y, and z components of the force vector
        roll_adjustment = atan2(force_vector[1], force_vector[2])
        pitch_adjustment = atan2(force_vector[0], force_vector[2])
        yaw_adjustment = atan2(force_vector[1], force_vector[0])

        #logic to calculate the adjusted orientation
        '''
        adjusted_roll = current_euler[0]
        adjusted_pitch = current_euler[1]
        adjusted_yaw = current_euler[2]'''

        current_pose = self.move_group.get_current_pose().pose
        current_orientation = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ] # new values different from go_to_pose_goal
        current_euler = tf_transform.euler_from_quaternion(current_orientation)

        adjusted_euler = [
            current_euler[0] + roll_adjustment,
            current_euler[1] + pitch_adjustment,
            current_euler[2] + yaw_adjustment,
        ]
        adjusted_quaternion = tf_transform.quaternion_from_euler(*adjusted_euler)

        return adjusted_quaternion

    def go_to_adjusted_orientation_goal(self, adjusted_orientation):
        move_group = self.move_group
        force_vector = self.compute_force_vector()
        adjusted_quaternion = self.compute_adjusted_orientation(force_vector)

        pose_goal = move_group.get_current_pose().pose

        pose_goal.orientation.w = adjusted_quaternion[3]
        pose_goal.orientation.x = adjusted_quaternion[0] #new values
        pose_goal.orientation.y = adjusted_quaternion[1]
        pose_goal.orientation.z = adjusted_quaternion[2]

        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
           
def main():
    try:
        print("============ Setting up the moveit_commander ...")
        tutorial = MoveFrankaGazeboRobot()

        input("============ Press `Enter` to start robot motion ...")
        print("========= Going to joint configuration....")
        tutorial.go_to_joint_state()

        print("============ executing  movement using a pose goal ...")
        tutorial.go_to_pose_goal()
        tutorial.print_current_pose()

        print("============ planning and displaying a cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        tutorial.display_trajectory(cartesian_plan)

        print("============ executing the saved cartesian path ...")
        tutorial.execute_plan(cartesian_plan)
        
        # check if re-orientation is necessary when contact is established
        if tutorial.is_in_contact():
            force_vector = tutorial.compute_force_vector()
            adjusted_orientation = tutorial.compute_adjusted_orientation(force_vector)
            tutorial.go_to_adjusted_orientation_goal(adjusted_orientation)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
