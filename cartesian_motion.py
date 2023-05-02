#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from franka_msgs.msg import FrankaState #new add


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
        #new additon 
        self.latest_tau_ext_hat_filtered = None
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.franka_state_callback)

         # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        
    def franka_state_callback(self, data):
        self.latest_tau_ext_hat_filtered = data.tau_ext_hat_filtered  #new additon 
        
        
    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.23 # this will make it move downwards by 20 cm 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.23  #move back up the z axis
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  
        return plan, fraction

      
    def display_trajectory(self, plan): # this whole constructor is to display the trajectoey path in RVIZ as well. 

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

       
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        
        move_group = self.move_group
        move_group.execute(plan, wait=True)
        
        #new addition
        if self.latest_tau_ext_hat_filtered is not None:
            rospy.loginfo("Latest tau_ext_hat_filtered (torque) values: %s", self.latest_tau_ext_hat_filtered)
        
def main():
    try:
        
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveFrankaGazeboRobot()
        
        input(
                "============ Press `Enter` to plan and display a cartesian path, ..."
        )
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        input(
                "============ Press `Enter` to  display a saved trajectory (this will replay the cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan)
        input(
                "============ Press `Enter` to  execute the saved path  ..."
        )
        tutorial.execute_plan(cartesian_plan)
        
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()


        


