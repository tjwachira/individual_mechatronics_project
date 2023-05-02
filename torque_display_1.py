#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState
import numpy as np

class TorqueMonitor:

    def __init__(self):
        rospy.init_node('torque_monitor', anonymous=True)
        self.latest_tau_ext_hat_filtered = None
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        self.contact_threshold = 0.0  # Set your desired threshold here
        
    def franka_state_callback(self, data):
        self.latest_tau_ext_hat_filtered = data.tau_ext_hat_filtered
    
    def is_in_contact(self):
        if self.latest_tau_ext_hat_filtered is not None:
            for torque_value in self.latest_tau_ext_hat_filtered:
                if abs(torque_value) > self.contact_threshold:
                    return True
        return False

    def compute_force_vector(self):
        jacobian_transpose = self.compute_jacobian_transpose()
        torque_values = np.array(self.latest_tau_ext_hat_filtered[:6])  # Only take the first 6 elements
        force_vector = np.dot(jacobian_transpose, torque_values)
        return force_vector

    def monitor_torque(self, rate=1):
        r = rospy.Rate(rate)  # Update frequency in Hz
        while not rospy.is_shutdown():
            if self.is_in_contact():
                rospy.loginfo("Latest tau_ext_hat_filtered values: %s", self.latest_tau_ext_hat_filtered)
            r.sleep()

if __name__ == '__main__':
    try:
        torque_monitor = TorqueMonitor()
        torque_monitor.monitor_torque(rate=1)  # Update frequency in Hz
    except rospy.ROSInterruptException:
        pass
