"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0


class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""
        # body rate control
        self.k_p_p = 8
        self.k_p_q = 8
        self.k_p_r = 4
        # altitude control
        self.k_p_z = 4
        self.k_d_z = 3.6
        # yaw control
        self.k_p_yaw = 3
        # roll-pitch control
        self.k_p_roll = 3
        self.k_p_pitch = 3

        self.k_p_body_rate = np.array([self.k_p_p, self.k_p_q, self.k_p_r], dtype=np.float)
        self.k_p_rp = np.array([self.k_p_roll, self.k_p_pitch], dtype=np.float)

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]

        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]

            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]

        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]

                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]

        position_cmd = (position1 - position0) * \
                       (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)

        return (position_cmd, velocity_cmd, yaw_cmd)

    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                                 acceleration_ff=np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        return np.array([0.0, 0.0])

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude,
                         acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: thrust command for the vehicle (+up)
        """
        rot_mat = euler2RM(*attitude)
        b_z = rot_mat[2, 2]

        e_z = altitude_cmd - altitude
        e_z_dot = vertical_velocity_cmd - vertical_velocity
        net_z_dot_dot_c = self.k_p_z * e_z + self.k_d_z * e_z_dot + acceleration_ff

        z_dot_dot_c = net_z_dot_dot_c - GRAVITY
        thrust = z_dot_dot_c / b_z * DRONE_MASS_KG

        return thrust

    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll,pitch,yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        rot_mat = euler2RM(*attitude)
        b = rot_mat[0:2, 2]
        c_c = -thrust_cmd / DRONE_MASS_KG
        b_c = acceleration_cmd / c_c

        e_b = b_c - b
        b_c_dot = self.k_p_rp * e_b

        r = np.array([[rot_mat[1, 0], -rot_mat[0, 0]],
                      [rot_mat[1, 1], -rot_mat[0, 1]]],
                     dtype=np.float)

        pq_c = np.dot(r, b_c_dot) / rot_mat[2, 2]
        return pq_c

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            attitude: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        e = body_rate_cmd - body_rate
        # print(body_rate_cmd, body_rate)
        angular_acc = self.k_p_body_rate * e
        tau = MOI * angular_acc
        # print("Tau: ", tau)
        return tau

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        # note: yaw must be within [-pi, pi)
        while np.abs(yaw_cmd) > np.pi:
            direction = -1 if yaw_cmd > np.pi else 1
            yaw_cmd = yaw_cmd + direction * 2 * np.pi
        e_yaw = yaw_cmd - yaw
        if np.abs(e_yaw) > np.pi:
            e_yaw = e_yaw - 2 * np.pi
        return self.k_p_yaw * e_yaw
