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
        self.k_p_p = 20
        self.k_p_q = 20
        self.k_p_r = 3
        # altitude control
        self.k_p_z = 3.6
        self.k_d_z = 2.1
        # yaw control
        self.k_p_yaw = 1.5
        # roll-pitch control
        self.k_p_pitch = 6.0
        self.k_p_roll = 6.0
        # lateral control
        self.k_p_x = 4.2
        self.k_d_x = 3.0
        self.k_p_y = 4.2
        self.k_d_y = 3.0
        # gains arranged in numpy array form
        self.k_p_body_rate = np.array([self.k_p_p, self.k_p_q, self.k_p_r], dtype=np.float)
        self.k_p_pr = np.array([self.k_p_pitch, self.k_p_roll], dtype=np.float)
        self.k_p_xy = np.array([self.k_p_x, self.k_p_y], dtype=np.float)
        self.k_d_xy = np.array([self.k_d_x, self.k_d_y], dtype=np.float)

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
        e_position = local_position_cmd - local_position
        e_velocity = local_velocity_cmd - local_velocity
        acc_cmd = self.k_p_xy * e_position + self.k_d_xy * e_velocity + acceleration_ff
        return acc_cmd

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
        z_dot_dot_c = self.k_p_z * e_z + self.k_d_z * e_z_dot + acceleration_ff
        #print("AltC: {}, Alt: {}, Vc: {}, V: {}, Ez: {}, Ezd: {}, Zddc: {}".format(
        #    altitude_cmd, altitude, vertical_velocity_cmd, vertical_velocity,
        #    e_z, e_z_dot, z_dot_dot_c))
        c_c = z_dot_dot_c / b_z
        thrust = c_c * DRONE_MASS_KG
        thrust = np.clip(thrust, 0.1, MAX_THRUST)
        # print("target alt: {}, current alt: {}, z_dd_c: {}, thrust:{}".format(altitude_cmd, altitude, z_dot_dot_c,
        #                                                                       thrust))
        #print("Bz: {}, ThrustO: {}, thrust: {}, attitude: {}".format(b_z, c_c * DRONE_MASS_KG, thrust, attitude))
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
        # notice the gain is named as k_p_pr here. the first element in e_b
        # is b_x_c, which need pitching. b_y_c on the other hand, need rolling
        b_c_dot = self.k_p_pr * e_b

        r = np.array([[rot_mat[1, 0], -rot_mat[0, 0]],
                      [rot_mat[1, 1], -rot_mat[0, 1]]],
                     dtype=np.float)

        pq_c = np.dot(r, b_c_dot) / rot_mat[2, 2]
        # print(acceleration_cmd, b_c_dot)
        return pq_c

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            attitude: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        e = body_rate_cmd - body_rate
        angular_acc = self.k_p_body_rate * e
        tau = MOI * angular_acc
        tau = np.clip(tau, -MAX_TORQUE, MAX_TORQUE)
        # print("body rate: cmd: {}, rate: {}, tau: {}".format(body_rate_cmd, body_rate, tau))
        return tau

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        # note: yaw must be within [-pi, pi)
        yaw_cmd = np.fmod(yaw_cmd + np.pi, 2 * np.pi) - np.pi
        e_yaw = yaw_cmd - yaw
        if np.abs(e_yaw) > np.pi:
            direction = -1 if e_yaw > 0 else 1
            e_yaw = e_yaw + direction * 2 * np.pi
        # print("yaw_cmd: {}, yaw: {}, E_yaw: {}".format(yaw_cmd, yaw, e_yaw))
        return self.k_p_yaw * e_yaw
