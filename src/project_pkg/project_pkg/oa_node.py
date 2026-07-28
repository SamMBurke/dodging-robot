#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

from .oa_algorithm import (
    step1_gating,
    step2_compute_ttc,
    step2_generate_candidates,
    step3_cost_function,
    step4_lock_global,
    step5_compute_velocity,
    wrap_angle,
    local_to_global,
    global_to_local
)


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('oa_node')

        # Parameter initialization
        self.R = 0.5               # Contact zone
        self.v_max = 0.22          # Turtlebot max linear velocity
        self.omega_max = 2.84      # Turtlebot max angular velocity
        self.L = 0.160             # Track

        # For velocity cmd
        self.Kv = 0.6              
        self.Kw = 0.8              

        # Weight factors
        self.w_dist = 0.25
        self.w_angle = 0.45
        self.w_time = 0.30

        # Others
        self.rho = 0.85            
        self.alpha = 0.3           
        self.v_dead = 0.005        
        self.omega_dead = 0.005    
        self.stop_dist = 0.05      

        # Evasion point grid parameter
        self.num_angles = 72       
        self.num_radii = 10       

        # ROS 2 subscriber + publisher
        # Kalman Filter (modify according to KF node)
        self.kf_sub = self.create_subscription(
            Float32MultiArray,
            '/obstacle_kf',
            self.kf_callback,
            10
        )

        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Vel cmd
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # State cache
        self.obs = None          # Tuple (x_o, y_o, vx_o, vy_o) from KF
        self.robot_pose = None   # Tuple (x_r, y_r, theta_r) from odometry

        # Step 4 locked target state
        self.locked_global = None
        self.locked_cost = float('inf')
        self.locked_reverse = False
        self.locked_delta_theta = 0.0

        # Step 5 smoothed vel hisotry
        self.V_prev = 0.0
        self.omega_prev = 0.0

        # --- Startup message ---
        self.get_logger().info("Obstacle Avoidance Node started!")
        self.get_logger().info(f"   R={self.R}m, v_max={self.v_max}m/s, omega_max={self.omega_max}rad/s")
        self.get_logger().info(f"   Kv={self.Kv}, Kw={self.Kw}, alpha={self.alpha}, rho={self.rho}")

    # Callback functions
    def kf_callback(self, msg):

        if len(msg.data) < 4:
            self.get_logger().warn("KF message has less than 4 elements!")
            return
        x_o, y_o, vx_o, vy_o = msg.data[0:4]
        self.obs = (x_o, y_o, vx_o, vy_o)

    def odom_callback(self, msg):
      
        x_r = msg.pose.pose.position.x
        y_r = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        theta_r = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.robot_pose = (x_r, y_r, theta_r)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # Utility functions
    def publish_cmd(self, linear, angular):
       
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    # Main control loop
    def control_loop(self):
  
        # Check data readiness
        if self.obs is None:
            self.get_logger().debug("No KF data yet; wait for the first message")
            return
        if self.robot_pose is None:
            self.get_logger().debug("No odometry yet; wait for the first message")
            return

        x_o, y_o, vx_o, vy_o = self.obs
        robot_pose = self.robot_pose

        # If detected obstacle is stationary, then its not the moving Turtlebot 4, robot does not move
        if abs(vx_o) < 1e-8 and abs(vy_o) < 1e-8:
            self.locked_global = None
            self.publish_cmd(0.0, 0.0)
            return

        # Step 1: Gating Condition
        is_threat, d_cpa = step1_gating(
            x_o, y_o, vx_o, vy_o,
            self.R
        )
        if not is_threat:
            self.locked_global = None
            self.publish_cmd(0.0, 0.0)
            return


        # Step 2: Compute True TTC and Generate Candidates
        t_entry = step2_compute_ttc(
            x_o, y_o, vx_o, vy_o,
            self.R
        )
        if t_entry is None or t_entry <= 0:
            self.publish_cmd(0.0, 0.0)
            return

        candidates = step2_generate_candidates(
            x_o, y_o, vx_o, vy_o,
            t_entry,
            self.R,
            self.v_max,
            self.omega_max,
            self.num_angles,
            self.num_radii
        )
        if not candidates:
            self.locked_global = None
            self.publish_cmd(0.0, 0.0)
            return

        
        # Step 3: Cost Function
        best_local = step3_cost_function(
            candidates,
            t_entry,
            self.R,
            self.w_dist,
            self.w_angle,
            self.w_time
        )
        if best_local is None:
            self.locked_global = None
            self.publish_cmd(0.0, 0.0)
            return

        
        # Step 4: Lock Global + Safety Check
        (self.locked_global,
         self.locked_cost,
         self.locked_reverse,
         self.locked_delta_theta) = step4_lock_global(
            best_local,
            robot_pose,
            self.locked_global,
            self.locked_cost,
            self.locked_reverse,
            self.locked_delta_theta,
            t_entry,
            x_o, y_o, vx_o, vy_o,
            self.R,
            self.v_max,
            self.omega_max,
            self.rho
        )

        if self.locked_global is None:
            self.publish_cmd(0.0, 0.0)
            return


        # Step 5: Velocity Command
        V_R, V_L, V_out, omega_out, self.V_prev, self.omega_prev = step5_compute_velocity(
            self.locked_global,
            robot_pose,
            self.locked_reverse,
            self.locked_delta_theta,
            self.v_max,
            self.omega_max,
            self.L,
            self.Kv,
            self.Kw,
            self.alpha,
            self.V_prev,
            self.omega_prev,
            self.v_dead,
            self.omega_dead,
            self.stop_dist
        )

        self.publish_cmd(V_out, omega_out)

        self.get_logger().debug(
            f"V={V_out:.3f}, omega={omega_out:.3f}, "
            f"rev={self.locked_reverse}, delta={math.degrees(self.locked_delta_theta):.1f}°, "
            f"candidates={len(candidates)}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()