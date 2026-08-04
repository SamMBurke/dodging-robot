#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from project_interfaces.msg import TrackedObjectArray

from .oa_algorithm import (
    step1_gating,
    step2_compute_ttc,
    step2_generate_candidates,
    step3_cost_function,
    step4_lock_global,
    step5_compute_velocity,
    global_to_local,
    global_to_local_vector,
)


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('oa_node')

        # Parameter initialization
        self.R = 1.5                # Contact zone
        self.engagement_radius = 3    # the radius in which the robot actually reacts to incoming obstacles
                                        # this way, objects that are far out and noisy aren't reacted to
        self.robot_radius = 0.22
        self.v_max = 0.31
        self.omega_max = 1.90
        self.L = 0.160               # Track (wheel base)

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

        # Tracked objects come from ObjectTrackerNode, in the fixed ('odom') frame.
        # We cache the raw message and convert to the robot's local frame inside
        # control_loop, using the latest robot_pose from /odom -- this avoids needing
        # a separate tf2 lookup, since /odom already gives us robot_pose in this same
        # fixed frame.
        self.objects_sub = self.create_subscription(
            TrackedObjectArray,
            '/tracked_objects',
            self.objects_callback,
            10
        )

        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        # Vel cmd
        self.cmd_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        # Debug visualization -- shows the locked evasion point and planned direction in RViz2
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/oa_debug_markers', 
            10
        )

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # State cache
        self.tracked_objects_msg = None   # latest TrackedObjectArray, in the fixed ('odom') frame
        self.robot_pose = None            # Tuple (x_r, y_r, theta_r) from odometry, in the fixed ('odom') frame

        # Step 4 locked target state
        self.locked_global = None
        self.locked_cost = float('inf')
        self.locked_reverse = False
        self.locked_delta_theta = 0.0

        # Step 5 smoothed vel history
        self.V_prev = 0.0
        self.omega_prev = 0.0

        # --- Startup message ---
        self.get_logger().info("Obstacle Avoidance Node started!")
        self.get_logger().info(f"   R={self.R}m, robot_radius={self.robot_radius}m, v_max={self.v_max}m/s, omega_max={self.omega_max}rad/s")
        self.get_logger().info(f"   Kv={self.Kv}, Kw={self.Kw}, alpha={self.alpha}, rho={self.rho}")

    # Callback functions
    def objects_callback(self, msg):
        self.tracked_objects_msg = msg

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

    def select_worst_case_object(self, robot_pose):
        '''
        Converts every tracked object from the fixed (odom) frame into the robot's local
        frame, filters out stationary ones (this node only reacts to moving obstacles, by
        design), runs the CPA gating check on each, and returns the local-frame (x_o, y_o,
        vx_o, vy_o) of whichever moving, threatening object has the smallest CPA distance --
        i.e. the single most urgent threat this control cycle. Returns None if nothing
        currently qualifies as a threat.
        '''
        worst = None
        worst_d_cpa = float('inf')

        for obj in self.tracked_objects_msg.objects:
            x_o, y_o = global_to_local(obj.center.x, obj.center.y, robot_pose)
            if math.hypot(x_o, y_o) > self.engagement_radius:
                continue # object is outside the engagement radius => don't react

            vx_o, vy_o = global_to_local_vector(obj.velocity.x, obj.velocity.y, robot_pose)

            if abs(vx_o) < 1e-8 and abs(vy_o) < 1e-8:
                continue  # stationary -- not something this reactive layer avoids

            is_threat, d_cpa = step1_gating(x_o=x_o, y_o=y_o, vx_o=vx_o, vy_o=vy_o, R=self.R)
            if is_threat and d_cpa < worst_d_cpa:
                worst_d_cpa = d_cpa
                worst = (x_o, y_o, vx_o, vy_o)

        return worst

    def publish_debug_markers(self, locked_global, robot_pose):
        marker_array = MarkerArray()

        if locked_global is None or robot_pose is None:
            for marker_id in (0, 1):
                delete_marker = Marker()
                delete_marker.header.frame_id = 'odom'
                delete_marker.ns = 'oa_debug'
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            return

        stamp = self.get_clock().now().to_msg()
        X, Y = locked_global
        x_r, y_r, _ = robot_pose

        point_marker = Marker()
        point_marker.header.frame_id = 'odom'
        point_marker.header.stamp = stamp
        point_marker.ns = 'oa_debug'
        point_marker.id = 0
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position.x = X
        point_marker.pose.position.y = Y
        point_marker.pose.position.z = 0.1
        point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = 0.15
        point_marker.color.r, point_marker.color.g, point_marker.color.b, point_marker.color.a = 1.0, 0.0, 1.0, 1.0
        point_marker.lifetime = Duration(seconds=0.3).to_msg()
        marker_array.markers.append(point_marker)

        line_marker = Marker()
        line_marker.header.frame_id = 'odom'
        line_marker.header.stamp = stamp
        line_marker.ns = 'oa_debug'
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.03
        line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a = 1.0, 0.0, 1.0, 1.0
        line_marker.points = [
            Point(x=x_r, y=y_r, z=0.1),
            Point(x=X, y=Y, z=0.1),
        ]
        line_marker.lifetime = Duration(seconds=0.3).to_msg()
        marker_array.markers.append(line_marker)

        self.marker_pub.publish(marker_array)

    # Main control loop
    def control_loop(self):

        # Check data readiness
        if self.tracked_objects_msg is None:
            self.get_logger().debug("No tracked objects yet; wait for the first message")
            return
        if self.robot_pose is None:
            self.get_logger().debug("No odometry yet; wait for the first message")
            return

        robot_pose = self.robot_pose

        # Pick the single most urgent threat among all currently tracked objects
        worst = self.select_worst_case_object(robot_pose)
        if worst is None:
            self.locked_global = None
            self.publish_cmd(0.0, 0.0)
            self.publish_debug_markers(None, robot_pose)
            return

        x_o, y_o, vx_o, vy_o = worst

        # Step 2: Compute True TTC and Generate Candidates
        t_entry = step2_compute_ttc(x_o=x_o, y_o=y_o, vx_o=vx_o, vy_o=vy_o, R=self.R)
        if t_entry is None or t_entry <= 0:
            self.publish_cmd(0.0, 0.0)
            self.publish_debug_markers(None, robot_pose)
            return

        candidates = step2_generate_candidates(
            x_o=x_o, y_o=y_o, vx_o=vx_o, vy_o=vy_o,
            t_entry=t_entry,
            R=self.R,
            robot_radius=self.robot_radius,
            v_max=self.v_max,
            omega_max=self.omega_max,
            num_angles=self.num_angles,
            num_radii=self.num_radii,
        )
        if not candidates:
            self.locked_global = None
            self.publish_cmd(0.0, 0.0)
            self.publish_debug_markers(None, robot_pose)
            return

        # Step 3: Cost Function
        best_local = step3_cost_function(
            candidates=candidates,
            t_entry=t_entry,
            R=self.R,
            w_dist=self.w_dist,
            w_angle=self.w_angle,
            w_time=self.w_time,
        )
        if best_local is None:
            self.locked_global = None
            self.publish_cmd(0.0, 0.0)
            self.publish_debug_markers(None, robot_pose)
            return

        # Step 4: Lock Global + Safety Check
        (self.locked_global,
         self.locked_cost,
         self.locked_reverse,
         self.locked_delta_theta) = step4_lock_global(
            best_local=best_local,
            robot_pose=robot_pose,
            locked_global=self.locked_global,
            locked_cost=self.locked_cost,
            locked_reverse=self.locked_reverse,
            locked_delta_theta=self.locked_delta_theta,
            t_entry=t_entry,
            x_o=x_o, y_o=y_o, vx_o=vx_o, vy_o=vy_o,
            R=self.R,
            v_max=self.v_max,
            omega_max=self.omega_max,
            rho=self.rho,
        )

        if self.locked_global is None:
            self.publish_cmd(0.0, 0.0)
            self.publish_debug_markers(None, robot_pose)
            return

        # Step 5: Velocity Command
        V_R, V_L, V_out, omega_out, self.V_prev, self.omega_prev = step5_compute_velocity(
            locked_global=self.locked_global,
            robot_pose=robot_pose,
            use_reverse=self.locked_reverse,
            delta_theta_eff=self.locked_delta_theta,
            v_max=self.v_max,
            omega_max=self.omega_max,
            wheel_base=self.L,
            Kv=self.Kv,
            Kw=self.Kw,
            alpha=self.alpha,
            v_prev=self.V_prev,
            omega_prev=self.omega_prev,
            v_dead=self.v_dead,
            omega_dead=self.omega_dead,
            stop_dist=self.stop_dist,
        )

        self.publish_cmd(V_out, omega_out)
        self.publish_debug_markers(self.locked_global, robot_pose)

        self.get_logger().debug(
            f"V={V_out:.3f}, omega={omega_out:.3f}, "
            f"rev={self.locked_reverse}, delta={math.degrees(self.locked_delta_theta):.1f}deg, "
            f"candidates={len(candidates)}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoidanceNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
