#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from controller.srv import GoToPose

class PoseControllerNode(Node):
    def __init__(self):
        super().__init__('pose_controller_node')

        # --- Parameters ---
        self.declare_parameter('k_rho', 1.2)
        self.declare_parameter('k_alpha', 3.5)
        self.declare_parameter('k_beta', -0.6)
        self.declare_parameter('k_theta', 2.0)
        self.declare_parameter('pos_tolerance', 0.05) 
        self.declare_parameter('yaw_tolerance', math.radians(5.0)) 

        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.k_beta = self.get_parameter('k_beta').value
        self.k_theta = self.get_parameter('k_theta').value
        self.pos_tolerance = self.get_parameter('pos_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        # --- Robot State ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # --- Goal State ---
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.is_active = False

        # --- ROS Communications ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(GoToPose, 'go_to_pose', self.handle_go_to_pose)

        # Main control loop running at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Pose Controller Node Initialized successfully.")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract Yaw from Quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def handle_go_to_pose(self, request, response):
        if not self.odom_received:
            response.success = False
            response.message = "Rejected: No odometry data received yet."
            return response

        self.target_x = request.x
        self.target_y = request.y
        self.target_yaw = math.radians(request.yaw) 
        self.is_active = True

        self.get_logger().info(f"New Target Accepted: X={self.target_x}, Y={self.target_y}, Yaw={request.yaw}°")

        response.success = True
        response.message = "Target coordinates accepted. Navigating..."
        return response

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.is_active or self.target_x is None:
            return

        # Compute Errors
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        rho = math.sqrt(dx**2 + dy**2)

        twist = Twist()

        # Step 1: Position Control Check
        if rho > self.pos_tolerance:
            alpha = self.normalize_angle(math.atan2(dy, dx) - self.current_yaw)
            beta = self.normalize_angle(self.target_yaw - math.atan2(dy, dx))

            if alpha > math.pi / 2 or alpha < -math.pi / 2:
                alpha = self.normalize_angle(alpha - math.pi)
                beta = self.normalize_angle(beta - math.pi)
                v = -self.k_rho * rho
            else:
                v = self.k_rho * rho

            omega = self.k_alpha * alpha + self.k_beta * beta

            twist.linear.x = max(min(v, 0.22), -0.22) 
            twist.angular.z = max(min(omega, 2.84), -2.84)

        else:
            # Step 2: Final Orientation Alignment Check
            final_yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)

            if abs(final_yaw_error) > self.yaw_tolerance:
                twist.linear.x = 0.0
                omega = self.k_theta * final_yaw_error
                twist.angular.z = max(min(omega, 2.84), -2.84)
            else:
                # Goal fully achieved within specified limits
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.is_active = False
                self.get_logger().info("Target goal reached successfully!")

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PoseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
