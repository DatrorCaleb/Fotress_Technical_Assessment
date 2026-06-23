#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import your custom service
from pose_controller.srv import GoToPose

class PoseControllerNode(Node):
    def __init__(self):
        super().__init__('pose_controller_node')

        # Create the service server
        self.srv = self.create_service(GoToPose, 'go_to_pose', self.go_to_pose_callback)
        self.get_logger().info("Pose Controller Service Server is ready.")

    def go_to_pose_callback(self, request, response):
        # Extract the requested coordinates
        target_x = request.x
        target_y = request.y
        target_yaw = request.yaw

        self.get_logger().info(f"Received request: x={target_x}, y={target_y}, yaw={target_yaw}")

        # --- INSERT YOUR CONTROL LOGIC HERE ---
        # (e.g., publish velocity commands to move the robot to the target)

        # Send response back
        response.success = True
        response.message = f"Successfully target reached/processed for ({target_x}, {target_y})"
        return response

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
