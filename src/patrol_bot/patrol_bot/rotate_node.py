import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from custom_interfaces.action import RotateAngle


class RotateNode(Node):
    def __init__(self):
        super().__init__('rotate_node')

        self.current_yaw = None

        self.odom_group = ReentrantCallbackGroup()
        self.action_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.odom_group
        )

        self.action_server = ActionServer(
            self,
            RotateAngle,
            '/rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group
        )

        self.get_logger().info('Rotate node iniciado.')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Recebido goal: {goal_request.degrees} graus')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelamento solicitado.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_degrees = goal_handle.request.degrees
        target_radians = math.radians(abs(target_degrees))

        while rclpy.ok() and self.current_yaw is None:
            time.sleep(0.05)

        start_yaw = self.current_yaw
        last_yaw = start_yaw
        rotated = 0.0

        direction = 1.0 if target_degrees >= 0 else -1.0
        angular_speed = 0.8 * direction

        twist = Twist()
        twist.angular.z = angular_speed

        feedback = RotateAngle.Feedback()

        while rclpy.ok() and rotated < target_radians:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result = RotateAngle.Result()
                result.success = False
                return result

            current_delta = self.normalize_angle(self.current_yaw - last_yaw)
            rotated += abs(current_delta)
            last_yaw = self.current_yaw

            remaining = math.degrees(target_radians - rotated)
            if remaining < 0:
                remaining = 0.0

            feedback.degrees_remaining = remaining
            goal_handle.publish_feedback(feedback)

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.stop_robot()
        goal_handle.succeed()

        result = RotateAngle.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RotateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()