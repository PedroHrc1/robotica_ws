import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import SetBool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from custom_interfaces.action import RotateAngle


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.start_service = self.create_service(
            SetBool,
            '/start_patrol',
            self.start_patrol_callback
        )

        self.rotate_client = ActionClient(self, RotateAngle, '/rotate')

        self.timer = self.create_timer(0.1, self.control_loop)

        self.patrolling = False
        self.rotating = False
        self.obstacle_detected = False

        self.obstacle_distance = 0.6
        self.linear_speed = 0.8
        self.rotation_angle = 45.0
        self.front_angle_deg = 15.0

        self.get_logger().info('Patrol node iniciado. Aguardando serviço /start_patrol.')

    def scan_callback(self, msg):
        if len(msg.ranges) == 0:
            return

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        min_angle = math.radians(-self.front_angle_deg)
        max_angle = math.radians(self.front_angle_deg)

        start_index = int((min_angle - angle_min) / angle_increment)
        end_index = int((max_angle - angle_min) / angle_increment)

        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        front_ranges = []

        for i in range(start_index, end_index + 1):
            distance = msg.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance):
                front_ranges.append(distance)

        if len(front_ranges) == 0:
            self.obstacle_detected = False
        else:
            min_distance = min(front_ranges)
            self.obstacle_detected = min_distance < self.obstacle_distance

    def start_patrol_callback(self, request, response):
        if request.data:
            if not self.patrolling:
                self.patrolling = True
                response.success = True
                response.message = 'Patrulha iniciada.'
                self.get_logger().info('Patrulha iniciada.')
            else:
                response.success = False
                response.message = 'O robo ja esta em patrulha.'
                self.get_logger().info('Tentaram iniciar, mas o robo ja esta em patrulha.')
        else:
            self.patrolling = False
            self.rotating = False
            self.stop_robot()
            response.success = True
            response.message = 'Patrulha interrompida.'
            self.get_logger().info('Patrulha interrompida.')

        return response

    def control_loop(self):
        if not self.patrolling:
            return

        if self.rotating:
            return

        if self.obstacle_detected:
            self.stop_robot()
            self.send_rotate_goal(self.rotation_angle)
            return

        self.move_forward()

    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def send_rotate_goal(self, degrees):
        if not self.rotate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server /rotate nao está disponível.')
            return

        self.rotating = True

        goal_msg = RotateAngle.Goal()
        goal_msg.degrees = degrees

        self.get_logger().info(f'Enviando goal de rotação: {degrees} graus')

        self._send_goal_future = self.rotate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.degrees_remaining
        self.get_logger().info(f'Graus restantes: {remaining:.2f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal de rotação foi rejeitado.')
            self.rotating = False
            return

        self.get_logger().info('Goal de rotação aceito.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info('Rotação concluída com sucesso.')
        else:
            self.get_logger().info('Rotação falhou.')

        self.rotating = False


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()