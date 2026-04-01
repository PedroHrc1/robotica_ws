import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Navigator(Node):
    def __init__(self):
        super().__init__('atividades_navigator')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.subscription_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # faixa frontal que você quer analisar
        self.front_angle_deg = 20.0   # olha de -20° até +20°
        self.stop_distance = 1.0      # para/gira se achar algo a menos de 1 metro

    def get_front_ranges(self, msg: LaserScan):
        """
        Retorna os ranges da faixa frontal [-front_angle_deg, +front_angle_deg].
        """
        front_angle_rad = math.radians(self.front_angle_deg)

        # converte ângulo -> índice
        start_index = int(((-front_angle_rad) - msg.angle_min) / msg.angle_increment)
        end_index = int(((front_angle_rad) - msg.angle_min) / msg.angle_increment)

        # garante que os índices estão dentro do vetor
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        selected_ranges = msg.ranges[start_index:end_index + 1]

        # remove medições inválidas
        valid_ranges = [
            r for r in selected_ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]

        return valid_ranges

    def subscription_callback(self, msg: LaserScan):
        twist = Twist()

        front_ranges = self.get_front_ranges(msg)

        if not front_ranges:
            # se não houver leitura válida, segue com cautela
            self.get_logger().warn('Nenhuma leitura válida na faixa frontal.')
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self.publisher.publish(twist)
            return

        min_front = min(front_ranges)

        self.get_logger().info(f"Menor distância na frente: {min_front:.2f} m")

        if min_front < self.stop_distance:
            self.get_logger().info('Obstáculo detectado na faixa frontal!')
            twist.linear.x = 0.0
            twist.angular.z = 0.8
        else:
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    print("Iniciando o nó de navegação...")
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()