import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Aula8

class Subscriber(Node):
    def __init__(self):
        super().__init__('aula8_subscriber')
        self.subscription = self.create_subscription(
            Aula8, 'aula8_topic', self.subscription_callback, 10)

    def subscription_callback(self, msg):
        mensagem = msg.message
        contagem = msg.count
        self.get_logger().info('Recebendo: "%s%i"' % (mensagem, contagem))

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()