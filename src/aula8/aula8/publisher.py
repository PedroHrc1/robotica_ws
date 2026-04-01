import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Aula8

class Publisher(Node):
    def __init__(self):
        super().__init__('aula8_publisher')
        self.publisher = self.create_publisher(Aula8, 'aula8_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.contador = 0

    def timer_callback(self):
        self.contador += 1
        msg = Aula8()
        msg.count = self.contador
        msg.message = 'A contagem é: '
        self.publisher.publish(msg)
        self.get_logger().info('Publicando: "%s%i"' % (msg.message, msg.count))

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()