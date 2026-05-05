import rclpy
from rclpy.node import Node
import time
import random
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan, Imu

class FaultInjectorNode(Node):
    def __init__(self):
        super().__init__('fault_injector_node')
        
        # Parameters
        self.declare_parameter('input_topic', '/input_topic')
        self.declare_parameter('output_topic', '/output_topic')
        self.declare_parameter('msg_type', 'Pose') # Support Pose, Twist, LaserScan, etc.
        self.declare_parameter('fault_type', 'delay') # delay, drop, noise
        self.declare_parameter('delay_ms', 100)
        self.declare_parameter('drop_probability', 0.1) # 10%
        self.declare_parameter('noise_std_dev', 0.05)
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.msg_type_str = self.get_parameter('msg_type').value
        self.fault_type = self.get_parameter('fault_type').value
        
        # Determine Message Type
        self.msg_class = self._get_msg_class(self.msg_type_str)
        
        # Publisher and Subscriber
        self.pub = self.create_publisher(self.msg_class, self.output_topic, 10)
        self.sub = self.create_subscription(self.msg_class, self.input_topic, self.listener_callback, 10)
        
        self.get_logger().info(f'Fault Injector started: {self.input_topic} -> {self.output_topic} with {self.fault_type} fault')

    def _get_msg_class(self, type_name):
        if type_name == 'Pose': return Pose
        if type_name == 'Twist': return Twist
        if type_name == 'LaserScan': return LaserScan
        if type_name == 'Imu': return Imu
        return String

    def listener_callback(self, msg):
        if self.fault_type == 'drop':
            if random.random() < self.get_parameter('drop_probability').value:
                self.get_logger().warn('Message dropped')
                return
        
        if self.fault_type == 'delay':
            delay = self.get_parameter('delay_ms').value / 1000.0
            threading.Timer(delay, self.publish_msg, [msg]).start()
            return
            
        if self.fault_type == 'noise':
            msg = self._inject_noise(msg)
            
        self.publish_msg(msg)

    def publish_msg(self, msg):
        self.pub.publish(msg)

    def _inject_noise(self, msg):
        std_dev = self.get_parameter('noise_std_dev').value
        # Simple noise injection for Pose/Twist
        if isinstance(msg, Pose):
            msg.position.x += random.gauss(0, std_dev)
            msg.position.y += random.gauss(0, std_dev)
            msg.position.z += random.gauss(0, std_dev)
        elif isinstance(msg, Twist):
            msg.linear.x += random.gauss(0, std_dev)
            msg.angular.z += random.gauss(0, std_dev)
        # More complex types could be added here
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
