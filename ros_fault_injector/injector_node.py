import random
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan, Imu

class FaultInjectorNode(Node):
    """
    ROS2 Fault Injector Node for Chaos Engineering in Robotics.
    Injects deterministic or stochastic latency, packet loss, and Gaussian/impulse noise
    into standard ROS2 message streams (Pose, Twist, LaserScan, Imu).
    """
    def __init__(self):
        super().__init__('fault_injector_node')

        # Declare ROS2 Parameters
        self.declare_parameter('input_topic', '/input_topic')
        self.declare_parameter('output_topic', '/output_topic')
        self.declare_parameter('msg_type', 'Pose')  # Pose, Twist, LaserScan, Imu, String
        self.declare_parameter('fault_type', 'delay')  # delay, drop, noise, corrupt
        self.declare_parameter('delay_ms', 100)
        self.declare_parameter('drop_probability', 0.1)
        self.declare_parameter('noise_std_dev', 0.05)
        self.declare_parameter('salt_pepper_prob', 0.02)

        # Retrieve Initial Parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.msg_type_str = self.get_parameter('msg_type').value
        self.fault_type = self.get_parameter('fault_type').value
        self.delay_ms = self.get_parameter('delay_ms').value
        self.drop_probability = self.get_parameter('drop_probability').value
        self.noise_std_dev = self.get_parameter('noise_std_dev').value
        self.salt_pepper_prob = self.get_parameter('salt_pepper_prob').value

        # Parameter Change Callback
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Message Type Mapping
        self.msg_class = self._get_msg_class(self.msg_type_str)

        # Delay Queue for thread-safe ROS2 native timer scheduling
        self.delay_queue = deque()

        # Publisher & Subscriber Initialization
        self.pub = self.create_publisher(self.msg_class, self.output_topic, 10)
        self.sub = self.create_subscription(self.msg_class, self.input_topic, self.listener_callback, 10)

        # ROS2 Native Timer for processing delayed messages cleanly (100Hz tick)
        self.timer = self.create_timer(0.01, self._process_delay_queue)

        self.get_logger().info(
            f"Fault Injector operational: [{self.input_topic}] -> [{self.output_topic}] "
            f"(Type: {self.msg_type_str}, Mode: {self.fault_type})"
        )

    def _get_msg_class(self, type_name: str):
        mapping = {
            'Pose': Pose,
            'Twist': Twist,
            'LaserScan': LaserScan,
            'Imu': Imu,
            'String': String
        }
        return mapping.get(type_name, String)

    def _on_parameter_change(self, params):
        for param in params:
            if param.name == 'fault_type':
                self.fault_type = param.value
            elif param.name == 'delay_ms':
                self.delay_ms = param.value
            elif param.name == 'drop_probability':
                self.drop_probability = param.value
            elif param.name == 'noise_std_dev':
                self.noise_std_dev = param.value
            elif param.name == 'salt_pepper_prob':
                self.salt_pepper_prob = param.value
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        """Processes incoming messages based on configured fault strategy."""
        # 1. Packet Drop Simulation
        if self.fault_type == 'drop':
            if random.random() < self.drop_probability:
                self.get_logger().debug("Fault injected: Message dropped")
                return

        # 2. Latency Injection (ROS2 Timer Queue)
        if self.fault_type == 'delay':
            release_time = self.get_clock().now().nanoseconds + int(self.delay_ms * 1e6)
            self.delay_queue.append((release_time, msg))
            return

        # 3. Sensor Noise Injection
        if self.fault_type == 'noise':
            msg = self._inject_noise(msg)

        self.pub.publish(msg)

    def _process_delay_queue(self):
        """Drains ready delayed messages using ROS2 clock timestamps."""
        now_ns = self.get_clock().now().nanoseconds
        while self.delay_queue and self.delay_queue[0][0] <= now_ns:
            _, msg = self.delay_queue.popleft()
            self.pub.publish(msg)

    def _inject_noise(self, msg):
        """Injects domain-specific noise into Pose, Twist, LaserScan, or Imu messages."""
        std_dev = self.noise_std_dev

        if isinstance(msg, Pose):
            msg.position.x += random.gauss(0, std_dev)
            msg.position.y += random.gauss(0, std_dev)
            msg.position.z += random.gauss(0, std_dev)
            # Add small quaternion jitter while maintaining approximate normalization
            msg.orientation.z += random.gauss(0, std_dev * 0.1)
            msg.orientation.w += random.gauss(0, std_dev * 0.1)
            norm = math.sqrt(msg.orientation.x**2 + msg.orientation.y**2 + msg.orientation.z**2 + msg.orientation.w**2)
            if norm > 0:
                msg.orientation.x /= norm
                msg.orientation.y /= norm
                msg.orientation.z /= norm
                msg.orientation.w /= norm

        elif isinstance(msg, Twist):
            msg.linear.x += random.gauss(0, std_dev)
            msg.linear.y += random.gauss(0, std_dev)
            msg.angular.z += random.gauss(0, std_dev)

        elif isinstance(msg, LaserScan):
            new_ranges = []
            for r in msg.ranges:
                if math.isnan(r) or math.isinf(r):
                    new_ranges.append(r)
                elif random.random() < self.salt_pepper_prob:
                    # Salt & Pepper impulse noise: dropout or max range spike
                    new_ranges.append(msg.range_max if random.random() > 0.5 else 0.0)
                else:
                    noisy_r = max(msg.range_min, min(msg.range_max, r + random.gauss(0, std_dev)))
                    new_ranges.append(noisy_r)
            msg.ranges = new_ranges

        elif isinstance(msg, Imu):
            msg.linear_acceleration.x += random.gauss(0, std_dev)
            msg.linear_acceleration.y += random.gauss(0, std_dev)
            msg.linear_acceleration.z += random.gauss(0, std_dev)
            msg.angular_velocity.x += random.gauss(0, std_dev * 0.5)
            msg.angular_velocity.y += random.gauss(0, std_dev * 0.5)
            msg.angular_velocity.z += random.gauss(0, std_dev * 0.5)

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
