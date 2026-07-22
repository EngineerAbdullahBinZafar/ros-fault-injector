import math
import sys

# Lightweight standalone mocks if ROS2 is not sourced in current Python environment
try:
    from geometry_msgs.msg import Pose, Twist
    from sensor_msgs.msg import LaserScan, Imu
except ModuleNotFoundError:
    class Vector3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()

    class Twist:
        def __init__(self):
            self.linear = Vector3()
            self.angular = Vector3()

    class LaserScan:
        def __init__(self):
            self.range_min = 0.1
            self.range_max = 10.0
            self.ranges = []

    class Imu:
        def __init__(self):
            self.linear_acceleration = Vector3()
            self.angular_velocity = Vector3()
            self.orientation = Quaternion()

    # Mock rclpy modules
    class MockNode:
        def get_logger(self):
            class Logger:
                def info(self, m): pass
                def debug(self, m): pass
                def warn(self, m): pass
                def error(self, m): pass
            return Logger()
        def declare_parameter(self, n, v): pass
        def get_parameter(self, n):
            class Param:
                value = 'Pose'
            return Param()
        def add_on_set_parameters_callback(self, cb): pass
        def create_publisher(self, c, t, q): pass
        def create_subscription(self, c, t, cb, q): pass
        def create_timer(self, i, cb): pass

    sys.modules['rclpy'] = type(sys)('rclpy')
    sys.modules['rclpy.node'] = type(sys)('rclpy.node')
    sys.modules['rclpy.node'].Node = MockNode
    sys.modules['rcl_interfaces.msg'] = type(sys)('rcl_interfaces.msg')
    sys.modules['rcl_interfaces.msg'].SetParametersResult = lambda successful=True: True
    sys.modules['std_msgs.msg'] = type(sys)('std_msgs.msg')
    sys.modules['std_msgs.msg'].String = str
    sys.modules['geometry_msgs.msg'] = type(sys)('geometry_msgs.msg')
    sys.modules['geometry_msgs.msg'].Pose = Pose
    sys.modules['geometry_msgs.msg'].Twist = Twist
    sys.modules['sensor_msgs.msg'] = type(sys)('sensor_msgs.msg')
    sys.modules['sensor_msgs.msg'].LaserScan = LaserScan
    sys.modules['sensor_msgs.msg'].Imu = Imu

from ros_fault_injector.injector_node import FaultInjectorNode

def test_msg_type_resolution():
    node = FaultInjectorNode.__new__(FaultInjectorNode)
    assert node._get_msg_class('Pose') == Pose
    assert node._get_msg_class('Twist') == Twist
    assert node._get_msg_class('LaserScan') == LaserScan
    assert node._get_msg_class('Imu') == Imu

def test_pose_noise_injection():
    node = FaultInjectorNode.__new__(FaultInjectorNode)
    node.noise_std_dev = 0.1
    
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.w = 1.0
    
    noisy_pose = node._inject_noise(pose)
    assert noisy_pose.position.x != 1.0 or noisy_pose.position.y != 2.0
    norm = math.sqrt(noisy_pose.orientation.x**2 + noisy_pose.orientation.y**2 + noisy_pose.orientation.z**2 + noisy_pose.orientation.w**2)
    assert abs(norm - 1.0) < 1e-4

def test_laserscan_noise_injection():
    node = FaultInjectorNode.__new__(FaultInjectorNode)
    node.noise_std_dev = 0.05
    node.salt_pepper_prob = 0.0
    
    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
    
    noisy_scan = node._inject_noise(scan)
    assert len(noisy_scan.ranges) == 5
    for r in noisy_scan.ranges:
        assert 0.1 <= r <= 10.0
