from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_fault_injector',
            executable='injector_node',
            name='chaos_monkey',
            parameters=[{
                'input_topic': '/cmd_vel',
                'output_topic': '/cmd_vel_noisy',
                'msg_type': 'Twist',
                'fault_type': 'noise',
                'noise_std_dev': 0.1
            }]
        )
    ])
