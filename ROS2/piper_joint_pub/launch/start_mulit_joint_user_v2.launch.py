from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piper_joint_pub',
            executable='piper_multi_joint_user',
            name='piper_multi_joint_user',
            output='screen',
            parameters=[
                {'num_arms': 7},
                {'base_path': '/home/agilex/piper_ws/src/ROS2/piper_joint_pub/config/piper/user_v2/'},
            ]
        )
    ])
