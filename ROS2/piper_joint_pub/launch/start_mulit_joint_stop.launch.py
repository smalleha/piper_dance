from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piper_joint_pub',
            executable='piper_multi_joint',
            name='piper_multi_joint',
            output='screen',
            parameters=[
                {'num_arms': 2},
                {'base_path': '/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/mamo/piper_mamo_pose_stop/'},
            ]
        )
    ])
