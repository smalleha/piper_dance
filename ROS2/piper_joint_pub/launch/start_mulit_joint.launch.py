from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piper_joint_pub',
            executable='piper_joint_pub_eighteen',
            name='piper_joint_pub_eighteen',
            output='screen',
            parameters=[
                {'arm_count': 18},
                {'yaml_dir': '/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/piper/dance_pose_v1'},
            ]
        )
    ])
