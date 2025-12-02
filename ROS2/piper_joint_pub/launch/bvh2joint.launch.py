from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     parameters=[{"robot_description": open("your_bvh.urdf").read()}],
        # ),

        Node(
            package="piper_joint_pub",
            executable="bvh2joint",
            parameters=[{"file_path": "/home/agilex/ros2_project/piper_dancer_ws/src/piper_joint_pub/config/mamo_piper.bvh"}],
        ),
    ])
