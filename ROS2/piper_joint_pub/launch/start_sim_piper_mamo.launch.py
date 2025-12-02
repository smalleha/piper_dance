from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # === 多机械臂关节控制节点 ===
        Node(
            package='piper_joint_pub',
            executable='piper_joint_pub_tenth',
            name='piper_joint_pub_tenth',
            output='screen',
        ),

        # Node(
        #     package='piper_joint_pub',         
        #     executable='read_mp3',             
        #     name='piper_audio_player',             
        #     output='screen',
        # )
    ])
