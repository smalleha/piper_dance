from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 需要启动的多个 piper 手
    hand_list = [1, 2, 3, 4, 5, 6, 7]

    nodes = []

    for i in hand_list:
        topic_name = f"/piper_{i}/hand_cmd"

        nodes.append(
            Node(
                package="hand_cmd_pub",
                executable="multi_hand_cmd",
                name=f"hand_cmd_piper_{i}",
                output="screen",
                parameters=[
                    {"topic_name": topic_name}
                ]
            )
        )

    return LaunchDescription(nodes)
