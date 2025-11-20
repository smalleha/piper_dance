from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 7 个 CAN 端口名称
    can_ports = [
        "piper_1",
        "piper_2",
        "piper_3",
        "piper_4",
        "piper_5",
        "piper_6",
        "piper_7",

    ]

    nodes = []

    for port in can_ports:
        node_name = f"{port}_hand_cmd"

        nodes.append(
            Node(
                package="hand_cmd_pub",
                executable="hand_cmd",
                name=node_name,
                output="screen",
                parameters=[
                    {"can_port": port},
                    {"auto_enable": True},
                    {"gripper_val_mutiple": 1},
                    {"num_hand": 7},
                ],
            )
        )

    return LaunchDescription(nodes)
