from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_nodes(context):
    """动态生成多个 Piper 控制节点"""
    num_arms = int(LaunchConfiguration('num_arms').perform(context))
    auto_enable = LaunchConfiguration('auto_enable')
    gripper_exist = LaunchConfiguration('gripper_exist')
    gripper_val_mutiple = LaunchConfiguration('gripper_val_mutiple')
    piper_dir = get_package_share_directory('piper')

    nodes = []
    for i in range(1, num_arms + 1):
        ns = f'piper_{i}'
        can_port = f'can{i}'  # 从 can1 开始
        node = Node(
            package='piper',
            executable='piper_single_ctrl',
            name=f'piper_ctrl_node_{i}',
            namespace=ns,
            output='screen',
            parameters=[{
                'can_port': can_port,
                'auto_enable': auto_enable,
                'gripper_exist': gripper_exist,
                'gripper_val_mutiple': gripper_val_mutiple,
            }],
            remappings=[
                ('joint_ctrl_single', f'/{ns}/joint_states')
            ]
        )
        nodes.append(node)
    return nodes


def generate_launch_description():
    # ---- Launch 参数定义 ----
    num_arms_arg = DeclareLaunchArgument(
        'num_arms',
        default_value='4',
        description='Number of Piper arms to launch (starting from can1).'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable Piper node.'
    )
    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Whether gripper exists.'
    )
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='Gripper value multiplier.'
    )

    # ---- 动态创建节点 ----
    create_nodes = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        num_arms_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        create_nodes
    ])
