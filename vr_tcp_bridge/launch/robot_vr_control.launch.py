from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vr_input_node = Node(
        package='vr_tcp_bridge',
        executable='vr_input',
        name='vr_input',
        output='screen'
    )

    pc2arm_node = Node(
        package='vr_tcp_bridge',
        executable='pc2arm',
        name='pc2arm',
        output='screen'
    )

    return LaunchDescription([
        vr_input_node,
        pc2arm_node
    ])