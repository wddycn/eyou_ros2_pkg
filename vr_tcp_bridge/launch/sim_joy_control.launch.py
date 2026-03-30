from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_input_node = Node(
        package='vr_tcp_bridge',
        executable='joy_input',
        name='joy_input',
        output='screen'
    )

    mujoco_node = Node(
        package='vr_tcp_bridge',
        executable='robot_sim',
        name='robot_sim',
        output='screen'
    )

    return LaunchDescription([
        joy_input_node,
        mujoco_node
    ])