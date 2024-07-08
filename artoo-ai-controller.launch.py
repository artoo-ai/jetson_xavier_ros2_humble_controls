# artoo-ai-controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'joy_config': 'xbox360',
                #'config_filepath': '/ros2_ws/install/teleop_twist_joy/share/teleop_twist_joy/config/xbox360.config.yaml'
            }],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='artoo_ai_controller',
            executable='artoo-ai_controller_node',
            name='pwm_controller',
            output='screen',
            parameters=[
                {'i2c_bus_number': 8}
            ]
        )
    ])