from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='fastbot_waypoints', executable='fastbot_action_server', output='screen'),
    ])
