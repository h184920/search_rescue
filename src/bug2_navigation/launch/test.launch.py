import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            default_value='tb3_5'
        )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='go_to_point',
            parameters=[{'namespace':'tb3_0'}]),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller',
            parameters=[{'namespace':'tb3_0'}]),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='wallfollower_controller',
            parameters=[{'namespace':'tb3_0'}])
  ])