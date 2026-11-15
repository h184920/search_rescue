import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='go_to_point',
            namespace='tb3_0'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='wallfollower_controller',
            namespace='tb3_1'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller',
            namespace='tb3_0'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='go_to_point',
            namespace='tb3_1'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='wallfollower_controller',
            namespace='tb3_0'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller',
            namespace='tb3_1'),
        
        #launch_ros.actions.Node(
            #package='bug2_navigation',
            #executable='test_action_server',
            #name='test_action_server'),
  ])