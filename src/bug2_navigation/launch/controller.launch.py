import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller',
            name='bug2_controller'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='go_to_point',
            name='go_to_point'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='wall_follower',
            name='wall_follower'),
        #launch_ros.actions.Node(
            #package='bug2_navigation',
            #executable='test_action_server',
            #name='test_action_server'),
  ])