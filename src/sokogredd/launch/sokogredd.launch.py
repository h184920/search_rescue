import launch
import launch_ros.actions
import os 
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            default_value='tb3_5'
        )
    
    return launch.LaunchDescription([
        #ifølge en random thread jeg fant i robotoic.stackexchange.com kan man refferer til en .launch fil i en .launch fil slik som dette, gjerne endre om noen vet om en bedre måte, for dette virker komplisert:) 
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(
                "bug2_navigation"), "launch", "controller.launch.py"))
                ),
        launch_ros.actions.Node(
            package='sokogredd',
            executable='coordinator',
            name='coordinator'),
        launch_ros.actions.Node(
            package='sokogredd',
            executable='detection',
            namespace='detection'),
        namespace_launch_arg,
        launch_ros.actions.Node(
            package='sokogredd',
            executable='actionHandler',
            parameters=[{'namespace':'tb3_0'}]
        ), 
        launch_ros.actions.Node(
            package='sokogredd',
            executable='actionHandler',
            parameters=[{'namespace':'tb3_1'}]
                )
        # launch_ros.actions.Node(
        #     package='sokogredd',
        #     executable='markerBro',
        #     name='markerBro',
        #     namespace='tb3_0',
        #     parameters=[{'tb_id': 0}]),
        # launch_ros.actions.Node(
        #     package='sokogredd',
        #     executable='markerBro',
        #     name='markerBro',
        #     namespace='tb3_1',
        #     parameters=[{'tb_id': 1}]),



  ])