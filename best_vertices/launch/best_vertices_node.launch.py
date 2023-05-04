import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import TimerAction
import launch_ros.actions

    
def generate_launch_description():
    
    pkg_dir = get_package_share_directory("best_vertices")
    launch_dir = os.path.join(pkg_dir, 'launch')
    
    #Declare arguments
        
    declare_arg_env = DeclareLaunchArgument('env',
        default_value= "test_zone",
        description='Environment')

    
    declare_arg_path = DeclareLaunchArgument('path',
        default_value= pkg_dir,
        description='path to the best_vertices package')   
     
                
    #Create Launch configuratios
    env = LaunchConfiguration('env',default="test_zone")
    path = LaunchConfiguration('path')
    namespace = LaunchConfiguration('namespace',default="human")

    bv_node= Node(
            package='best_vertices',
            executable='exec_py',
            name='best_vertices_node',
            namespace=namespace,
            output="screen",
            parameters=[ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
            #prefix=['xterm -e gdb -ex run --args']
        # arguments=['--ros-args', '--log-level', 'debug'],
        #emulate_tty=True)
            
        )
    rtg_node= Node(
        package='robots_to_goals',
        executable='exec_py',
        name='robots_to_goals_node',
        namespace=namespace,
        output="screen",
        #prefix=['xterm -e gdb -ex run --args']
    # arguments=['--ros-args', '--log-level', 'debug'],
    #emulate_tty=True)
        
    )

    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz.launch.py')),
        condition=IfCondition("True"),
        )
    
    graph_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'graph_generator.launch.py')),
        
        launch_arguments={'cfg': env, 'namespace':namespace}.items()
    
        )
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_env)
    ld.add_action(declare_arg_path)
    ld.add_action(graph_cmd)
    ld.add_action(bv_node)
    ld.add_action(rtg_node)
    ld.add_action(rviz_cmd)
    
    return ld
