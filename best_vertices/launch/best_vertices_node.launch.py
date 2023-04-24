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
    declare_arg_n_robots = DeclareLaunchArgument("n_robots",
        default_value = "6",
        description ='Number of robots')
    
    declare_arg_human_pos = DeclareLaunchArgument('human_pos',
        default_value= "[6.0,28.0]",
        description='Position of the human')
        
    declare_arg_radius = DeclareLaunchArgument('radius',
        default_value= "12.0",
        description='Radius')
        
    declare_arg_env = DeclareLaunchArgument('env',
        default_value= "room_lvl7",
        description='Environment')

    
    declare_arg_path = DeclareLaunchArgument('path',
        default_value= pkg_dir,
        description='path to the best_vertices package')   
     
    declare_arg_update_rate_info = DeclareLaunchArgument('update_rate_info',
        default_value= "1.0",
        description='Information update rate')       
    
    
                
    #Create Launch configuratios
    n_robots = LaunchConfiguration("n_robots")
    human_pos = LaunchConfiguration('human_pos')
    radius = LaunchConfiguration('radius')
    env = LaunchConfiguration('env',default="room_lvl7")
    path = LaunchConfiguration('path')
    update_rate_info = LaunchConfiguration('update_rate_info')
    

    bv_node= Node(
            package='best_vertices',
            executable='exec_py',
            name='best_vertices_node',
            output="screen",
            parameters=[ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
            #prefix=['xterm -e gdb -ex run --args']
        # arguments=['--ros-args', '--log-level', 'debug'],
        #emulate_tty=True)
            
        )
    
    ms_node = Node( 
        package = 'nav2_map_server',
        executable = "map_server",
        name = "map_server",
        output = "screen",
        parameters=[ParameterFile(os.path.join(pkg_dir, 'config', 'map_server_param.yaml'), allow_substs=True)],
        )        
        
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz.launch.py')),
        condition=IfCondition("True"),
        )
    
    graph_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'graph_generator.launch.py')),
        
        launch_arguments={'cfg': env}.items()
    
        )
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_n_robots)
    ld.add_action(declare_arg_human_pos)
    ld.add_action(declare_arg_radius)
    ld.add_action(declare_arg_env)
    ld.add_action(declare_arg_path)
    ld.add_action(declare_arg_update_rate_info)
    
    ld.add_action(graph_cmd)
    ld.add_action(bv_node)
    ld.add_action(ms_node)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(rviz_cmd)
    
    return ld
