import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import TimerAction
import launch_ros.actions
 
 
    
def generate_launch_description():
    
    pkg_dir = get_package_share_directory("best_vertices")
    #Declare arguments

  
    declare_arg_custom_graph_path = DeclareLaunchArgument('custom_graph_path',
        default_value= str(""),
        description='custom_graph_path')

    
    declare_arg_path = DeclareLaunchArgument('path',
        default_value= pkg_dir,
        description='path to the best_vertices package') 
     
    declare_arg_cfg = DeclareLaunchArgument('cfg',
        default_value= "room_lvl7" ,
        description='cfg')      
           
    #Create Launch configuratios
    
    custom_graph_path = LaunchConfiguration('custom_graph_path')    
    path = LaunchConfiguration('path')
    cfg = LaunchConfiguration('cfg')
    namespace = LaunchConfiguration('namespace')
    
    gg_node= Node(
            package='tuw_voronoi_graph',
            executable='exec_graph',
            name='graph_generator',
            namespace=namespace,
            output="screen",
            parameters=[ParameterFile(os.path.join(pkg_dir, 'config', 'graph_generator_param.yaml'), allow_substs=True)],
            #prefix=['xterm -e gdb -ex run --args']
        # arguments=['--ros-args', '--log-level', 'debug'],
        #emulate_tty=True)
            
        )
     
        
    
    ld = LaunchDescription()

    ld.add_action(declare_arg_custom_graph_path)
    ld.add_action(declare_arg_path)
    ld.add_action(declare_arg_cfg)

    ld.add_action(gg_node)


    
    return ld
