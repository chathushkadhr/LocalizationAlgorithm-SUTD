import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

 
 
    
def generate_launch_description():
    
    pkg_dir = get_package_share_directory("best_vertices")

    #Declare arguments
    declare_arg_n_robots = DeclareLaunchArgument("n_robots",
        default_value = "4",
        description ='Number of robots')
    
    # declare_arg_human_pos = DeclareLaunchArgument('human_pos',
    #     default_value= "[2,0]",
    #     description='Position of the human')
        
    # declare_arg_radius = DeclareLaunchArgument('radius',
    #     default_value= "10",
    #     description='Radius')
        
    # declare_arg_env = DeclareLaunchArgument('env',
    #     default_value= "room_lvl7",
    #     description='Environment')
        
    # declare_arg_update_rate_info = DeclareLaunchArgument('update_rate_info',
    #     default_value= "1.0",
    #     description='Information update rate')       
                
    # Create Launch configuratios
    n_robots = LaunchConfiguration("n_robots")
    # human_pos = LaunchConfiguration('human_pos')
    # radius = LaunchConfiguration('radius')
    # env = LaunchConfiguration('env')
    # update_rate_info = LaunchConfiguration('update_rate_info')
    

    bv_node= Node(
            package='best_vertices',
            # namespace='turtlesim1',
            executable='exec_py',
            name='best_vertices_node',
            output="screen",
            #parameters=[ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
            #prefix=['xterm -e gdb -ex run --args']
        # arguments=['--ros-args', '--log-level', 'debug'],
        #emulate_tty=True)
            
        )
   
    
    ld = LaunchDescription()
    ld.add_action(bv_node)
    ld.add_action(declare_arg_n_robots)
    # ld.add_action(declare_arg_human_pos)
    # ld.add_action(declare_arg_radius)
    # ld.add_action(declare_arg_env)
    # ld.add_action(declare_arg_update_rate_info)

    
    return ld
