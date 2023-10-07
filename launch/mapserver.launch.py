import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

robot_length = 0.4;
robot_width = 0.4;
safe_margin = 0.02;
stop_zone = 0.3;
collision_margin = 0.02;

pkg_name = 'ros2demo'
pkg_dir = FindPackageShare(package=pkg_name).find(pkg_name)

map_name = 'map50';

map_pgm = pkg_dir+'/maps/'+map_name+'.pgm'
assert os.path.isfile(map_pgm), 'Map file not found at '+map_pgm
map_yaml = pkg_dir+'/maps/'+map_name+'.yaml'
assert os.path.isfile(map_yaml), 'Map file not found at '+map_yaml
with open(map_yaml,'r') as stream:
  map_cfg = yaml.safe_load(stream);
#print(map_cfg);

def generate_launch_description():
    # Create the map server node
    map_server_node = Node(
        package='ros2demo',
        executable='mapservernode',
        output='screen',
        parameters=[{
        	'map_pgm':map_pgm,
        	'map_yaml':map_yaml,
        	'map_resol':map_cfg['resolution'],
        	'map_origin_x':map_cfg['origin'][0],
        	'map_origin_y':map_cfg['origin'][1],
        	'map_occupied_thresh':map_cfg['occupied_thresh'],
        	'map_free_thresh':map_cfg['free_thresh'],
        	'combine_ziczac':1,
        	'laserscan_rate':100,
        	'fov':180.0,
        	'angle_inc':2.0,
        	'max_range':10.0,
        	'min_range':0.01,
        	'robot_length': robot_length,
        	'robot_width': robot_width,
        	'safe_margin': safe_margin,
        	'stop_zone': stop_zone,
        	'collision_margin': collision_margin,
        }],
        arguments=[],
    )
    
    # Create the launch description with the nodes
    ld = LaunchDescription()
    ld.add_action(map_server_node)
    
    return ld
    
