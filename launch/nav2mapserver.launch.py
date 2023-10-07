from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Set the file paths and parameters for the map server
    pkg_dir = '/home/thhiep/ros2_ws/src/ros2demo'
    map_file = pkg_dir + '/maps/map.pgm'
    yaml_file = pkg_dir + '/maps/map.yaml'
    resolution = 0.05
    origin_x = 0.0
    origin_y = 0.0
    origin_z = 0.0
    
    # Create the map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        arguments=[map_file],
        parameters=[{
            'yaml_filename': yaml_file,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'origin_z': origin_z
        }]
    )
    
    # Create the RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', pkg_dir + '/rviz/mapserver.rviz']
    )

    # Create the launch description with the nodes
    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(rviz2_node)
    
    return ld
