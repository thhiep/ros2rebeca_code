import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

use_sim_time = True

rviz_config = 'rviz/multi_robots.rviz'

#measurements are in meters
robot_length = 0.4;
robot_width = 0.4;
safe_margin = 0.02;
stop_zone = 0.3;
collision_margin = 0.02;

#(x,y)=start point, (tx,ty)=target point, in pixel coordinates
robots = [
	{'x':5,'y':5,'tx':45,'ty':45,'velocity':0.5,'max_waiting_time':1500,'scan_first':0,'angular_velocity':360.0,},
	{'x':5,'y':15,'tx':45,'ty':35,'velocity':0.7,'max_waiting_time':2000,'scan_first':0,'angular_velocity':360.0,},
	{'x':5,'y':25,'tx':45,'ty':25,'velocity':0.8,'max_waiting_time':2500,'scan_first':0,'angular_velocity':360.0,},
	{'x':5,'y':35,'tx':45,'ty':15,'velocity':0.5,'max_waiting_time':3000,'scan_first':0,'angular_velocity':360.0,},
	{'x':5,'y':45,'tx':45,'ty':5,'velocity':0.3,'max_waiting_time':1500,'scan_first':0,'angular_velocity':360.0,},
];


def launch_single_robot(i):
  rname = 'r' + str(i)	
  robot_radius = max(robot_width,robot_length)/2
  rmodel = 'boxbot.xacro'; 
  if 'model' in robots[i-1]:	
  	rmodel = robots[i-1]['model'];
  		
  xacro_path = pkg_dir+'/urdf/'+rmodel+' rname:='+rname+' rlen:='+str(robot_length)+' rwidth:='+str(robot_width)+' rradius:='+str(robot_radius); 
  urdf_path = pkg_dir+'/urdf/'+rname+'.urdf'  
  #generate urdf file for the robot from the xacro template
  os.system('xacro '+xacro_path + ' > '+urdf_path)
  
  params = robots[i-1]
  params['robot_id'] = i;
  params['robot_length'] = robot_length
  params['robot_width'] = robot_width
  params['safe_margin'] = safe_margin
  params['stop_zone'] = stop_zone
  params['collision_margin'] = collision_margin
  params['map_pgm'] = map_pgm;
  params['map_yaml'] = map_yaml;
  params['map_resol']=map_cfg['resolution']
  params['map_origin_x']=map_cfg['origin'][0]
  params['map_origin_y']=map_cfg['origin'][1]
  params['map_origin_yaw']=map_cfg['origin'][2]
        	
  rnode = Node(
  	namespace=rname,
    package='ros2demo',
    executable='robotnode',
    output='screen',
    parameters=[params],
    arguments=[]
  )
  
  cmd1 = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name = rname+'_robot_state_publisher',
    remappings=[('tf','/'+rname+'/tf'),
    	('tf_static','/'+rname+'/tf_static'),
    	('joint_states','/'+rname+'/joint_states'),
    	('robot_description','/'+rname+'/robot_description')
    ],
    parameters=[{'use_sim_time': use_sim_time, 
    	'tf_prefix': rname,
    	'frame_prefix': rname,
    	'robot_description': Command(['xacro',' ',xacro_path]),
    }],
    arguments=[])
  
  #only need to publish joint_states if having moving parts, 
  #for simplicity we only have a box robot
  cmd2 = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name = rname+'_joint_state_publisher',
    remappings=[('/tf','/'+rname+'/tf'),
    	('joint_states','/'+rname+'/joint_states'),
    	('robot_description','/'+rname+'/robot_description')
    ],
    parameters=[{'use_sim_time': use_sim_time, 
    	'tf_prefix': rname,
    	'frame_prefix': rname,
    	'robot_description': '/'+rname+'/robot_description',
    }])

  return [cmd1,rnode]		

def generate_launch_description():
  
  ld = LaunchDescription()
  
  # Launch RViz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', os.path.join(pkg_dir, rviz_config)
  ])
 
  ld.add_action(rviz_node)
  
  # Create the launch description and populate
  num_robots = len(robots);
  for i in range(1,1+num_robots):
  	nodes = launch_single_robot(i)
  	for node in nodes:
  	  ld.add_action(node)  	
  

  return ld
