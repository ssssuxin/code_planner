# code_planner

Expeiments Videos
①https://youtu.be/jENgJ3dgBa0  or  https://www.bilibili.com/video/BV1h18YejEWP/



note:

simulation environment

①garage with part of it's driveway destoryed: https://github.com/ssssuxin/autonomous_navigation_environment_ego

②narrow tunnel: https://github.com/ssssuxin/autonomous_navigation_environment_ros1


1.Garage

in env

①ros2 launch ego_planner uav_new_garage.launch.py

②ros2 launch visualization_tools visualization_tools.launch.py 

in planner

③ros2 launch uav_planner 5_21_new_garage_invalid.launch.py



2.Narrow Tunnel

in Ros1 evn

①roslaunch vehicle_simulator narrow_tunnel.launch

②ros2 run ros1_bridge dynamic_bridge 

in Ros2 env

③ros2 launch visualization_tools visualization_tools.launch.py 

in planner

④ros2 launch uav_planner explore_tunnel_5_20_guv.launch.py 
