![2024-08-21 17-38-41 的屏幕截图](https://github.com/user-attachments/assets/e4a5d39c-f178-436e-869b-2a1e33764cf6)# code_planner

## Expeiments Videos  
We have designed several experimental evironments to validate the capability of our method. Updates are ongoing.  
  
Experiment ① "Narrow Tunnel": https://youtu.be/jENgJ3dgBa0 or https://www.bilibili.com/video/BV1h18YejEWP/    
Experiment ② "Narrow Tunnel: DSVP VS CODE": https://www.youtube.com/watch?v=VOSmzPwXvmc or https://www.bilibili.com/video/BV1vMvKesEBz/  
Experiment ③ "Garage"       : https://www.youtube.com/watch?v=5gJdlG9kfTY or https://www.bilibili.com/video/BV14DeZe9Ev3/
## Requirement
Ubuntu20.04  
Ros2 Foxy
## Simulation environment  
①garage with part of it's driveway destoryed: https://github.com/ssssuxin/autonomous_navigation_environment_ego  
②narrow tunnel: https://github.com/ssssuxin/autonomous_navigation_environment_ros1  

## Usages  
Please follow step "Simulation environment" to install experimental environments first.
### Garage  
in env_ego  

    ros2 launch ego_planner uav_new_garage.launch.py  
    ros2 launch visualization_tools visualization_tools.launch.py  
    (这里visualization_tools.launch.py需要解注释才能用，后续再传一个上去)  
in planner  

    ros2 launch uav_planner 5_21_new_garage_invalid.launch.py  
### Narrow Tunnel  
in env_ros1  

    roslaunch vehicle_simulator narrow_tunnel.launch  
in env_ego  

    ros2 launch visualization_tools visualization_tools.launch.py   
in planner  

    ros2 launch uav_planner explore_tunnel_5_20_guv.launch.py 
in a new terminate  

    souce /opt/ros/noetic/setup.bash
    souce /opt/ros/foxy/setup.bash
    ros2 run ros1_bridge dynamic_bridge   
note: you cloud install ros1_bridge by apt-get or https://github.com/ros2/ros1_bridge

## Results of Tests in Multi-Layer environment

![Uploading 2024-08-21 17-38-41 的屏幕截图.png…]()
![4](https://github.com/user-attachments/assets/35003bf1-c5a4-4447-bace-52876e887d7d)
![2024-08-21 17-38-26 的屏幕截图](https://github.com/user-attachments/assets/50668b63-0882-4705-b65f-c306adbb5e36)
![final_2](https://github.com/user-attachments/assets/4dc56c65-c8d5-4534-abe4-198731e6c45d)


![2024-08-21 17-36-01 的屏幕截图](https://github.com/user-attachments/assets/60b07ed3-48e2-42f3-8b84-651dad9b65d0)

![2024-08-21 17-35-49 的屏幕截图](https://github.com/user-attachments/assets/647ba26a-7a4a-489c-a82b-0a8cf26367d7)
![1](https://github.com/user-attachments/assets/632e14ce-6e95-4aaa-88b9-df3a304858e4)
![2024-08-21 17-35-41 的屏幕截图](https://github.com/user-attachments/assets/e93a805a-c7be-4aac-8f71-addd8376c67e)

![2024-08-21 17-39-07 的屏幕截图](https://github.com/user-attachments/assets/c789df3a-129b-47bd-92f2-9e8ed34d7e77)

