## Requirement
Ubuntu20.04  
Ros2 Foxy

## introduction  
This repositories is the descendence of https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git. We have make a Ros2 Foxy version deriving from that Ros1 version. And it's specifically designed for UAV exploration simulation. We use ego-planner for navigation module https://github.com/ZJU-FAST-Lab/ego-planner.

## Installation  
    git clone https://github.com/ssssuxin/autonomous_navigation_environment_ego.git  
    cd autonomous_navigation_environment_ego  
    colcon build --symlink-install  
    source install/setup.bash  
    ros2 launch ego_planner uav_new_garage.launch.py  
note: please make sure the path does not contain Chinese characters.
