installation

git clone https://github.com/ssssuxin/autonomous_navigation_environment_ego.git

cd autonomous_navigation_environment_ego

colcon build --symlink-install

source install/setup.bash

ros2 launch ego_planner uav_new_garage.launch.py

note: please make sure the path does not contain Chinese characters.
