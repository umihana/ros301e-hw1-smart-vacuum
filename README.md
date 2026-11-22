# Smart Vacuum Cleaner – ROS 2 (ROS301E Homework 1)

A simulated home vacuum cleaner in ROS 2 Humble, written in C++.

The system has three nodes:
- hardware_driver_node: publishes the robot status on /vacuum/status (1 Hz)
- map_loader_node: service /vacuum/set_floor_level to load a floor map
- task_manager_node: action /vacuum/clean_session to run a cleaning session

The custom msg, srv and action files are in the package smart_vacuum_interfaces.

## Build

    cd ~/vacuum_ws
    colcon build
    source install/setup.bash

## Run

    ros2 launch smart_vacuum vacuum_system.launch.xml

## Test (in a second terminal)

    ros2 topic echo /vacuum/status
    ros2 service call /vacuum/set_floor_level smart_vacuum_interfaces/srv/SetFloorLevel "{floor_id: 'Floor_1'}"
    ros2 action send_goal /vacuum/clean_session smart_vacuum_interfaces/action/CleanSession "{mode: 1, power_mode: 2}" --feedback

Cancel a running goal:

    ros2 service call /vacuum/clean_session/_action/cancel_goal action_msgs/srv/CancelGoal "{}"

mode: 0 = MAP_ONLY, 1 = CLEAN_EXISTING_MAP, 2 = SLAM_AND_CLEAN
power_mode: 0 = ECO, 1 = STANDARD, 2 = TURBO

