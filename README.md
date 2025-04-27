# Robot Arm - PX100

## Launch PX100 simulation:

ros2 launch interbotix_xsarm_sim xsarm_gz_classic.launch.py robot_model:=px100

don't need Moveit yet.

## Run simple_control.py:

ros2 run px100_test simple_control

Test GUI topic messages:

ros2 topic pub /GUI std_msgs/msg/String "data: '{\"click\": 1, \"motion\": \"lifting\", \"channel\": \"3\"}'" --once

## Rebuild

cd ~/interbotix_ws

colcon build

source install/setup.bash