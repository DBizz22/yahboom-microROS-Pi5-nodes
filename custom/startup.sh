#!/bin/bash

# Delay start by 30 seconds
sleep 30

export ROS_DOMAIN_ID=20
echo -e "MY_DOMAIN_ID: \033[32m$ROS_DOMAIN_ID\033[0m"
source ~/imu_ws/install/setup.bash
source ~/gmapping_ws/install/setup.bash
source ~/yahboomcar_ws/install/setup.bash
source /opt/ros/humble/setup.bash
source /custom//install/setup.bash

echo "Environment sourced successfully"
# systemctl start supervisor

# Start rosbridge after all previous processes finish launching
ros2 run rosbridge_server rosbridge_websocket &
PID1=$!

# Start ROS 2 components in parallel
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py &
PID2=$!

ros2 run yahboomcar_astra2 drive_node &
PID3=$!

# ros2 run usb_cam usb_cam_node_exe &
ros2 run yahboomcar_astra2 camera_node &
PID4=$!

ros2 launch robot_pose_publisher_ros2 robot_pose_publisher_launch.py &
PID5=$!

ros2 launch yahboomcar_nav map_cartographer_launch.py &
PID6=$!

ros2 launch yahboomcar_nav navigation_dwb_launch.py &
PID7=$!

ros2 run yahboomcar_astra2 follow_line & 
PID8=$!

ros2 run yahboomcar_mediapipe2 HandCtrl &
PID9=$!

# ros2 launch yahboomcar_nav display_launch.py &
# PID10=$!

# Wait for all to start
wait $PID1
wait $PID2
wait $PID3
wait $PID4
wait $PID5
wait $PID6
wait $PID7
wait $PID8
wait $PID9
# wait $PID10
