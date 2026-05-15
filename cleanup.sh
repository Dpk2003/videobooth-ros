#!/bin/bash
echo "Stopping all ROS2 processes..."
pkill -f ros2_control_node
pkill -f robot_state_publisher
pkill -f rviz2
pkill -f move_group
pkill -f spawner
sleep 2

echo "Stopping Docker emulator..."
docker kill dsr01_emulator 2>/dev/null
docker rm -f dsr01_emulator 2>/dev/null
sleep 3

echo "Freeing port 12345..."
sudo fuser -k 12345/tcp 2>/dev/null
sleep 2

echo "Restarting ROS2 daemon..."
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 1

echo "Verifying port is free..."
sudo ss -tulnp | grep 12345 && echo "WARNING: port still in use!" || echo "Port 12345 is free"

echo "Done! Safe to launch."
