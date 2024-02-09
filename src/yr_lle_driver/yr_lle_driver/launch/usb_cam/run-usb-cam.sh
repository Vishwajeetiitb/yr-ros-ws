#!/usr/bin/env bash

try_1() {
    ros2 run usb_cam usb_cam_node_exe --ros-args --params-file usb_cam_params.yaml &
    ros2 run image_view image_view image:=image_raw
}

# Function to kill background processes
cleanup() {
    echo "Stopping ROS2 nodes..."
    kill %1 %2
}

# Trap Ctrl+C (Interrupt signal) and call cleanup function
trap cleanup INT

# Start the usb_cam node in the background
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file $(pwd)/usb_cam_params.yaml &

# Start the image_view node in the background
ros2 run image_view image_view image:=image_raw &

# Wait for both processes to end
wait
