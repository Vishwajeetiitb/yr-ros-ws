#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
# import tf_transformations # Not used in this script
import tf2_ros

# If needed, you can use transforms3d for quaternion operations, e.g., converting to Euler angles
# from transforms3d.quaternions import quat2mat, mat2euler

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Update this to your IMU topic
            self.imu_callback,
            10)
        self.br = tf2_ros.TransformBroadcaster(self)
        
    def imu_callback(self, msg):
        t = TransformStamped()
        
        # Customize the frame_id names as per your requirement
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_frame'
        
        t.transform.rotation = msg.orientation
        
        # Example of using transforms3d to convert quaternion to Euler (if needed)
        # quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # matrix = quat2mat(quaternion)
        # euler = mat2euler(matrix)
        # print("Euler angles:", euler)
        
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    imu_visualizer = IMUVisualizer()
    rclpy.spin(imu_visualizer)
    imu_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
