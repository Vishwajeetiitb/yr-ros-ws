#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import os
import sys
package_directory = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, package_directory)
print(sys.path)

from walk_data import WalkData, Angles

class JointStatePublisher(Node):
    
    def __init__(self):
        super().__init__('publish_joint_states')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.angle = 0
        self.wd = WalkData()
        self.ankle_offset = math.pi * 0.5
        self.get_logger().info('Initialized [publish_joint_states] JointStatePublisher')
        self.step_count = 0
        
    def get_new_angles(self):
        steps = len(self.wd.list)
        idx = self.step_count % steps
        idx2 = (self.step_count + (steps // 2 - 1)) % steps
        ang = self.wd.list[idx]
        ang2 = self.wd.list[idx2]
        self.step_count += 1
        if self.step_count % 10 == 0:
            self.get_logger().info(f'get_new_angles: [{self.step_count}] [{steps}]')
        
        # print(f"{TAG}: L:[{self.step_count}][0][{idx}],h:{ang.h},k:{ang.k},a:{ang.a}[{ankle_offset + ang.a}],{ang.as_string()}")
        # print(f"{TAG}: R:[{self.step_count}][1],h:{ang2.h},k:{ang2.k},a:{ang2.a}[{ankle_offset + ang2.a}]")
        
        return [ang, ang2] 

    def timer_callback(self):
        angles = self.get_new_angles() 
        for i, side in enumerate(['l', 'r']):
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = [f'{side}_hip_joint', f'{side}_knee_joint', f'{side}_ankle_joint']
           
            # simulated walking data 
            ang = angles[i]
            # c = math.pi / 180.0 
            c = -1
            h_o = -0.0
            k_o = 0.0
            joint_state.position = [h_o + ang.h * c, ang.k * -c, ang.a * c + self.ankle_offset] 
            # if i == 1:
            #     joint_state.position = [0, 0, 0] 
          
            # sine wave 
            angle_offset = math.pi if side == 'l' else 0
            angle = math.sin(self.angle + angle_offset)
            # joint_state.position = [angle * 0.5, angle * 0.2, math.pi/2]
            # joint_state.position = [angle, 0, 0]
            
            joint_state.velocity = []
            joint_state.effort = []

            self.publisher_.publish(joint_state)
            self.angle += 0.01

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()