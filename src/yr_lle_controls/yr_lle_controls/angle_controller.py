import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')
        self.cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller manager service...')
        
        # Initialize publishers
        self.angle_publisher = self.create_publisher(JointTrajectory, '/yr_angle_controller/joint_trajectory', 10)
        self.torque_publisher = self.create_publisher(Float64MultiArray, '/yr_torque_controller/commands', 10)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.encoder_callback,
            10)
        self.subscription  # prevent unused variable warning

    def encoder_callback(self, msg): 
        measurements = msg.position #['yr_l_pel_joint', 'yr_l_hip_joint', 'yr_l_kne_joint', 'yr_l_ank_joint','yr_r_pel_joint', 'yr_r_hip_joint', 'yr_r_kne_joint', 'yr_r_ank_joint']
        print('Measurements',measurements)

    def call_switch_service(self, activate, deactivate):
        req = SwitchController.Request(
            start_controllers=activate,
            stop_controllers=deactivate,
            strictness=1,
            timeout=Duration(sec=5, nanosec=0)
        )
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Switch successful')
        else:
            self.get_logger().error('Failed to switch controllers')

    def set_angles(self,angles):
        self.call_switch_service(['yr_angle_controller'], ['yr_torque_controller'])
        msg = JointTrajectory()
        msg.joint_names = ['yr_l_pel_joint', 'yr_l_hip_joint', 'yr_l_kne_joint', 'yr_l_ank_joint',
                            'yr_r_pel_joint', 'yr_r_hip_joint', 'yr_r_kne_joint', 'yr_r_ank_joint']
        point = JointTrajectoryPoint()
        
        point.positions = angles
        point.time_from_start = Duration(sec=1, nanosec=1000)
        msg.points.append(point)
        self.angle_publisher.publish(msg)
        self.get_logger().info('Published example angle command.')

    def set_torques(self,torques):
        self.call_switch_service(['yr_torque_controller'], ['yr_angle_controller'])
        msg = Float64MultiArray()
        msg.data = torques
        self.torque_publisher.publish(msg)
        self.get_logger().info('Published example torque command.')

def main(args=None):
    rclpy.init(args=args)
    switcher = ControllerSwitcher()
    pi = 3.14159265359
    # switcher.set_angles([-pi/2,-pi/6,0.0,-pi/4, pi/2,-pi/4, pi/2, -pi/4])  # Example: switch to angle mode and publish command
    switcher.set_torques([-100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rclpy.spin(switcher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
