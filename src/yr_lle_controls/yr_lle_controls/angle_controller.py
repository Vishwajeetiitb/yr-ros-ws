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

    def set_angles(self,angles):
        activate = ['yr_angle_controller']
        deactivate = ['yr_torque_controller']

        req = SwitchController.Request(
            start_controllers=activate, 
            stop_controllers=deactivate, 
            strictness=2, 
            # start_asap=True, 
            timeout=Duration(sec=3, nanosec=0)
        )
        self.future = self.cli.call_async(req)
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
        deactivate = ['yr_angle_controller']
        activate = ['yr_torque_controller']

        req = SwitchController.Request(
            start_controllers=activate, 
            stop_controllers=deactivate, 
            strictness=2, 
            # start_asap=True, 
            timeout=Duration(sec=0, nanosec=0)
        )
        self.future = self.cli.call_async(req)
        msg = Float64MultiArray()
        msg.data = torques
        self.torque_publisher.publish(msg)
        self.get_logger().info('Published example torque command.')

def main(args=None):
    rclpy.init(args=args)
    switcher = ControllerSwitcher()
    pi = 3.14159265359
    switcher.set_angles([-pi/2,-pi/6,pi/2,-pi/4, pi/2,-pi/4, pi/2, -pi/4])  # Example: switch to angle mode and publish command
    # switcher.set_torques([-10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rclpy.spin(switcher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
