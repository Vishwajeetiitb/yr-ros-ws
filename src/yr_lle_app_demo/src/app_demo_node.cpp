/**
 * @file app_demo_node.cpp
 * @author Kevin Shadle@Yrobot
 * @brief 
 * @version 0.1
 * @date 2024-02-24
 * 
 * @copyright Copyright (c) 2024
 * 
 * 
 * This is an example ROS2 node used to publish and subscribe to ROS2 messages of the 
 * exoskeleton. This is a top level application where one can write algorithm to 
 * get all inputs and sends out control commands.
 * 
 * This example demonstrates the followings:
 * - receiving r/hip/joint_state. The ROS2 message is defined in the package
 *   yr_lle_msgs/msg/joint_state.hpp.
 * - receiving r/hip/imu. The ROS2 message a built-in message type.
 * - pubslishing r/hip/joint_cmd. This allows the control commands to be 
 *   sent to the exoskeleton. This topics is subscribed by packages under 
 *   yr_lle_driver and will be automatic passed down to lower level MCUs.
 * 
 * Additionally, statistics are printed periodically.
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "yr_lle_msgs/msg/joint_state.hpp"
#include "yr_lle_msgs/msg/joint_cmd.hpp"
#include "yr_lle_app_demo/message_statistics.h"
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
using namespace std::chrono_literals;

class AppDemoNode : public rclcpp::Node {
 public:
  AppDemoNode(const rclcpp::NodeOptions& options)
    : Node("app_demo_node", options) {
    this->declare_parameter<bool>("print_rx_msgs", false);
    this->get_parameter("print_rx_msgs", print_rx_msgs_);

    joint_state_subscriber_ = this->create_subscription<yr_lle_msgs::msg::JointState>(
        "r/hip/joint_state", 10, std::bind(&AppDemoNode::JointStateCallback, this, std::placeholders::_1));

    joint_cmd_publisher_ = this->create_publisher<yr_lle_msgs::msg::JointCmd>("r/hip/joint_cmd", 10);

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "r/hip/imu", 10, std::bind(&AppDemoNode::ImuCallback, this, std::placeholders::_1));

    stats_timer_ = this->create_wall_timer(1s, std::bind(&AppDemoNode::PrintStats, this));

    cmd_timer_ = this->create_wall_timer(100ms, std::bind(&AppDemoNode::SendJointCmd, this));
  }

 private:
  void JointStateCallback(const yr_lle_msgs::msg::JointState::SharedPtr msg) {
    if (print_rx_msgs_) {
      RCLCPP_INFO(this->get_logger(), "Received JointState: pos=%f, vel=%f, cur=%f, encoder=%d", msg->pos, msg->vel,
                  msg->cur, msg->encoder);
    }
    // Update statistics
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (print_rx_msgs_) {
      RCLCPP_INFO(this->get_logger(), "Received IMU data");
    }
    // Update statistics
  }

  void PrintStats() {
    // Print message statistics
  }

  void SendJointCmd() {
    auto msg = yr_lle_msgs::msg::JointCmd();
    msg.cmd = 0.5;
    msg.cmd_type = yr_lle_msgs::msg::JointCmd::POSITION;
    joint_cmd_publisher_->publish(msg);
  }

  rclcpp::Subscription<yr_lle_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<yr_lle_msgs::msg::JointCmd>::SharedPtr joint_cmd_publisher_;
  
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool print_rx_msgs_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AppDemoNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
