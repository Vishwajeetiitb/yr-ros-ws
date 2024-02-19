#include "rclcpp/rclcpp.hpp"
#include "spi_driver.h"
#include <atomic>
#include <chrono>
#include <cmath>  // For std::sin
#include <fstream>
// #include "pb2ros_converter.h"
#include "yr_lle_msgs/msg/joint_cmd.hpp"
#include "yr_lle_msgs/msg/joint_state.hpp"

#include "desc_var_container.h"
#include "topic_manager.h"

#include "protobuf/exo_data.pb.h"  // Protobuf message header
#include "protobuf/channel_descriptor.pb.h"
#include <sensor_msgs/msg/imu.hpp>

constexpr char kSpiDevice[] = "/dev/spidev4.1";
constexpr uint32_t kSpiFrequency = 10000000;

class McuBridgeRosNode : public rclcpp::Node {
 public:
  McuBridgeRosNode()
    : Node("mcu_bridge_ros_node")
    , spi_driver_(kSpiDevice, kSpiFrequency) {
    this->declare_parameter<int>("duration", 10);
    this->declare_parameter<int>("data_size", 1024);
    this->declare_parameter<double>("data_rate_hz", 1000.0);
    this->declare_parameter<std::string>("filename", "spi_stats.csv");

    int duration_ = this->get_parameter("duration").as_int();
    int data_size_ = this->get_parameter("data_size").as_int();
    double data_rate_hz_ = this->get_parameter("data_rate_hz").as_double();
    this->get_parameter("filename", filename_);

    // Print out the parameters in a banner
    std::cout << std::setfill('=') << std::setw(50) << "=" << std::endl;
    std::cout << "SPI Test Configuration:" << std::endl;
    std::cout << "Duration: " << duration_ << " seconds" << std::endl;
    std::cout << "Data Size: " << data_size_ << " bytes" << std::endl;
    std::cout << "Data Rate: " << data_rate_hz_ << " Hz" << std::endl;
    std::cout << std::setfill('=') << std::setw(50) << "=" << std::endl;

    // Open the CSV file
    // csvFile.open(filename_);
    // if (!csvFile.is_open()) {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename_.c_str());
    // }

    joint_cmd_subscriber_ = this->create_subscription<yr_lle_msgs::msg::JointCmd>(
        "/joint_cmd", 10, std::bind(&McuBridgeRosNode::jointCmdCallback, this, std::placeholders::_1));

    // Initialize publisher
    joint_cmd_publisher_ = this->create_publisher<yr_lle_msgs::msg::JointCmd>("/joint_cmd", 10);

    // Publish a single message for testing
    // auto test_msg = yr_lle_msgs::msg::JointCmd();
    // test_msg.cmd = 1.0;
    // test_msg.cmd_type = yr_lle_msgs::msg::JointCmd::POSITION;
    // test_msg.joint = yr_lle_msgs::msg::JointCmd::HIP;
    // test_msg.side = yr_lle_msgs::msg::JointCmd::RIGHT;
    // joint_cmd_publisher_->publish(test_msg);

    spi_driver_.EnableSimulation(true);
    // spi_driver.EnableSimulation(false);
    spi_driver_.Init();

    // Create a high-frequency timer for SPI communication
    auto send_interval_us = std::chrono::microseconds(static_cast<int>(1000000 / data_rate_hz_));
    std::cout << "SendReceiveData thread running interval: " << send_interval_us.count() << " us" << std::endl;
    timer_1000hz_ = this->create_wall_timer(  // std::chrono::milliseconds(1),
        send_interval_us, std::bind(&McuBridgeRosNode::spiSendReceiveTask, this));

    // Create a low-frequency timer for printing statistics
    timer_1hz_ =
        this->create_wall_timer(std::chrono::seconds(1), std::bind(&McuBridgeRosNode::printStatisticsTask, this));
  }

  ~McuBridgeRosNode() {
    // Clean up
    if (csvFile.is_open()) {
      csvFile.close();
    }
  }

  void Init() {
    topicManager_ = std::make_unique<TopicManager>(shared_from_this());
    // topicManager_->CreateDescVarContainers(true);
    topicManager_->CreateDescVarContainers(descVarContainers, TopicManager::MsgType::Imu, true);
    topicManager_->CreateDescVarContainers(descVarContainers, TopicManager::MsgType::JointState, true);
    // topicManager_->CreateImuDescVarContainers(descVarContainers, true);
    topicManager_->PrintDescVarsList(descVarContainers);

    // test incoming desc var from MockMcu
    TestProcessIncomingDescVarArray();
  }

  void TestProcessIncomingDescVarArray() {
    DescVarContainerBase* dvcBase = topicManager_->findByTopicName(descVarContainers, "/l/hip/motor/joint_state");
    auto dvc = dynamic_cast<DescVarContainer<exoskeleton::JointState, yr_lle_msgs::msg::JointState>*>(dvcBase);
    if (dvc) {
      double currentTimeInSeconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      auto& pbMsg = dvc->getPbMsg();
      pbMsg.set_pos(std::sin(currentTimeInSeconds * 1.0));
      pbMsg.set_vel(std::sin(currentTimeInSeconds * 0.5));
      pbMsg.set_cur(std::sin(currentTimeInSeconds * 0.8));
      dvc->setPbMsg(pbMsg);
    }
    exoskeleton::DescVarArray pbArray;
    topicManager_->CreateDescVarPbArray(pbArray, descVarContainers);
    processIncomingDescVar(pbArray);
  }

  void processIncomingDescVar(const exoskeleton::DescVarArray& incomingArray) {
    for (const auto& incomingDescVar : incomingArray.desc_vars()) {
      for (auto& container : descVarContainers) {
        if (container->isMatch(incomingDescVar)) {
          // Match found, log the topic name
          // RCLCPP_INFO(this->get_logger(), "Match found for topic: %s", container->getTopicName().c_str());
          container->handleProtobufMessage(incomingDescVar.data());
        }
      }
    }
  }

 private:
  void jointCmdCallback(const yr_lle_msgs::msg::JointCmd::SharedPtr msg) {
    exoskeleton::JointCmd pb_msg;
    Pb2RosConverter::ConvertRos2Pb(*msg, pb_msg);
    printPbMsg(pb_msg);
  }

  void printPbMsg(const exoskeleton::JointCmd& pb_msg) {
    RCLCPP_INFO(this->get_logger(), "Protobuf JointCmd: Cmd: %f, Cmd Type: %d, Joint: %d, Side: %d", pb_msg.cmd(),
                pb_msg.cmd_type(), pb_msg.joint(), pb_msg.side());
  }

  void sendSpiData() {
    std::vector<uint8_t> testData(data_size_, 0xAA);
    spi_driver_.Send(testData);
  }

  void PrintMessageStats() {
    spi_driver_.PrintMsgStats();
    if (csvFile.is_open()) {
      spi_driver_.PrintMsgStats(csvFile);
    }
  }

  void spiSendReceiveTask() {
    high_freq_counter_++;
    // Print every 100th call (10 Hz)
    if (high_freq_counter_ % 10 == 0) {
      TestProcessIncomingDescVarArray();
    }
    if (high_freq_counter_ % 100 == 0) {
      // RCLCPP_INFO(this->get_logger(), "High Frequency Task: 10 Hz print");
    }
    // sendSpiData();
  }

  // Task for printing SPI communication statistics
  void printStatisticsTask() {
    RCLCPP_INFO(this->get_logger(), "Low Frequency Task: 1 Hz print");
    // PrintMessageStats();
    // TestProcessIncomingDescVarArray();
  }

  rclcpp::Subscription<yr_lle_msgs::msg::JointCmd>::SharedPtr joint_cmd_subscriber_;
  rclcpp::Publisher<yr_lle_msgs::msg::JointCmd>::SharedPtr joint_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_1000hz_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  std::vector<std::unique_ptr<DescVarContainerBase>> descVarContainers;
  std::unique_ptr<TopicManager> topicManager_;
  SPIDriver spi_driver_;  // SPI driver instance
  std::string filename_;
  std::ofstream csvFile;
  int high_freq_counter_ = 0;
  // params/args
  int duration_ = 10;
  int data_size_ = 1024;
  float data_rate_hz_ = 1000.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<McuBridgeRosNode>();
  node.get()->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}