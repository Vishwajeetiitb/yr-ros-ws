#include "rclcpp/rclcpp.hpp"
#include "spi_driver.h"
// #include "pb2ros_converter.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "pb2ros_converter.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "protobuf/exo_data.pb.h"

constexpr char kSpiDevice[] = "/dev/spidev4.1";
constexpr uint32_t kSpiFrequency = 10000000;

class SpiDriverRosNode : public rclcpp::Node {
 public:
  SpiDriverRosNode()
    : Node("spi_driver_ros_node")
    , spi_driver_(kSpiDevice, kSpiFrequency) {
    this->declare_parameter<int>("duration", 10);
    this->declare_parameter<int>("data_size", 1024);
    this->declare_parameter<double>("data_rate_hz", 1000.0);

    int duration_ = this->get_parameter("duration").as_int();
    int data_size_ = this->get_parameter("data_size").as_int();
    double data_rate_hz_ = this->get_parameter("data_rate_hz").as_double();
    
    // Create a ROS Vector3 message and initialize it
    geometry_msgs::msg::Vector3 rosVector3Msg;
    rosVector3Msg.x = 1.0;
    rosVector3Msg.y = 2.0;
    rosVector3Msg.z = 3.0;

    Vector3 pbVector3Msg;
    Pb2RosConverter::ConvertRos2Pb(rosVector3Msg, pbVector3Msg);
    std::cout << "Protobuf Vector3 message:\n"
              << "x: " << pbVector3Msg.x() << "\n"
              << "y: " << pbVector3Msg.y() << "\n"
              << "z: " << pbVector3Msg.z() << "\n";

    // Print out the parameters in a banner
    std::cout << std::setfill('=') << std::setw(50) << "=" << std::endl;
    std::cout << "SPI Test Configuration:" << std::endl;
    std::cout << "Duration: " << duration_ << " seconds" << std::endl;
    std::cout << "Data Size: " << data_size_ << " bytes" << std::endl;
    std::cout << "Data Rate: " << data_rate_hz_ << " Hz" << std::endl;
    std::cout << std::setfill('=') << std::setw(50) << "=" << std::endl;

    spi_driver_.EnableSimulation(true);
    // spi_driver.EnableSimulation(false);
    spi_driver_.Init();

    // Create a high-frequency timer for SPI communication
    auto send_interval_us = std::chrono::microseconds(static_cast<int>(1000000 / data_rate_hz_));
    // auto send_interval_us = std::chrono::microseconds(1000000 / data_rate_hz_);
    std::cout << "SendReceiveData thread running interval: " << send_interval_us.count() << " us" << std::endl;
    timer_1000hz_ = this->create_wall_timer(  // std::chrono::milliseconds(1),
        send_interval_us, std::bind(&SpiDriverRosNode::spiSendReceiveTask, this));

    // Create a low-frequency timer for printing statistics
    timer_1hz_ =
        this->create_wall_timer(std::chrono::seconds(1), std::bind(&SpiDriverRosNode::printStatisticsTask, this));
  }

 private:
  void sendSpiData() {
    std::vector<uint8_t> testData(data_size_, 0xAA);
    spi_driver_.Send(testData);
    printf("SendReceiveData thread stopped\n");
  }

  void spiSendReceiveTask() {
    high_freq_counter_++;
    // Print every 100th call (10 Hz)
    if (high_freq_counter_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "High Frequency Task: 10 Hz print");
    }
    // sendSpiData();
  }

  // Task for printing SPI communication statistics
  void printStatisticsTask() {
    RCLCPP_INFO(this->get_logger(), "Low Frequency Task: 1 Hz print");
    //   spi_driver.PrintMsgStats();
  }

  rclcpp::TimerBase::SharedPtr timer_1000hz_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  SPIDriver spi_driver_;  // SPI driver instance
  int high_freq_counter_ = 0;

  int duration_;
  int data_size_;
  float data_rate_hz_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpiDriverRosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}