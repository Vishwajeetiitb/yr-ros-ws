
#include "rclcpp/rclcpp.hpp"

class PeriodicTasksNode : public rclcpp::Node {
public:
  PeriodicTasksNode() : Node("periodic_tasks_node") {
    // Create a timer for the 1000 Hz task
    timer_1000hz_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&PeriodicTasksNode::highFrequencyTask, this));

    // Create a timer for the 1 Hz task
    timer_1hz_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PeriodicTasksNode::lowFrequencyTask, this));
  }

private:
  void highFrequencyTask() {
    high_freq_counter_++;
    // Print every 100th call (10 Hz)
    if (high_freq_counter_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "High Frequency Task: 10 Hz print");
    }
  }

  void lowFrequencyTask() {
    // Print every call (1 Hz)
    RCLCPP_INFO(this->get_logger(), "Low Frequency Task: 1 Hz print");
  }

  rclcpp::TimerBase::SharedPtr timer_1000hz_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  int high_freq_counter_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PeriodicTasksNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}