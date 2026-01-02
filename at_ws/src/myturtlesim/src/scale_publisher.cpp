#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "myturtlesim/msg/scale.hpp"

using namespace std::chrono_literals;

class ScalePublisher : public rclcpp::Node
{
public:
  ScalePublisher() : Node("scale_publisher")
  {
    this->declare_parameter<float>("scale_forward", 0.5);
    this->declare_parameter<float>("scale_rotation", 1.0);
    publisher_ = this->create_publisher<myturtlesim::msg::Scale>("scale", 10);
    timer_ = this->create_wall_timer(3000ms, std::bind(&ScalePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    myturtlesim::msg::Scale msg;
    msg.scale_forward = this->get_parameter("scale_forward").get_parameter_value().get<float>();
    msg.scale_rotation = this->get_parameter("scale_rotation").get_parameter_value().get<float>();
    RCLCPP_INFO(this->get_logger(), "scale_forward=%.3f scale_rotation=%.3f", msg.scale_forward, msg.scale_rotation);
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<myturtlesim::msg::Scale>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScalePublisher>());
  rclcpp::shutdown();
  return 0;
}
