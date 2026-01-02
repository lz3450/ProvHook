#include <rclcpp/logging.hpp>
#include <string.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "myturtlesim/srv/spawn.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "myturtlesim/msg/sensor.hpp"
#include "myturtlesim/msg/scale.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener(double scale_forward = 1.0, double scale_rotation = 1.0) : Node("turtle_tf2_frame_listener")
  {
    scale_forward_ = scale_forward;
    scale_rotation_ = scale_rotation;
    // Create a client to spawn a turtle
    spawner_ = this->create_client<myturtlesim::srv::Spawn>("spawn");
    while (!spawner_->service_is_ready())
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for service \"spawn\" ready...");
      sleep(1);
    }
    // Initialize request with turtle name and coordinates
    // Note that x, y and theta are defined as floats in myturtlesim/srv/Spawn
    const auto turtlename = "turtle2";
    auto request = std::make_shared<myturtlesim::srv::Spawn::Request>();
    request->name = turtlename;
    request->x = 4.0;
    request->y = 2.0;
    request->theta = 0.0;

    // Call request
    RCLCPP_INFO(this->get_logger(), "Sending request to spawn \"%s\".", turtlename);
    auto result = spawner_->async_send_request(
        request, [this, turtlename](rclcpp::Client<myturtlesim::srv::Spawn>::SharedFuture future) {
          auto result_turtlename = future.get()->name.c_str();
          if (strcmp(result_turtlename, turtlename) == 0)
          {
            RCLCPP_INFO(this->get_logger(), "\"%s\" successfully spawned.", result_turtlename);
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Service callback result \"%s\" mismatch!", result_turtlename);
          }
        });

    // Create turtle2 velocity publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    sensor_subscription_ = this->create_subscription<myturtlesim::msg::Sensor>(
        "sensor", 10, std::bind(&FrameListener::set_vel, this, std::placeholders::_1));

    scale_subscription_ = this->create_subscription<myturtlesim::msg::Scale>(
        "scale", 10, std::bind(&FrameListener::set_scale, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "scale_forward_=%.3f scale_rotation_=%.3f", scale_forward_, scale_rotation_);
  }

private:
  void set_vel(const myturtlesim::msg::Sensor msg)
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = scale_forward_ * msg.distance;
    cmd_vel_msg.angular.z = scale_rotation_ * msg.angle;
    RCLCPP_INFO(this->get_logger(), "cmd_vel_.linear.x=%.3f cmd_vel_.angular.z=%.3f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
    publisher_->publish(cmd_vel_msg);
  }

  void set_scale(const myturtlesim::msg::Scale& msg)
  {
    scale_forward_ = msg.scale_forward;
    RCLCPP_INFO(this->get_logger(), "scale_forward_=%.3f", scale_forward_);
    scale_rotation_ = msg.scale_rotation;
    RCLCPP_INFO(this->get_logger(), "scale_rotation_=%.3f", scale_rotation_);
  }

  rclcpp::Client<myturtlesim::srv::Spawn>::SharedPtr spawner_{ nullptr };
  rclcpp::Subscription<myturtlesim::msg::Sensor>::SharedPtr sensor_subscription_;
  rclcpp::Subscription<myturtlesim::msg::Scale>::SharedPtr scale_subscription_;
  double scale_forward_;
  double scale_rotation_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{ nullptr };
};

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " <scale_forward> <scale_rotation> ...\n";
    return 1;
  }

  char* endptr1;
  char* endptr2;
  double arg1 = std::strtod(argv[1], &endptr1);
  double arg2 = std::strtod(argv[2], &endptr2);

  // Check for any conversion errors
  if (*endptr1 != '\0' || *endptr2 != '\0')
  {
    std::cerr << "Invalid input: Non-numeric characters present.\n";
    return 2;
  }

  for (int i = 1; i < argc - 2; ++i)
  {
    argv[i] = argv[i + 2];
  }

  rclcpp::init(argc - 2, argv);
  rclcpp::spin(std::make_shared<FrameListener>(arg1, arg2));
  rclcpp::shutdown();
  return 0;
}
