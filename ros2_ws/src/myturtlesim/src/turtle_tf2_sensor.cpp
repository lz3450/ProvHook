#include <functional>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "myturtlesim/msg/sensor.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener() : Node("turtle_tf2_sensor"), gen_(rd_()), dis_(-1.0, 1.0)
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<myturtlesim::msg::Sensor>("sensor", 10);
    // Call on_timer function every second
    std::chrono::milliseconds sensor_frequency = 500ms;
    timer_ = this->create_wall_timer(sensor_frequency, std::bind(&FrameListener::on_timer, this));
    RCLCPP_INFO(this->get_logger(), "angle_sensor_frequency=%s",
                std::to_string(sensor_frequency.count()).c_str());
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";

    geometry_msgs::msg::TransformStamped t;

    myturtlesim::msg::Sensor sensor_msg;

    // Look up for the transformation between target_frame and turtle2 frames
    try
    {
      t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(),
                  ex.what());
    }

    sensor_msg.distance = sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));
    // sensor_msg.distance = sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2)) + dis_(gen_);
    sensor_msg.angle = atan2(t.transform.translation.y, t.transform.translation.x);
    // sensor_msg.angle = atan2(t.transform.translation.y, t.transform.translation.x) + dis_(gen_);
    RCLCPP_INFO(this->get_logger(), "sensor_msg.distance=%.3f sensor_msg.angle=%.3f", sensor_msg.distance, sensor_msg.angle);

    publisher_->publish(sensor_msg);
  }

  // Create a random device and use it to seed the random number generator
  std::random_device rd_;
  std::mt19937 gen_;
  // Define the distribution to be between 0 and 1. Note that std::uniform_real_distribution
  // is used for floating-point numbers.
  std::uniform_real_distribution<> dis_;

  std::string target_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  rclcpp::TimerBase::SharedPtr timer_{ nullptr };
  rclcpp::Publisher<myturtlesim::msg::Sensor>::SharedPtr publisher_{ nullptr };
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
