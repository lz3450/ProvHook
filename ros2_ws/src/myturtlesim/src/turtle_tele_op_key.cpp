#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <functional>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "myturtlesim/action/rotate_absolute.hpp"

static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT = 0x44;
static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;
static constexpr char KEYCODE_B = 0x62;
static constexpr char KEYCODE_C = 0x63;
static constexpr char KEYCODE_D = 0x64;
static constexpr char KEYCODE_E = 0x65;
static constexpr char KEYCODE_F = 0x66;
static constexpr char KEYCODE_G = 0x67;
static constexpr char KEYCODE_Q = 0x71;
static constexpr char KEYCODE_R = 0x72;
static constexpr char KEYCODE_T = 0x74;
static constexpr char KEYCODE_V = 0x76;

bool running = true;

class KeyboardReader final
{
public:
  KeyboardReader()
  {
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0)
    {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0)
    {
      throw std::runtime_error("Failed to set new console mode");
    }
  }

  char readOne()
  {
    char c = 0;

    int rc = read(0, &c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }

    return c;
  }

  ~KeyboardReader()
  {
    tcsetattr(0, TCSANOW, &cooked_);
  }

private:
  struct termios cooked_;
};

class TeleOpTurtle final
{
public:
  TeleOpTurtle()
  {
    nh_ = rclcpp::Node::make_shared("turtle_tele_op");
    nh_->declare_parameter("scale_angular", 2.0);
    nh_->declare_parameter("scale_linear", 2.0);

    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
    rotate_absolute_client_ =
        rclcpp_action::create_client<myturtlesim::action::RotateAbsolute>(nh_, "turtle1/rotate_absolute");
  }

  int keyLoop()
  {
    char c;

    std::thread{ std::bind(&TeleOpTurtle::spin, this) }.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle.");
    puts("Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.");
    puts("'Q' to quit.");

    while (running)
    {
      // get the next event from the keyboard
      try
      {
        c = input_.readOne();
      }
      catch (const std::runtime_error&)
      {
        perror("read():");
        return -1;
      }

      double linear = 0.0;
      double angular = 0.0;

      RCLCPP_DEBUG(nh_->get_logger(), "value=0x%02X\n", c);

      switch (c)
      {
        case KEYCODE_LEFT:
          RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
          angular = 1.0;
          break;
        case KEYCODE_RIGHT:
          RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
          angular = -1.0;
          break;
        case KEYCODE_UP:
          RCLCPP_DEBUG(nh_->get_logger(), "UP");
          linear = 1.0;
          break;
        case KEYCODE_DOWN:
          RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
          linear = -1.0;
          break;
        case KEYCODE_G:
          RCLCPP_DEBUG(nh_->get_logger(), "G");
          sendGoal(0.0f);
          break;
        case KEYCODE_T:
          RCLCPP_DEBUG(nh_->get_logger(), "T");
          sendGoal(0.7854f);
          break;
        case KEYCODE_R:
          RCLCPP_DEBUG(nh_->get_logger(), "R");
          sendGoal(1.5708f);
          break;
        case KEYCODE_E:
          RCLCPP_DEBUG(nh_->get_logger(), "E");
          sendGoal(2.3562f);
          break;
        case KEYCODE_D:
          RCLCPP_DEBUG(nh_->get_logger(), "D");
          sendGoal(3.1416f);
          break;
        case KEYCODE_C:
          RCLCPP_DEBUG(nh_->get_logger(), "C");
          sendGoal(-2.3562f);
          break;
        case KEYCODE_V:
          RCLCPP_DEBUG(nh_->get_logger(), "V");
          sendGoal(-1.5708f);
          break;
        case KEYCODE_B:
          RCLCPP_DEBUG(nh_->get_logger(), "B");
          sendGoal(-0.7854f);
          break;
        case KEYCODE_F:
          RCLCPP_DEBUG(nh_->get_logger(), "F");
          cancelGoal();
          break;
        case KEYCODE_Q:
          RCLCPP_DEBUG(nh_->get_logger(), "quit");
          running = false;
          break;
        default:
          // This can happen if the read returned when there was no data, or
          // another key was pressed.  In these cases, just silently ignore the
          // press.
          break;
      }

      if (running && (linear != 0.0 || angular != 0.0))
      {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = nh_->get_parameter("scale_angular").as_double() * angular;
        twist.linear.x = nh_->get_parameter("scale_linear").as_double() * linear;
        twist_pub_->publish(twist);
      }
    }

    return 0;
  }

private:
  void spin()
  {
    rclcpp::spin(nh_);
  }

  void sendGoal(float theta)
  {
    auto goal = myturtlesim::action::RotateAbsolute::Goal();
    goal.theta = theta;
    auto send_goal_options = rclcpp_action::Client<myturtlesim::action::RotateAbsolute>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<myturtlesim::action::RotateAbsolute>::SharedPtr goal_handle) {
          RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
          this->goal_handle_ = goal_handle;
        };
    rotate_absolute_client_->async_send_goal(goal, send_goal_options);
  }

  void cancelGoal()
  {
    if (goal_handle_)
    {
      RCLCPP_DEBUG(nh_->get_logger(), "Sending cancel request");
      try
      {
        rotate_absolute_client_->async_cancel_goal(goal_handle_);
      }
      catch (...)
      {
        // This can happen if the goal has already terminated and expired
      }
    }
  }

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp_action::Client<myturtlesim::action::RotateAbsolute>::SharedPtr rotate_absolute_client_;
  rclcpp_action::ClientGoalHandle<myturtlesim::action::RotateAbsolute>::SharedPtr goal_handle_;

  KeyboardReader input_;
};

void quit(int sig)
{
  (void)sig;
  running = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, quit);

  TeleOpTurtle tele_op_turtle;

  int rc = tele_op_turtle.keyLoop();

  rclcpp::shutdown();

  return rc;
}
