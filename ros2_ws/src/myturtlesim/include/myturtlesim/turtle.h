#ifndef __TURTLE_H__
#define __TURTLE_H__

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <myturtlesim/action/rotate_absolute.hpp>
#include <myturtlesim/msg/color.hpp>
#include <myturtlesim/msg/pose.hpp>
#include <myturtlesim/srv/set_pen.hpp>
#include <myturtlesim/srv/teleport_absolute.hpp>
#include <myturtlesim/srv/teleport_relative.hpp>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

#define PI 3.14159265359
#define TAU 2.0 * PI

namespace myturtlesim
{

class Turtle
{
public:
  using RotateAbsoluteGoalHandle = rclcpp_action::ServerGoalHandle<myturtlesim::action::RotateAbsolute>;

  Turtle(rclcpp::Node::SharedPtr& nh, const std::string& real_name, const QImage& turtle_image, const QPointF& pos,
         float orient);

  bool update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height);
  void paint(QPainter& painter);

private:
  void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel);
  bool setPenCallback(const myturtlesim::srv::SetPen::Request::SharedPtr,
                      myturtlesim::srv::SetPen::Response::SharedPtr);
  bool teleportRelativeCallback(const myturtlesim::srv::TeleportRelative::Request::SharedPtr,
                                myturtlesim::srv::TeleportRelative::Response::SharedPtr);
  bool teleportAbsoluteCallback(const myturtlesim::srv::TeleportAbsolute::Request::SharedPtr,
                                myturtlesim::srv::TeleportAbsolute::Response::SharedPtr);
  void rotateAbsoluteAcceptCallback(const std::shared_ptr<RotateAbsoluteGoalHandle>);

  void rotateImage();

  rclcpp::Node::SharedPtr nh_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  QPointF pos_;
  qreal orient_;

  qreal lin_vel_x_;
  qreal lin_vel_y_;
  qreal ang_vel_;
  bool pen_on_;
  QPen pen_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<myturtlesim::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<myturtlesim::msg::Color>::SharedPtr color_pub_;
  rclcpp::Service<myturtlesim::srv::SetPen>::SharedPtr set_pen_srv_;
  rclcpp::Service<myturtlesim::srv::TeleportRelative>::SharedPtr teleport_relative_srv_;
  rclcpp::Service<myturtlesim::srv::TeleportAbsolute>::SharedPtr teleport_absolute_srv_;
  rclcpp_action::Server<myturtlesim::action::RotateAbsolute>::SharedPtr rotate_absolute_action_server_;

  std::shared_ptr<RotateAbsoluteGoalHandle> rotate_absolute_goal_handle_;
  std::shared_ptr<myturtlesim::action::RotateAbsolute::Feedback> rotate_absolute_feedback_;
  std::shared_ptr<myturtlesim::action::RotateAbsolute::Result> rotate_absolute_result_;
  qreal rotate_absolute_start_orient_;

  rclcpp::Time last_command_time_;

  float meter_;

  struct TeleportRequest
  {
    TeleportRequest(float x, float y, qreal _theta, qreal _linear, bool _relative)
      : pos(x, y), theta(_theta), linear(_linear), relative(_relative)
    {
    }

    QPointF pos;
    qreal theta;
    qreal linear;
    bool relative;
  };
  typedef std::vector<TeleportRequest> V_TeleportRequest;
  V_TeleportRequest teleport_requests_;
};
typedef std::shared_ptr<Turtle> TurtlePtr;

}  // namespace myturtlesim

#endif
