#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_srvs/srv/empty.hpp>
#include <myturtlesim/srv/spawn.hpp>
#include <myturtlesim/srv/kill.hpp>
#include <map>

#include "turtle.h"
#endif

namespace myturtlesim
{

class TurtleFrame : public QFrame
{
  Q_OBJECT
public:
  TurtleFrame(rclcpp::Node::SharedPtr& node_handle, QWidget* parent = 0, Qt::WindowFlags f = Qt::WindowFlags());
  ~TurtleFrame();

  std::string spawnTurtle(const std::string& name, float x, float y, float angle);
  std::string spawnTurtle(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

private:
  void updateTurtles();
  void clear();
  bool hasTurtle(const std::string& name);

  bool clearCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  bool resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  bool spawnCallback(const myturtlesim::srv::Spawn::Request::SharedPtr, myturtlesim::srv::Spawn::Response::SharedPtr);
  bool killCallback(const myturtlesim::srv::Kill::Request::SharedPtr, myturtlesim::srv::Kill::Response::SharedPtr);

  void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

  rclcpp::Node::SharedPtr nh_;

  QTimer* update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  rclcpp::Time last_turtle_update_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<myturtlesim::srv::Spawn>::SharedPtr spawn_srv_;
  rclcpp::Service<myturtlesim::srv::Kill>::SharedPtr kill_srv_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}  // namespace myturtlesim
