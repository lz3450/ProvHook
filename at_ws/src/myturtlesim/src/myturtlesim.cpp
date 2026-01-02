#include <QApplication>

#include "rclcpp/rclcpp.hpp"
#include "myturtlesim/turtle_frame.h"

class TurtleApp : public QApplication
{
public:
  rclcpp::Node::SharedPtr nh_;

  explicit TurtleApp(int& argc, char** argv) : QApplication(argc, argv)
  {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("myturtlesim");
  }

  ~TurtleApp()
  {
    rclcpp::shutdown();
  }

  int exec()
  {
    myturtlesim::TurtleFrame frame(nh_);
    frame.show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  TurtleApp app(argc, argv);
  return app.exec();
}
