#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;


class RvizOdometry : public rclcpp::Node
{
public:
  RvizOdometry()
  : Node("rviz_odometry")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&RvizOdometry::topic_callback, this, _1));

    publication_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/rviz_odom", 10);
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry position;

    position.header.frame_id = "odom";
    position.child_frame_id = "base_footprint";

    /*==================================================*/

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);  // [rad]

    position.pose.pose.position.x = (msg->pose.pose.position.x * 100 + 33) - 9.5 * cos(yaw);  // [cm]
    position.pose.pose.position.y = (msg->pose.pose.position.y * 100 + 50) - 9.5 * sin(yaw);  // [cm]

    /*==================================================*/

    position.pose.pose.position.z = 0.0;
    position.pose.pose.orientation = msg->pose.pose.orientation;

    publication_->publish(position);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publication_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizOdometry>());
  rclcpp::shutdown();
  return 0;
}