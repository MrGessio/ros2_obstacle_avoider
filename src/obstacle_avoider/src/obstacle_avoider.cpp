#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoider : public rclcpp::Node
{
public:
  ObstacleAvoider()
  : Node("obstacle_avoider")
  {
    // Publisher to send velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscriber to receive LaserScan data
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&ObstacleAvoider::scanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Obstacle Avoider Node Started");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float min_range = msg->range_max;

    // Find the minimum distance in the central sector (-30 to +30 degrees)
    int center_index = msg->ranges.size() / 2;
    int angle_range = 30;

    for (int i = center_index - angle_range; i <= center_index + angle_range; ++i)
    {
      if (i >= 0 && i < static_cast<int>(msg->ranges.size()) &&
          std::isfinite(msg->ranges[i]) &&
          msg->ranges[i] < min_range)
      {
        min_range = msg->ranges[i];
      }
    }

    RCLCPP_INFO(this->get_logger(), "Minimum distance: %.2f m", min_range);

    geometry_msgs::msg::Twist cmd;

    if (min_range < 0.5) {
      // Obstacle is close – turn left
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.5;
    } else {
      // Path is clear – move forward
      cmd.linear.x = 0.2;
      cmd.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoider>());
  rclcpp::shutdown();
  return 0;
}

