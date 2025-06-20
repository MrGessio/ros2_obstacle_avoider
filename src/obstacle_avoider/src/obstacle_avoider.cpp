#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <vector>

class ObstacleAvoider : public rclcpp::Node
{
public:
  ObstacleAvoider() : Node("obstacle_avoider")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ObstacleAvoider::scanCallback, this, std::placeholders::_1));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto ranges = msg->ranges;
    int range_size = ranges.size();

    // Divide scan into 3 sectors
    int left_start = range_size * 0.0;
    int left_end   = range_size * 0.15;

    int front_start = range_size * 0.45;
    int front_end   = range_size * 0.55;

    int right_start = range_size * 0.85;
    int right_end   = range_size * 1.0;

    float left_min = *std::min_element(ranges.begin() + left_start, ranges.begin() + left_end);
    float front_min = *std::min_element(ranges.begin() + front_start, ranges.begin() + front_end);
    float right_min = *std::min_element(ranges.begin() + right_start, ranges.begin() + right_end);

    geometry_msgs::msg::Twist cmd;

    float threshold = 0.4;  // meters

    // Decision logic based on obstacle position
    if (front_min < threshold) {
      // Obstacle in front → rotate 180 degrees (turn in place longer)
      cmd.angular.z = 0.8;
      cmd.linear.x = 0.0;
    } else if (left_min < threshold) {
      // Obstacle on the left → turn right
      cmd.angular.z = -0.5;
      cmd.linear.x = 0.05;
    } else if (right_min < threshold) {
      // Obstacle on the right → turn left
      cmd.angular.z = 0.5;
      cmd.linear.x = 0.05;
    } else {
      // No obstacle nearby → move forward
      cmd.linear.x = 0.2;
      cmd.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};
  
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoider>());
  rclcpp::shutdown();
  return 0;
}

