#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ObstacleAvoider : public rclcpp::Node
{
public:
  ObstacleAvoider();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // ROS publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Obstacle avoidance state
  int stuck_counter_;
  float current_turn_;
  bool turning_90_;

  // Parameters for turning behavior
  const float target_turn_angle_ = 1.57;     // ~90 degrees in radians
  const int stuck_threshold_ = 10;           // steps before deciding it's stuck
};

