#ifndef ODOM_TO_DVL_HPP_
#define ODOM_TO_DVL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <random>

class OdomToDvlNode : public rclcpp::Node
{
public:
  OdomToDvlNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
  
  double noise_std_dev_;
  
  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
};

#endif // ODOM_TO_DVL_HPP_