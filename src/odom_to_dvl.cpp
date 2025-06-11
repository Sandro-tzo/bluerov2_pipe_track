#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <random>
#include <functional>

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

using std::placeholders::_1;

// --- Implementazione della Classe del Nodo ---
OdomToDvlNode::OdomToDvlNode()
: Node("odom_to_dvl_node")
{
  this->declare_parameter<double>("noise_std_dev", 0.05);
  this->get_parameter("noise_std_dev", noise_std_dev_);

  normal_distribution_ = std::normal_distribution<double>(0.0, noise_std_dev_);

  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/bluerov2/odom", 10, std::bind(&OdomToDvlNode::odom_callback, this, _1));
    
  publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/bluerov2/dvl/twist", 10);

  RCLCPP_INFO(this->get_logger(), "Nodo odom_to_dvl avviato. Rumore (std dev): %f", noise_std_dev_);
}

void OdomToDvlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto dvl_twist_msg = geometry_msgs::msg::TwistWithCovarianceStamped();

  dvl_twist_msg.header.stamp = msg->header.stamp;
  dvl_twist_msg.header.frame_id = "dvl_link";

  const auto& odom_twist = msg->twist.twist.linear;
  dvl_twist_msg.twist.twist.linear.x = odom_twist.x + normal_distribution_(random_generator_);
  dvl_twist_msg.twist.twist.linear.y = odom_twist.y + normal_distribution_(random_generator_);
  dvl_twist_msg.twist.twist.linear.z = odom_twist.z + normal_distribution_(random_generator_);

  double variance = noise_std_dev_ * noise_std_dev_;
  dvl_twist_msg.twist.covariance[0] = variance;
  dvl_twist_msg.twist.covariance[7] = variance;
  dvl_twist_msg.twist.covariance[14] = variance;
  dvl_twist_msg.twist.covariance[21] = 9999.0;
  dvl_twist_msg.twist.covariance[28] = 9999.0;
  dvl_twist_msg.twist.covariance[35] = 9999.0;

  publisher_->publish(dvl_twist_msg);
}

// --- Funzione Main per l'eseguibile ---
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToDvlNode>());
  rclcpp::shutdown();
  return 0;
}