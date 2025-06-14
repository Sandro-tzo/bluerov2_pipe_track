#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <random>
#include <functional>

class MlatNode : public rclcpp::Node
{
public:
  MlatNode() : Node("mlat_node")
  {
    this->declare_parameter<double>("noise_std_dev", 0.15);
    this->get_parameter("noise_std_dev", noise_std_dev_);

    // Inizializziamo il generatore di rumore
    normal_distribution_ = std::normal_distribution<double>(0.0, noise_std_dev_);

    // Subscriber con topic relativo "odom"
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MlatNode::odom_callback, this, std::placeholders::_1));

    // Publisher con topic relativo "pose"
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo 'mlat_node' avviato.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

    // --- Header ---
    // ========================================================================= //
    // Invece di usare l'ora corrente, preserviamo il timestamp del messaggio
    // originale del simulatore. Questo sincronizza tutti i dati.
    pose_msg.header.stamp = msg->header.stamp;
    // ========================================================================= //

    // Il dato di posizione assoluta è nel frame globale 'map'
    pose_msg.header.frame_id = "map"; 

    // --- Pose: Position ---
    // Prendiamo la posizione reale e aggiungiamo rumore
    pose_msg.pose.pose.position.x = msg->pose.pose.position.x + normal_distribution_(random_generator_);
    pose_msg.pose.pose.position.y = msg->pose.pose.position.y + normal_distribution_(random_generator_);
    pose_msg.pose.pose.position.z = msg->pose.pose.position.z + normal_distribution_(random_generator_);

    // --- Pose: Orientation ---
    // Impostiamo esplicitamente un quaternione identità (w=1), che significa "nessuna rotazione".
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;

    // --- Covariance ---
    // Impostiamo una matrice di covarianza realistica.
    std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);
    double position_variance = noise_std_dev_ * noise_std_dev_;
    
    pose_msg.pose.covariance[0] = position_variance;  // Varianza X
    pose_msg.pose.covariance[7] = position_variance;  // Varianza Y
    pose_msg.pose.covariance[14] = position_variance; // Varianza Z
    pose_msg.pose.covariance[21] = 9999.0;             // Varianza Roll (altissima)
    pose_msg.pose.covariance[28] = 9999.0;             // Varianza Pitch (altissima)
    pose_msg.pose.covariance[35] = 9999.0;             // Varianza Yaw (altissima)

    publisher_->publish(pose_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  
  double noise_std_dev_;
  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MlatNode>());
  rclcpp::shutdown();
  return 0;
}