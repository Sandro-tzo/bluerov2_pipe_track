#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
// --- MODIFICA 1: Includere il tipo di messaggio corretto ---
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <random>
#include <functional>

class MlatNode : public rclcpp::Node
{
public:
  MlatNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  // --- MODIFICA 2: Cambiare il tipo del publisher ---
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  
  double noise_std_dev_;
  
  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
};

using std::placeholders::_1;

MlatNode::MlatNode()
: Node("mlat_node")
{
  this->declare_parameter<double>("noise_std_dev", 0.1);
  this->get_parameter("noise_std_dev", noise_std_dev_);

  normal_distribution_ = std::normal_distribution<double>(0.0, noise_std_dev_);

  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/bluerov2/odom", 10, std::bind(&MlatNode::odom_callback, this, _1));
    
  // --- MODIFICA 3: Creare il publisher con il nuovo tipo ---
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/bluerov2/mlat", 10);

  RCLCPP_INFO(this->get_logger(), "Nodo 'mlat_node' avviato. Pubblica PoseWithCovarianceStamped.");
}

void MlatNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // --- MODIFICA 4: Creare il messaggio del tipo corretto ---
  auto mlat_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

  // L'header rimane lo stesso
  mlat_pose_msg.header = msg->header;
  // Per sicurezza, assicuriamoci che il frame_id sia quello che il filtro si aspetta
  mlat_pose_msg.header.frame_id = "odom";

  // La posa ora è dentro un campo "pose"
  const auto& odom_pose = msg->pose.pose;
  mlat_pose_msg.pose.pose.position.x = odom_pose.position.x + normal_distribution_(random_generator_);
  mlat_pose_msg.pose.pose.position.y = odom_pose.position.y + normal_distribution_(random_generator_);
  mlat_pose_msg.pose.pose.position.z = odom_pose.position.z + normal_distribution_(random_generator_);
  mlat_pose_msg.pose.pose.orientation = odom_pose.orientation;

  // --- MODIFICA 5: Aggiungere la covarianza ---
  // La covarianza è una matrice 6x6 in forma di array [36].
  // Gli elementi sulla diagonale sono le varianze di [x, y, z, roll, pitch, yaw].
  double variance = noise_std_dev_ * noise_std_dev_;
  
  // Varianza per X (posizione 0)
  mlat_pose_msg.pose.covariance[0] = variance;
  // Varianza per Y (posizione 7)
  mlat_pose_msg.pose.covariance[7] = variance;
  // Varianza per Z (posizione 14)
  mlat_pose_msg.pose.covariance[14] = variance;

  // Non abbiamo informazioni sull'orientamento, quindi impostiamo la loro
  // varianza a un valore altissimo per dire al filtro di ignorarle.
  mlat_pose_msg.pose.covariance[21] = 9999.0; // Roll
  mlat_pose_msg.pose.covariance[28] = 9999.0; // Pitch
  mlat_pose_msg.pose.covariance[35] = 9999.0; // Yaw

  publisher_->publish(mlat_pose_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MlatNode>());
  rclcpp::shutdown();
  return 0;
}