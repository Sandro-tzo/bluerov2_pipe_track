#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <algorithm> // Per std::fill
#include <functional>

/*
 * Questo nodo si sottoscrive a un topic di odometria (/odom),
 * estrae le informazioni sulla velocità e le ripubblica come
 * messaggio TwistWithCovarianceStamped, simulando l'output di un DVL.
 * La matrice di covarianza viene impostata manualmente per riflettere
 * l'alta precisione sulle velocità lineari e la totale incertezza
 * su quelle angolari.
 */

class OdomToDvlNode : public rclcpp::Node
{
public:
  OdomToDvlNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
};

using std::placeholders::_1;

// --- Implementazione della Classe del Nodo ---
OdomToDvlNode::OdomToDvlNode()
: Node("odom_to_dvl_node")
{
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&OdomToDvlNode::odom_callback, this, _1));
    
  publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/bluerov2/dvl", 10);

  RCLCPP_INFO(this->get_logger(), "Nodo odom_to_dvl avviato.");
  RCLCPP_INFO(this->get_logger(), "Sottoscritto a /odom, pubblica su /bluerov2/dvl");
}

void OdomToDvlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto dvl_twist_msg = geometry_msgs::msg::TwistWithCovarianceStamped();

  // --- Header ---
  // ========================================================================= //
  // --- MODIFICA CRITICA PER LA SINCRONIZZAZIONE E LA COERENZA ---
  // 1. Preserviamo il timestamp del messaggio originale del simulatore.
  // 2. Forziamo il frame_id a "base_link" per essere sicuri che sia sempre corretto.
  // ========================================================================= //
  dvl_twist_msg.header.stamp = msg->header.stamp; 
  dvl_twist_msg.header.frame_id = "bluerov2/base_link"; 

  // --- Twist (Velocità) ---
  // Copiamo solo la parte lineare e azzeriamo quella angolare
  // per prevenire conflitti con l'IMU.
  dvl_twist_msg.twist.twist.linear = msg->twist.twist.linear;
  dvl_twist_msg.twist.twist.angular.x = 0.0;
  dvl_twist_msg.twist.twist.angular.y = 0.0;
  dvl_twist_msg.twist.twist.angular.z = 0.0;
  
  // --- Covariance (Incertezza) ---
  std::fill(dvl_twist_msg.twist.covariance.begin(), dvl_twist_msg.twist.covariance.end(), 0.0);

  // Varianza BASSISSIMA per le velocità LINEARI (vx, vy, vz)
  dvl_twist_msg.twist.covariance[0]  = 1e-6;
  dvl_twist_msg.twist.covariance[7]  = 1e-6;
  dvl_twist_msg.twist.covariance[14] = 1e-6;
  
  // Varianza ALTISSIMA per le velocità ANGOLARI (wx, wy, wz)
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