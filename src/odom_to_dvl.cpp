#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <algorithm> 
#include <functional>
#include <random>  

/*
 * Questo nodo si sottoscrive a un topic di odometria (/odom),
 * estrae le informazioni sulla velocità, aggiunge un rumore bianco
 * gaussiano e le ripubblica come messaggio TwistWithCovarianceStamped,
 * simulando l'output di un DVL.
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

  // --- Membri per la generazione di rumore gaussiano ---
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
};

using std::placeholders::_1;

// --- Implementazione della Classe del Nodo ---
OdomToDvlNode::OdomToDvlNode()
: Node("dvl_node")
{
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&OdomToDvlNode::odom_callback, this, _1));
    
  publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist", 10);

  // --- Inizializzazione del generatore di rumore ---
  // Varianza desiderata per le velocità lineari
  const double linear_velocity_variance = 1e-4;
  // La deviazione standard è la radice quadrata della varianza
  const double std_dev = std::sqrt(linear_velocity_variance);
  
  // Inizializza il generatore di numeri casuali con un seed non deterministico
  std::random_device rd;
  generator_ = std::default_random_engine(rd());
  // Imposta la distribuzione normale con media 0.0 e la deviazione standard calcolata
  distribution_ = std::normal_distribution<double>(0.0, std_dev);

  RCLCPP_INFO(this->get_logger(), "Nodo 'dvl_node' avviato. Aggiunge rumore gaussiano con std_dev = %f", std_dev);
}

void OdomToDvlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto dvl_twist_msg = geometry_msgs::msg::TwistWithCovarianceStamped();

  // --- Header ---
  // Preserviamo il timestamp del messaggio originale e forziamo il frame_id corretto.
  dvl_twist_msg.header.stamp = msg->header.stamp; 
  dvl_twist_msg.header.frame_id = "bluerov2/base_link"; 

  // --- Twist (Velocità) ---
  // Copiamo la velocità lineare dal messaggio di odometria.
  dvl_twist_msg.twist.twist.linear = msg->twist.twist.linear;
  
  // ========================================================================= //
  // --- AGGIUNTA DI RUMORE BIANCO GAUSSIANO ---
  // Simuliamo l'imprecisione del sensore aggiungendo un piccolo valore
  // casuale, campionato da una distribuzione normale, a ogni componente.
  // ========================================================================= //
  dvl_twist_msg.twist.twist.linear.x += distribution_(generator_);
  dvl_twist_msg.twist.twist.linear.y += distribution_(generator_);
  dvl_twist_msg.twist.twist.linear.z += distribution_(generator_);

  // Azzeriamo la velocità angolare per non creare conflitti con l'IMU.
  dvl_twist_msg.twist.twist.angular.x = 0.0;
  dvl_twist_msg.twist.twist.angular.y = 0.0;
  dvl_twist_msg.twist.twist.angular.z = 0.0;
  
  // --- Covariance (Incertezza) ---
  // Impostiamo la matrice di covarianza per comunicare al filtro 
  // l'incertezza associata a queste misure.
  std::fill(dvl_twist_msg.twist.covariance.begin(), dvl_twist_msg.twist.covariance.end(), 0.0);

  // Varianza per le velocità LINEARI (vx, vy, vz). 
  dvl_twist_msg.twist.covariance[0]  = 1e-4; // var(vx)
  dvl_twist_msg.twist.covariance[7]  = 1e-4; // var(vy)
  dvl_twist_msg.twist.covariance[14] = 1e-4; // var(vz)
  
  // Varianza ALTISSIMA per le velocità ANGOLARI (wx, wy, wz) per indicare
  // al filtro di ignorare completamente questi valori (che sono a zero).
  dvl_twist_msg.twist.covariance[21] = 9999.0; // var(wx)
  dvl_twist_msg.twist.covariance[28] = 9999.0; // var(wy)
  dvl_twist_msg.twist.covariance[35] = 9999.0; // var(wz)

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