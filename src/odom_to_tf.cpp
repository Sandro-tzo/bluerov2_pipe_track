// src/odom_to_tf.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

class OdomToTfNode : public rclcpp::Node
{
public:
  OdomToTfNode() : Node("odom_to_tf_node")
  {
    // Dichiara il topic di odometria come parametro
    this->declare_parameter<std::string>("odom_topic", "/bluerov2/odom");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    // Inizializza il broadcaster TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Sottoscrizione al topic di odometria
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>( // <<< CORREZIONE QUI
      odom_topic, 10, std::bind(&OdomToTfNode::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo Odom->TF avviato. In ascolto su '%s'", odom_topic.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Popola l'header della trasformata
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->header.frame_id; // Parent frame, es: "odom"
    t.child_frame_id = msg->child_frame_id;   // Child frame, es: "bluerov2/base_link"

    // Popola la trasformata con i dati di posa dal messaggio di odometria
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;

    // Invia la trasformata sull'albero TF
    tf_broadcaster_->sendTransform(t);
  }

  // Variabili membro
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTfNode>());
  rclcpp::shutdown();
  return 0;
}