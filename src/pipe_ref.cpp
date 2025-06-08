// src/autonomous_cylinder_tracker.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

// Header per le trasformazioni dei quaternioni
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iomanip>
#include <optional>

// Alias per i tipi di messaggio
using ImageMsg = sensor_msgs::msg::Image;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using OdometryMsg = nav_msgs::msg::Odometry;

class AutonomousCylinderTracker : public rclcpp::Node
{
public:
  AutonomousCylinderTracker() : Node("autonomous_cylinder_tracker")
  {
    // --- Dichiarazione di tutti i parametri ---

    // Parametri di rilevamento
    this->declare_parameter<std::string>("image_topic", "/bluerov2/image");
    this->declare_parameter<int>("hsv_v_low", 0);
    this->declare_parameter<int>("hsv_v_high", 50);
    this->declare_parameter<int>("min_area", 1000);

    // Parametri di controllo
    this->declare_parameter<std::string>("velocity_setpoint_topic", "cmd_vel");
    this->declare_parameter<std::string>("pose_setpoint_topic", "cmd_pose");
    this->declare_parameter<std::string>("odom_topic", "/bluerov2/odom");
    this->declare_parameter<std::string>("robot_frame_id", "bluerov2/base_link");
    this->declare_parameter<std::string>("world_frame_id", "odom");
    this->declare_parameter<double>("target_area", 15000.0);
    this->declare_parameter<double>("yaw_gain", 0.0015);
    this->declare_parameter<double>("forward_gain", 0.00005);
    this->declare_parameter<double>("depth_setpoint", -8.0);
    this->declare_parameter<double>("depth_gain", 0.5);

    // --- Lettura dei parametri nelle variabili membro ---
    // (Omettiamo la lettura esplicita qui, useremo get_parameter() direttamente per chiarezza)

    // --- Sottoscrizioni e Pubblicazioni ---
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // QoS robusto per i comandi

    image_subscription_ = this->create_subscription<ImageMsg>(
        this->get_parameter("image_topic").as_string(), sensor_qos,
        std::bind(&AutonomousCylinderTracker::image_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<OdometryMsg>(
        this->get_parameter("odom_topic").as_string(), 10, // QoS standard per odometria
        std::bind(&AutonomousCylinderTracker::odom_callback, this, std::placeholders::_1));

    processed_image_pub_ = this->create_publisher<ImageMsg>("/cylinder_tracker/processed_image", 10);
    velocity_pub_ = this->create_publisher<TwistStampedMsg>(this->get_parameter("velocity_setpoint_topic").as_string(), command_qos);
    pose_pub_ = this->create_publisher<PoseStampedMsg>(this->get_parameter("pose_setpoint_topic").as_string(), command_qos);

    RCLCPP_INFO(this->get_logger(), "Tracker Autonomo (Pose+Velocity) avviato.");
  }

private:
  void odom_callback(const OdometryMsg::SharedPtr msg)
  {
    current_odometry_ = *msg;
  }

  void image_callback(const ImageMsg::SharedPtr msg)
  {
    if (!current_odometry_.has_value())
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "In attesa di odometria sul topic '%s'...", this->get_parameter("odom_topic").as_string().c_str());
      return;
    }

    // --- 1. Genera e Pubblica il Setpoint di POSA (per stabilità) ---
    this->publish_pose_setpoint();

    // --- 2. Genera e Pubblica il Setpoint di VELOCITÀ (per movimento) ---
    this->publish_velocity_setpoint(msg);
  }

  void publish_pose_setpoint()
  {
    auto pose_msg = std::make_unique<PoseStampedMsg>();
    pose_msg->header.stamp = this->get_clock()->now();
    pose_msg->header.frame_id = this->get_parameter("world_frame_id").as_string();

    // Estrai l'imbardata (yaw) corrente dall'odometria per non comandare rotazioni indesiderate
    tf2::Quaternion current_q;
    tf2::fromMsg(current_odometry_->pose.pose.orientation, current_q);
    double roll, pitch, current_yaw;
    tf2::Matrix3x3(current_q).getRPY(roll, pitch, current_yaw);

    // Crea un nuovo quaternione con roll=0, pitch=0, e l'imbardata corrente
    tf2::Quaternion setpoint_q;
    setpoint_q.setRPY(0.0, 0.0, current_yaw);

    // Popola il messaggio di posa:
    // Obiettivo X/Y = Posizione attuale (non muoverti in base alla posa)
    // Obiettivo Z = Setpoint di profondità desiderato
    // Obiettivo Orientamento = Livellato e con l'imbardata attuale
    pose_msg->pose.position.x = current_odometry_->pose.pose.position.x;
    pose_msg->pose.position.y = current_odometry_->pose.pose.position.y;
    pose_msg->pose.position.z = this->get_parameter("depth_setpoint").as_double();
    pose_msg->pose.orientation = tf2::toMsg(setpoint_q);

    pose_pub_->publish(std::move(pose_msg));
  }

  void publish_velocity_setpoint(const ImageMsg::SharedPtr msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat &frame = cv_ptr->image;
      int image_center_x = frame.cols / 2;

      // Crea il messaggio di velocità
      auto twist_msg = std::make_unique<TwistStampedMsg>();
      twist_msg->header.stamp = msg->header.stamp; // Sincronizzato con l'immagine per minor latenza
      twist_msg->header.frame_id = this->get_parameter("robot_frame_id").as_string();

      // Controlla sempre la profondità
      double current_depth = current_odometry_->pose.pose.position.z;
      double depth_error = this->get_parameter("depth_setpoint").as_double() - current_depth;
      twist_msg->twist.linear.z = this->get_parameter("depth_gain").as_double() * depth_error;

      // Logica di Computer Vision
      cv::Mat hsv_frame, mask;
      cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
      cv::Scalar lower_black(0, 0, this->get_parameter("hsv_v_low").as_int());
      cv::Scalar upper_black(180, 255, this->get_parameter("hsv_v_high").as_int());
      cv::inRange(hsv_frame, lower_black, upper_black, mask);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      bool target_found = false;
      if (!contours.empty())
      {
        double max_area = 0;
        int largest_contour_idx = -1;
        for (size_t i = 0; i < contours.size(); i++) {
          double area = cv::contourArea(contours[i]);
          if (area > max_area) { max_area = area; largest_contour_idx = i; }
        }

        if (max_area > this->get_parameter("min_area").as_int())
        {
          target_found = true;
          cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_idx]);
          cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);

          // Calcola velocità di avanzamento e imbardata
          int target_center_x = bounding_box.x + bounding_box.width / 2;
          double yaw_error = image_center_x - target_center_x;
          double forward_error = this->get_parameter("target_area").as_double() - max_area;

          twist_msg->twist.linear.x = this->get_parameter("forward_gain").as_double() * forward_error;
          twist_msg->twist.angular.z = this->get_parameter("yaw_gain").as_double() * yaw_error;
        }
      }

      if (!target_found)
      {
        twist_msg->twist.linear.x = 0.0;
        twist_msg->twist.angular.z = 0.0;
      }

      velocity_pub_->publish(std::move(twist_msg));
      processed_image_pub_->publish(*cv_ptr->toImageMsg());
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge: %s", e.what());
    }
  }

  // Variabili Membro
  rclcpp::Subscription<ImageMsg>::SharedPtr image_subscription_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_subscription_;
  rclcpp::Publisher<ImageMsg>::SharedPtr processed_image_pub_;
  rclcpp::Publisher<TwistStampedMsg>::SharedPtr velocity_pub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr pose_pub_;
  std::optional<OdometryMsg> current_odometry_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousCylinderTracker>());
  rclcpp::shutdown();
  return 0;
}