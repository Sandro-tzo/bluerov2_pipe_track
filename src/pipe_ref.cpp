#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iomanip>
#include <optional>
#include <cmath>

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
    // I parametri sono quelli che hai calibrato tu
    this->declare_parameter<std::string>("image_topic", "/bluerov2/image");
    this->declare_parameter<int>("hsv_v_low", 0);
    this->declare_parameter<int>("hsv_v_high", 50);
    this->declare_parameter<int>("min_area", 1000);
    this->declare_parameter<std::string>("velocity_setpoint_topic", "tracker/cmd_vel");
    this->declare_parameter<std::string>("pose_setpoint_topic", "tracker/cmd_pose");
    this->declare_parameter<std::string>("odom_topic", "/bluerov2/odom");
    this->declare_parameter<std::string>("robot_frame_id", "bluerov2/base_link");
    this->declare_parameter<std::string>("world_frame_id", "world");
    this->declare_parameter<double>("depth_setpoint", -8.0);
    this->declare_parameter<double>("depth_tolerance", 0.5);
    this->declare_parameter<double>("constant_forward_speed", 1.0);
    this->declare_parameter<double>("yaw_correction_gain", 0.0001);
    this->declare_parameter<double>("search_yaw_velocity", 0.5);
    
    // Sottoscrizioni e Pubblicazioni
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    image_subscription_ = this->create_subscription<ImageMsg>(
        this->get_parameter("image_topic").as_string(), sensor_qos,
        std::bind(&AutonomousCylinderTracker::image_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<OdometryMsg>(
        this->get_parameter("odom_topic").as_string(), 10,
        std::bind(&AutonomousCylinderTracker::odom_callback, this, std::placeholders::_1));

    processed_image_pub_ = this->create_publisher<ImageMsg>("/cylinder_tracker/processed_image", 10);
    velocity_pub_ = this->create_publisher<TwistStampedMsg>(this->get_parameter("velocity_setpoint_topic").as_string(), command_qos);
    pose_pub_ = this->create_publisher<PoseStampedMsg>(this->get_parameter("pose_setpoint_topic").as_string(), command_qos);

    RCLCPP_INFO(this->get_logger(), "Tracker Autonomo (Guida su Punto Vicino) avviato.");
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr image_subscription_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_subscription_;
  rclcpp::Publisher<ImageMsg>::SharedPtr processed_image_pub_;
  rclcpp::Publisher<TwistStampedMsg>::SharedPtr velocity_pub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr pose_pub_;
  std::optional<OdometryMsg> current_odometry_;
  std::optional<double> initial_yaw_;

  void odom_callback(const OdometryMsg::SharedPtr msg)
  {
    current_odometry_ = *msg;
  }

  void image_callback(const ImageMsg::SharedPtr msg)
  {
    if (!current_odometry_.has_value()) { return; }

    bool target_found = false;
    int pixel_error = 0;
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat &frame = cv_ptr->image;
      int width = frame.cols;
      int height = frame.rows;

      cv::Mat hsv_frame, mask;
      cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
      cv::Scalar lower_black(this->get_parameter("hsv_v_low").as_int(), 0, 0);
      cv::Scalar upper_black(180, 255, this->get_parameter("hsv_v_high").as_int());
      cv::inRange(hsv_frame, lower_black, upper_black, mask);

      cv::Rect roi_near_rect(0, height * 0.8, width, height * 0.2);
      cv::Mat roi_near = mask(roi_near_rect);
      
      cv::Rect roi_far_rect(0, height * 0.5, width, height * 0.2);
      cv::Mat roi_far = mask(roi_far_rect);

      cv::Moments M_near = cv::moments(roi_near);
      cv::Moments M_far = cv::moments(roi_far);

      if (M_near.m00 > this->get_parameter("min_area").as_int() && M_far.m00 > this->get_parameter("min_area").as_int()) {
        target_found = true;
        
        cv::Point p_near(M_near.m10 / M_near.m00, M_near.m01 / M_near.m00 + height * 0.8);
        cv::Point p_far(M_far.m10 / M_far.m00, M_far.m01 / M_far.m00 + height * 0.5);

        // ### MODIFICA CHIAVE: L'ERRORE ORA SI BASA SUL PUNTO VICINO (BLU) ###
        // Questo forza il robot ad allinearsi con la parte di tubo immediatamente di fronte a lui.
        pixel_error = width / 2 - p_near.x;

        // Disegna aiuti visivi sull'immagine
        cv::rectangle(frame, roi_near_rect, cv::Scalar(255, 0, 0), 2); // Blu
        cv::rectangle(frame, roi_far_rect, cv::Scalar(0, 0, 255), 2);  // Rosso
        cv::circle(frame, p_near, 8, cv::Scalar(255, 0, 0), -1);
        cv::circle(frame, p_far, 8, cv::Scalar(0, 0, 255), -1);
        cv::line(frame, p_near, p_far, cv::Scalar(0, 255, 0), 3);
      }
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge: %s", e.what());
      return;
    }

    publish_pose_setpoint(target_found, pixel_error);
    publish_velocity_setpoint(target_found);
    
    if (cv_ptr) {
      processed_image_pub_->publish(*cv_ptr->toImageMsg());
    }
  }

  void publish_pose_setpoint(bool target_is_visible, int pixel_err)
  {
    auto pose_msg = std::make_unique<PoseStampedMsg>();
    pose_msg->header.stamp = this->get_clock()->now();
    pose_msg->header.frame_id = this->get_parameter("world_frame_id").as_string();

    tf2::Quaternion current_q;
    tf2::fromMsg(current_odometry_->pose.pose.orientation, current_q);
    double roll, pitch, current_yaw;
    tf2::Matrix3x3(current_q).getRPY(roll, pitch, current_yaw);

    if (!initial_yaw_.has_value()) {
      initial_yaw_ = current_yaw;
      RCLCPP_INFO(this->get_logger(), "Rotta iniziale ancorata a: %.2f gradi", initial_yaw_.value() * 180.0 / M_PI);
    }
    
    double target_yaw;
    double depth_setpoint = this->get_parameter("depth_setpoint").as_double();
    double depth_tolerance = this->get_parameter("depth_tolerance").as_double();
    bool at_target_depth = std::abs(depth_setpoint - current_odometry_->pose.pose.position.z) < depth_tolerance;

    if (!at_target_depth) {
      target_yaw = initial_yaw_.value();
    } else {
      if (target_is_visible) {
        double yaw_correction = this->get_parameter("yaw_correction_gain").as_double() * pixel_err;
        target_yaw = current_yaw + yaw_correction;
      } else {
        target_yaw = current_yaw;
      }
    }
    
    tf2::Quaternion setpoint_q;
    setpoint_q.setRPY(0.0, 0.0, target_yaw);

    pose_msg->pose.position.x = current_odometry_->pose.pose.position.x;
    pose_msg->pose.position.y = current_odometry_->pose.pose.position.y;
    pose_msg->pose.position.z = depth_setpoint;
    pose_msg->pose.orientation = tf2::toMsg(setpoint_q);

    pose_pub_->publish(std::move(pose_msg));
  }

  void publish_velocity_setpoint(bool target_is_found)
  {
    auto twist_msg = std::make_unique<TwistStampedMsg>();
    twist_msg->header.stamp = this->get_clock()->now();
    twist_msg->header.frame_id = this->get_parameter("robot_frame_id").as_string();

    twist_msg->twist.linear.x = 0.0;
    twist_msg->twist.linear.y = 0.0;
    twist_msg->twist.linear.z = 0.0;
    twist_msg->twist.angular.x = 0.0;
    twist_msg->twist.angular.y = 0.0;
    twist_msg->twist.angular.z = 0.0;

    double depth_setpoint = this->get_parameter("depth_setpoint").as_double();
    double current_depth = current_odometry_->pose.pose.position.z;
    double depth_tolerance = this->get_parameter("depth_tolerance").as_double();
    bool at_target_depth = std::abs(depth_setpoint - current_depth) < depth_tolerance;

    if (at_target_depth) {
      if (target_is_found) {
        twist_msg->twist.linear.x = this->get_parameter("constant_forward_speed").as_double();
      } else {
        twist_msg->twist.angular.z = this->get_parameter("search_yaw_velocity").as_double();
      }
    }
    
    velocity_pub_->publish(std::move(twist_msg));
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousCylinderTracker>());
  rclcpp::shutdown();
  return 0;
}