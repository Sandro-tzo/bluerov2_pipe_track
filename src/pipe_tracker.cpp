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
#include <vector>
#include <algorithm>
#include <std_srvs/srv/trigger.hpp>

// Alias per i tipi di messaggio
using ImageMsg = sensor_msgs::msg::Image;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using OdometryMsg = nav_msgs::msg::Odometry;

// Macchina a stati
enum class State {
    DIVING,
    SEARCHING,
    TRACKING,
    SURFACING,
    RETURNING_HOME,
    MISSION_COMPLETE
};

class AutonomousCylinderTracker : public rclcpp::Node
{
public:
    AutonomousCylinderTracker() : Node("autonomous_cylinder_tracker")
    {
        // Parametri
        this->declare_parameter<std::string>("image_topic");
        this->declare_parameter<int>("hsv_v_low");
        this->declare_parameter<int>("hsv_v_high");
        this->declare_parameter<int>("min_area");
        this->declare_parameter<std::string>("velocity_setpoint_topic");
        this->declare_parameter<std::string>("pose_setpoint_topic");
        this->declare_parameter<std::string>("odom_topic");
        this->declare_parameter<std::string>("robot_frame_id");
        this->declare_parameter<std::string>("world_frame_id");
        this->declare_parameter<double>("depth_setpoint");
        this->declare_parameter<double>("depth_tolerance");
        this->declare_parameter<double>("constant_forward_speed");
        this->declare_parameter<double>("yaw_correction_gain");
        this->declare_parameter<double>("search_yaw_velocity");
        this->declare_parameter<double>("surface_depth");
        this->declare_parameter<double>("surface_depth_tolerance");
        this->declare_parameter<double>("home_position_tolerance");
        this->declare_parameter<double>("return_home_step_size_m");

        // Sottoscrizioni, Pubblicazioni e Servizio
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
        auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

        image_subscription_ = this->create_subscription<ImageMsg>(this->get_parameter("image_topic").as_string(), sensor_qos, std::bind(&AutonomousCylinderTracker::image_callback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<OdometryMsg>(this->get_parameter("odom_topic").as_string(), 10, std::bind(&AutonomousCylinderTracker::odom_callback, this, std::placeholders::_1));
        processed_image_pub_ = this->create_publisher<ImageMsg>("processed_image", 10);
        velocity_pub_ = this->create_publisher<TwistStampedMsg>(this->get_parameter("velocity_setpoint_topic").as_string(), command_qos);
        pose_pub_ = this->create_publisher<PoseStampedMsg>(this->get_parameter("pose_setpoint_topic").as_string(), command_qos);
        return_home_service_ = this->create_service<std_srvs::srv::Trigger>("~/trigger_return_home", std::bind(&AutonomousCylinderTracker::handle_return_home_trigger, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Tracker Supervisionato avviato. Stato iniziale: DIVING.");
    }

private:
    rclcpp::Subscription<ImageMsg>::SharedPtr image_subscription_;
    rclcpp::Subscription<OdometryMsg>::SharedPtr odom_subscription_;
    rclcpp::Publisher<ImageMsg>::SharedPtr processed_image_pub_;
    rclcpp::Publisher<TwistStampedMsg>::SharedPtr velocity_pub_;
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_service_;
    
    std::optional<OdometryMsg> current_odometry_;
    std::optional<geometry_msgs::msg::Point> initial_position_;
    std::optional<double> initial_yaw_;

    State current_state_ = State::DIVING;
    bool target_found_ = false;

    void odom_callback(const OdometryMsg::SharedPtr msg)
    {
        current_odometry_ = *msg;
        if (!initial_position_.has_value()) {
            initial_position_ = msg->pose.pose.position;
            tf2::Quaternion q;
            tf2::fromMsg(msg->pose.pose.orientation, q);
            double r, p, y;
            tf2::Matrix3x3(q).getRPY(r, p, y);
            initial_yaw_ = y;
            RCLCPP_INFO(this->get_logger(), "Posizione iniziale salvata: X=%.2f, Y=%.2f, Yaw=%.2f deg",
                initial_position_->x, initial_position_->y, initial_yaw_.value() * 180.0 / M_PI);
        }
    }

    void image_callback(const ImageMsg::SharedPtr msg)
    {
        if (!current_odometry_.has_value()) { return; }
        
        this->target_found_ = false;
        int pixel_error = 0;
        cv_bridge::CvImagePtr cv_ptr;

        if (current_state_ == State::SEARCHING || current_state_ == State::TRACKING) {
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                cv::Mat &frame = cv_ptr->image;
                int frame_width = frame.cols;
                int frame_height = frame.rows;
                cv::Mat hsv_frame, mask;
                cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
                cv::Scalar lower_black(0, 0, this->get_parameter("hsv_v_low").as_int());
                cv::Scalar upper_black(180, 255, this->get_parameter("hsv_v_high").as_int());
                cv::inRange(hsv_frame, lower_black, upper_black, mask);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                double max_area = 0;
                std::optional<std::vector<cv::Point>> largest_contour;
                int min_area = this->get_parameter("min_area").as_int();
                for (const auto& contour : contours) {
                    double area = cv::contourArea(contour);
                    if (area > min_area && area > max_area) {
                        max_area = area;
                        largest_contour = contour;
                    }
                }
                if (largest_contour.has_value()) {
                    cv::Rect main_bbox = cv::boundingRect(largest_contour.value());
                    int roi_height = std::max(10, static_cast<int>(main_bbox.height * 0.25));
                    cv::Rect bottom_roi_rect(main_bbox.x, main_bbox.y + main_bbox.height - roi_height, main_bbox.width, roi_height);
                    bottom_roi_rect &= cv::Rect(0, 0, frame_width, frame_height);
                    cv::Mat bottom_mask_roi = mask(bottom_roi_rect);
                    std::vector<std::vector<cv::Point>> bottom_contours;
                    cv::findContours(bottom_mask_roi, bottom_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                    if (!bottom_contours.empty()) {
                        std::vector<cv::Point> all_bottom_points;
                        for(const auto& c : bottom_contours) {
                            all_bottom_points.insert(all_bottom_points.end(), c.begin(), c.end());
                        }
                        if (!all_bottom_points.empty()) {
                            this->target_found_ = true;
                            cv::Rect final_bbox = cv::boundingRect(all_bottom_points);
                            final_bbox.x += bottom_roi_rect.x;
                            final_bbox.y += bottom_roi_rect.y;
                            cv::Point reference_point(final_bbox.x + final_bbox.width / 2, final_bbox.y + final_bbox.height / 2);
                            pixel_error = frame_width / 2 - reference_point.x;
                            cv::rectangle(frame, main_bbox, cv::Scalar(0, 255, 255), 2); 
                            cv::rectangle(frame, bottom_roi_rect, cv::Scalar(255, 0, 0), 2);
                            cv::rectangle(frame, final_bbox, cv::Scalar(255, 0, 255), 2);
                            cv::circle(frame, reference_point, 12, cv::Scalar(0, 255, 0), -1);
                            cv::circle(frame, reference_point, 14, cv::Scalar(0, 0, 0), 2);
                        }
                    }
                }

            } catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge: %s", e.what());
                return;
            }
        }
        
        update_state();
        publish_pose_setpoint(pixel_error);
        publish_velocity_setpoint();
        
        if (cv_ptr) {
            processed_image_pub_->publish(*cv_ptr->toImageMsg());
        }
    }

    void update_state()
    {
        if (!current_odometry_.has_value() || !initial_position_.has_value()) return;

        switch (current_state_) {
            case State::DIVING: {
                bool at_target_depth = std::abs(current_odometry_->pose.pose.position.z - this->get_parameter("depth_setpoint").as_double()) 
                                       < this->get_parameter("depth_tolerance").as_double();
                if (at_target_depth) {
                    current_state_ = State::SEARCHING;
                    RCLCPP_INFO(this->get_logger(), "Quota raggiunta -> SEARCHING (ricerca iniziale)");
                }
                break;
            }
            case State::SEARCHING:
                if (this->target_found_) {
                    current_state_ = State::TRACKING;
                    RCLCPP_INFO(this->get_logger(), "Target trovato -> TRACKING");
                }
                break;
            case State::TRACKING:
                if (!this->target_found_) {
                    current_state_ = State::SEARCHING;
                    RCLCPP_WARN(this->get_logger(), "Target perso durante il tracking -> SEARCHING");
                }
                break;
            case State::SURFACING: {
                bool at_surface = std::abs(current_odometry_->pose.pose.position.z - this->get_parameter("surface_depth").as_double()) 
                                  < this->get_parameter("surface_depth_tolerance").as_double();
                if (at_surface) {
                    current_state_ = State::RETURNING_HOME;
                    RCLCPP_INFO(this->get_logger(), "Superficie raggiunta -> RETURNING_HOME");
                }
                break;
            }
            case State::RETURNING_HOME: {
                double dx = initial_position_->x - current_odometry_->pose.pose.position.x;
                double dy = initial_position_->y - current_odometry_->pose.pose.position.y;
                double distance_to_home = std::sqrt(dx*dx + dy*dy);
                if (distance_to_home < this->get_parameter("home_position_tolerance").as_double()) {
                    current_state_ = State::MISSION_COMPLETE;
                    RCLCPP_INFO(this->get_logger(), "Posizione iniziale raggiunta -> MISSION_COMPLETE");
                }
                break;
            }
            case State::MISSION_COMPLETE:
                break;
        }
    }

    void handle_return_home_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (current_state_ == State::SEARCHING || current_state_ == State::TRACKING) {
            RCLCPP_WARN(this->get_logger(), "Comando esterno ricevuto! Avvio sequenza di ritorno a casa.");
            current_state_ = State::SURFACING;
            response->success = true;
            response->message = "Return to home sequence initiated.";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Impossibile avviare il ritorno a casa, il robot non è in stato SEARCHING o TRACKING.");
            response->success = false;
            response->message = "Cannot initiate return, robot not in an active mission state.";
        }
    }

    void publish_pose_setpoint(int pixel_err)
    {
        if (!current_odometry_ || !initial_position_) return;
        auto pose_msg = std::make_unique<PoseStampedMsg>();
        pose_msg->header.stamp = this->get_clock()->now();
        pose_msg->header.frame_id = this->get_parameter("world_frame_id").as_string();
        tf2::Quaternion current_q;
        tf2::fromMsg(current_odometry_->pose.pose.orientation, current_q);
        double roll, pitch, current_yaw;
        tf2::Matrix3x3(current_q).getRPY(roll, pitch, current_yaw);
        pose_msg->pose.position = current_odometry_->pose.pose.position;
        pose_msg->pose.orientation = current_odometry_->pose.pose.orientation;
        switch (current_state_) {
            case State::DIVING: {
                pose_msg->pose.position.x = initial_position_->x;
                pose_msg->pose.position.y = initial_position_->y;
                pose_msg->pose.position.z = this->get_parameter("depth_setpoint").as_double();
                tf2::Quaternion q_dive;
                q_dive.setRPY(0.0, 0.0, initial_yaw_.value());
                pose_msg->pose.orientation = tf2::toMsg(q_dive);
                break;
            }
            case State::SEARCHING:
            case State::TRACKING: {
                pose_msg->pose.position.z = this->get_parameter("depth_setpoint").as_double();
                double target_yaw = current_yaw;
                if (current_state_ == State::TRACKING && this->target_found_) {
                    target_yaw += this->get_parameter("yaw_correction_gain").as_double() * pixel_err;
                }
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, target_yaw);
                pose_msg->pose.orientation = tf2::toMsg(q);
                break;
            }
            case State::SURFACING: {
                pose_msg->pose.position.z = this->get_parameter("surface_depth").as_double();
                tf2::Quaternion q_surf;
                q_surf.setRPY(0.0, 0.0, initial_yaw_.value());
                pose_msg->pose.orientation = tf2::toMsg(q_surf);
                break;
            }
            case State::RETURNING_HOME: {
                double dx = initial_position_->x - current_odometry_->pose.pose.position.x;
                double dy = initial_position_->y - current_odometry_->pose.pose.position.y;
                double distance_to_home = std::sqrt(dx*dx + dy*dy);
                double return_step_size = this->get_parameter("return_home_step_size_m").as_double();
                geometry_msgs::msg::Point target_position;
                if (distance_to_home > return_step_size) {
                    target_position.x = current_odometry_->pose.pose.position.x + (dx / distance_to_home) * return_step_size;
                    target_position.y = current_odometry_->pose.pose.position.y + (dy / distance_to_home) * return_step_size;
                } else {
                    target_position.x = initial_position_->x;
                    target_position.y = initial_position_->y;
                }
                target_position.z = this->get_parameter("surface_depth").as_double();
                pose_msg->pose.position = target_position;
                double yaw_to_home = std::atan2(dy, dx);
                tf2::Quaternion q_home;
                q_home.setRPY(0.0, 0.0, yaw_to_home);
                pose_msg->pose.orientation = tf2::toMsg(q_home);
                break;
            }
            case State::MISSION_COMPLETE: {
                pose_msg->pose.position = initial_position_.value();
                tf2::Quaternion q_final;
                q_final.setRPY(0.0, 0.0, initial_yaw_.value());
                pose_msg->pose.orientation = tf2::toMsg(q_final);
                break;
            }
        }
        pose_pub_->publish(std::move(pose_msg));
    }
    
    void publish_velocity_setpoint()
    {
        auto twist_msg = std::make_unique<TwistStampedMsg>();
        twist_msg->header.stamp = this->get_clock()->now();
        twist_msg->header.frame_id = this->get_parameter("robot_frame_id").as_string();
        twist_msg->twist.linear.x = 0.0;
        twist_msg->twist.angular.z = 0.0;
        twist_msg->twist.linear.y = 0.0;
        twist_msg->twist.linear.z = 0.0;
        twist_msg->twist.angular.x = 0.0;
        twist_msg->twist.angular.y = 0.0;
        if (current_odometry_ && current_state_ != State::MISSION_COMPLETE && current_state_ != State::DIVING) {
            if (current_state_ == State::SEARCHING) {
                twist_msg->twist.angular.z = this->get_parameter("search_yaw_velocity").as_double();
            } else if (current_state_ == State::TRACKING && this->target_found_) {
                twist_msg->twist.linear.x = this->get_parameter("constant_forward_speed").as_double();
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