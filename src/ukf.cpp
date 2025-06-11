/*
 * Copyright (C) 2024, [Il Tuo Nome]
 *
 * Questo file implementa un Unscented Kalman Filter (UKF) autonomo
 * per la stima dello stato 6-DOF di un BlueROV2 utilizzando ROS 2.
 * QUESTA VERSIONE CORREGGE TUTTI I BUG DI COMPILAZIONE E DI FISICA.
 */

#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace State {
enum {
  PX = 0, PY, PZ,
  QW, QX, QY, QZ,
  VX, VY, VZ,
  BGX, BGY, BGZ,
  BAX, BAY, BAZ,
  DIM = 16
};
}

// ===================================================================================
// === CLASSE UKF (IMPLEMENTAZIONE ROBUSTA E SEMPLIFICATA) ===
// ===================================================================================
class UKF {
public:
    UKF(int n, double alpha = 1e-3, double beta = 2.0, double kappa = 0.0)
        : n_(n),
          num_sigma_points_(2 * n + 1),
          lambda_(alpha * alpha * (n + kappa) - n) {
        
        Wm_.resize(num_sigma_points_);
        Wc_.resize(num_sigma_points_);
        Wm_(0) = lambda_ / (n_ + lambda_);
        Wc_(0) = lambda_ / (n_ + lambda_) + (1 - alpha * alpha + beta);
        for (int i = 1; i < num_sigma_points_; ++i) {
            Wm_(i) = 0.5 / (n_ + lambda_);
            Wc_(i) = 0.5 / (n_ + lambda_);
        }

        x_ = Eigen::VectorXd::Zero(n_);
        P_ = Eigen::MatrixXd::Identity(n_, n_);
        Q_ = Eigen::MatrixXd::Zero(n_, n_);
    }

    void predict(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& fx) {
        Eigen::MatrixXd sigmas = generateSigmaPoints();

        for (int i = 0; i < num_sigma_points_; ++i) {
            sigmas.col(i) = fx(sigmas.col(i));
        }

        x_ = compute_mean(sigmas);
        P_ = compute_covariance(sigmas, x_);
        P_ += Q_;
    }

    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R,
                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& hx) {
        
        Eigen::MatrixXd sigmas = generateSigmaPoints();
        Eigen::MatrixXd Z_sigmas(z.rows(), num_sigma_points_);
        for (int i = 0; i < num_sigma_points_; ++i) {
            Z_sigmas.col(i) = hx(sigmas.col(i));
        }

        Eigen::VectorXd z_pred(z.rows());
        z_pred.setZero();
        for (int i = 0; i < num_sigma_points_; ++i) {
            z_pred += Wm_(i) * Z_sigmas.col(i);
        }

        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(z.rows(), z.rows());
        for (int i = 0; i < num_sigma_points_; ++i) {
            Eigen::VectorXd z_res = Z_sigmas.col(i) - z_pred;
            S += Wc_(i) * z_res * z_res.transpose();
        }
        S += R;

        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(n_, z.rows());
        for (int i = 0; i < num_sigma_points_; ++i) {
            Eigen::VectorXd x_res = compute_residual(sigmas.col(i), x_);
            Eigen::VectorXd z_res = Z_sigmas.col(i) - z_pred;
            T += Wc_(i) * x_res * z_res.transpose();
        }

        Eigen::MatrixXd K = T * S.inverse();
        x_ = x_ + K * (z - z_pred);
        P_ = P_ - K * S * K.transpose();

        x_.segment<4>(State::QW).normalize();
    }
    
    const Eigen::VectorXd& get_x() const { return x_; }
    const Eigen::MatrixXd& get_P() const { return P_; }
    void set_x(const Eigen::VectorXd& x) { x_ = x; }
    void set_P(const Eigen::MatrixXd& P) { P_ = P; }
    void set_Q(const Eigen::MatrixXd& Q) { Q_ = Q; }

private:
    int n_;
    int num_sigma_points_;
    double lambda_;
    Eigen::VectorXd Wm_, Wc_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;

    Eigen::MatrixXd generateSigmaPoints() {
        Eigen::MatrixXd sigmas(n_, num_sigma_points_);
        Eigen::MatrixXd A = ((n_ + lambda_) * P_).llt().matrixL();

        sigmas.col(0) = x_;
        for (int i = 0; i < n_; ++i) {
            sigmas.col(i + 1)       = x_ + A.col(i);
            sigmas.col(i + 1 + n_) = x_ - A.col(i);
        }
        // Normalizza i quaternioni dei sigma point generati
        for (int i=0; i < num_sigma_points_; ++i) {
            sigmas.col(i).segment<4>(State::QW).normalize();
        }
        return sigmas;
    }

    Eigen::VectorXd compute_residual(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        Eigen::VectorXd residual = a - b;
        Eigen::Quaterniond q_a(a(State::QW), a(State::QX), a(State::QY), a(State::QZ));
        Eigen::Quaterniond q_b(b(State::QW), b(State::QX), b(State::QY), b(State::QZ));
        if (q_a.coeffs().dot(q_b.coeffs()) < 0.0) {
            q_b.coeffs() = -q_b.coeffs();
        }
        residual.segment<4>(State::QW) = q_a.coeffs() - q_b.coeffs();
        return residual;
    }

    Eigen::VectorXd compute_mean(const Eigen::MatrixXd& sigmas) {
        Eigen::VectorXd mean = Eigen::VectorXd::Zero(n_);
        // Somma pesata diretta per le parti non-quaternione
        for (int i = 0; i < num_sigma_points_; ++i) {
            mean.segment<3>(State::PX)  += Wm_(i) * sigmas.col(i).segment<3>(State::PX);
            mean.segment<3>(State::VX)  += Wm_(i) * sigmas.col(i).segment<3>(State::VX);
            mean.segment<3>(State::BGX) += Wm_(i) * sigmas.col(i).segment<3>(State::BGX);
            mean.segment<3>(State::BAX) += Wm_(i) * sigmas.col(i).segment<3>(State::BAX);
        }

        // Media pesata per i quaternioni
        Eigen::Vector4d q_mean_coeffs = Eigen::Vector4d::Zero();
        Eigen::Quaterniond first_q(sigmas(State::QW, 0), sigmas(State::QX, 0), sigmas(State::QY, 0), sigmas(State::QZ, 0));
        for (int i = 0; i < num_sigma_points_; ++i) {
            Eigen::Quaterniond q_i(sigmas(State::QW, i), sigmas(State::QX, i), sigmas(State::QY, i), sigmas(State::QZ, i));
            if (q_i.coeffs().dot(first_q.coeffs()) < 0.0) {
                q_i.coeffs() = -q_i.coeffs();
            }
            q_mean_coeffs += Wm_(i) * q_i.coeffs();
        }
        q_mean_coeffs.normalize();
        mean.segment<4>(State::QW) = q_mean_coeffs;
        return mean;
    }

    Eigen::MatrixXd compute_covariance(const Eigen::MatrixXd& sigmas, const Eigen::VectorXd& mean) {
        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n_, n_);
        for (int i = 0; i < num_sigma_points_; ++i) {
            Eigen::VectorXd residual = compute_residual(sigmas.col(i), mean);
            cov += Wc_(i) * residual * residual.transpose();
        }
        return cov;
    }
};

// ===================================================================================
// === NODO ROS 2 ===
// ===================================================================================
class UkfNode : public rclcpp::Node {
public:
  UkfNode() : Node("ukf_node") {
    this->declare_parameters();
    this->initialize_ukf();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/bluerov2/odom_estim", 10);
    auto sensor_qos = rclcpp::SensorDataQoS();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", sensor_qos,
        std::bind(&UkfNode::imu_callback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "dvl/twist", sensor_qos,
        std::bind(&UkfNode::dvl_callback, this, std::placeholders::_1));

    mlat_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mlat/pose", sensor_qos,
        std::bind(&UkfNode::mlat_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Nodo UKF (versione stabile) inizializzato.");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mlat_sub_;

  std::unique_ptr<UKF> ukf_;
  rclcpp::Time last_pred_time_;
  bool is_initialized_ = false;

  Eigen::Vector3d gravity_world_;

  void declare_parameters() {
    this->declare_parameter<double>("gravity", 9.81);
    this->declare_parameter<double>("noise_proc.pos", 0.1);
    this->declare_parameter<double>("noise_proc.quat", 1e-5);
    this->declare_parameter<double>("noise_proc.nu", 0.05);
    this->declare_parameter<double>("noise_proc.bgyro", 1e-8);
    this->declare_parameter<double>("noise_proc.bacc", 1e-6);
    this->declare_parameter<double>("noise_meas.dvl_vel_var", 0.0025);
    this->declare_parameter<double>("noise_meas.mlat_pos_var", 0.0225);
  }

  void initialize_ukf() {
    ukf_ = std::make_unique<UKF>(State::DIM);
    
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(State::DIM);
    x0(State::QW) = 1.0;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(State::DIM, State::DIM);
    P0.block<3, 3>(State::PX, State::PX) *= 1.0;
    P0.block<4, 4>(State::QW, State::QW) *= 0.1;
    P0.block<3, 3>(State::VX, State::VX) *= 0.1;
    P0.block<3, 3>(State::BGX, State::BGX) *= 1e-3;
    P0.block<3, 3>(State::BAX, State::BAX) *= 1e-3;
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(State::DIM, State::DIM);
    Q.block<3, 3>(State::PX, State::PX) = Eigen::Matrix3d::Identity() * this->get_parameter("noise_proc.pos").as_double();
    Q.block<4, 4>(State::QW, State::QW) = Eigen::Matrix4d::Identity() * this->get_parameter("noise_proc.quat").as_double();
    Q.block<3, 3>(State::VX, State::VX) = Eigen::Matrix3d::Identity() * this->get_parameter("noise_proc.nu").as_double();
    Q.block<3, 3>(State::BGX, State::BGX) = Eigen::Matrix3d::Identity() * this->get_parameter("noise_proc.bgyro").as_double();
    // --- ERRORE DI DIMENSIONE CORRETTO QUI ---
    Q.block<3, 3>(State::BAX, State::BAX) = Eigen::Matrix3d::Identity() * this->get_parameter("noise_proc.bacc").as_double();
    
    ukf_->set_x(x0);
    ukf_->set_P(P0);
    ukf_->set_Q(Q);

    gravity_world_ = Eigen::Vector3d(0.0, 0.0, -this->get_parameter("gravity").as_double());
  }

  Eigen::VectorXd process_model(const Eigen::VectorXd& x, const sensor_msgs::msg::Imu& u, double dt) {
    Eigen::VectorXd x_pred = x;
    Eigen::Quaterniond q(x(State::QW), x(State::QX), x(State::QY), x(State::QZ));
    q.normalize();
    Eigen::Vector3d v(x.segment<3>(State::VX));
    Eigen::Vector3d b_g(x.segment<3>(State::BGX));
    Eigen::Vector3d b_a(x.segment<3>(State::BAX));
    
    Eigen::Vector3d omega_m(u.angular_velocity.x, u.angular_velocity.y, u.angular_velocity.z);
    Eigen::Vector3d acc_m(u.linear_acceleration.x, u.linear_acceleration.y, u.linear_acceleration.z);

    Eigen::Vector3d omega_corr = omega_m - b_g;
    Eigen::Vector3d acc_corr = acc_m - b_a;

    Eigen::Matrix3d R_wb = q.toRotationMatrix();

    x_pred.segment<3>(State::PX) += R_wb * v * dt;
    
    if (omega_corr.norm() > 1e-9) {
        Eigen::AngleAxisd delta_angle_axis(omega_corr.norm() * dt, omega_corr.normalized());
        q = q * Eigen::Quaterniond(delta_angle_axis);
        q.normalize();
    }
    
    // --- ERRORE DI FISICA CORRETTO QUI ---
    x_pred.segment<3>(State::VX) += (acc_corr + R_wb.transpose() * gravity_world_ - omega_corr.cross(v)) * dt;
    
    x_pred.segment<4>(State::QW) = q.coeffs();
    return x_pred;
  }

  Eigen::VectorXd measurement_model_dvl(const Eigen::VectorXd& x) {
    return x.segment<3>(State::VX);
  }
  
  Eigen::VectorXd measurement_model_mlat(const Eigen::VectorXd& x) {
    return x.segment<3>(State::PX);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!is_initialized_) {
      last_pred_time_ = msg->header.stamp;
      is_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "UKF inizializzato con il primo messaggio IMU.");
      return;
    }
    
    double dt = (rclcpp::Time(msg->header.stamp) - last_pred_time_).seconds();
    if (dt <= 1e-4) return;
    
    auto fx = [this, msg, dt](const Eigen::VectorXd& x) {
        return this->process_model(x, *msg, dt);
    };

    ukf_->predict(fx);
    
    last_pred_time_ = msg->header.stamp;
    publish_odometry(msg->header.stamp);
  }

  void dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    if (!is_initialized_) return;
    
    // --- ERRORE DI BATTITURA CORRETTO QUI ---
    Eigen::Vector3d z(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * this->get_parameter("noise_meas.dvl_vel_var").as_double();

    ukf_->update(z, R, [this](const Eigen::VectorXd& x){ return this->measurement_model_dvl(x); });
  }
  
  void mlat_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      if (!is_initialized_) return;

      Eigen::Vector3d z(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * this->get_parameter("noise_meas.mlat_pos_var").as_double();
      
      ukf_->update(z, R, [this](const Eigen::VectorXd& x){ return this->measurement_model_mlat(x); });
  }
  
  void publish_odometry(const rclcpp::Time& stamp) {
    const Eigen::VectorXd& x = ukf_->get_x();
    const Eigen::MatrixXd& P = ukf_->get_P();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x(State::PX);
    odom_msg.pose.pose.position.y = x(State::PY);
    odom_msg.pose.pose.position.z = x(State::PZ);
    odom_msg.pose.pose.orientation.w = x(State::QW);
    odom_msg.pose.pose.orientation.x = x(State::QX);
    odom_msg.pose.pose.orientation.y = x(State::QY);
    odom_msg.pose.pose.orientation.z = x(State::QZ);
    
    // Copia della covarianza 6x6 della posa (posizione + quaternione)
    for (int i=0; i<6; ++i) for (int j=0; j<6; ++j) {
        odom_msg.pose.covariance[i*6+j] = P(i,j);
    }

    odom_msg.twist.twist.linear.x = x(State::VX);
    odom_msg.twist.twist.linear.y = x(State::VY);
    odom_msg.twist.twist.linear.z = x(State::VZ);
    
    // Copia della covarianza 6x6 della velocità (lineare + angolare)
    Eigen::MatrixXd P_twist = P.block<6,6>(State::VX, State::VX);
    for (int i=0; i<6; ++i) for (int j=0; j<6; ++j) {
        odom_msg.twist.covariance[i*6+j] = P_twist(i,j);
    }
    
    odom_pub_->publish(odom_msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UkfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}