#include <auv_control/controller_io.h>
#include <Eigen/Dense>

// Spazio dei nomi
using namespace auv_control;

class HinfController : public ControllerIO
{
public:
  // Matrice di guadagno
  Eigen::Matrix<double, 6, 24> K_gain_matrix_;
  
  // Vettore per l'errore integrale
  Eigen::Matrix<double, 12, 1> int_error_;

  // Limiti per l'anti-windup dell'errore integrale
  Eigen::Matrix<double, 12, 1> integral_limits_;

  // --- CORREZIONE: Usa il tipo Vector6d definito in auv_control, non Eigen ---
  Vector6d wrench_saturation_limits_;

  // Per calcolare il dt dinamico
  rclcpp::Time last_time_;

  // Costruttore
  HinfController() : ControllerIO("hinf_controller")
  {
    // --- INIZIALIZZA LA TUA MATRICE DI GUADAGNO K ---
    K_gain_matrix_ <<
    -42.92, 0.01, -0.02, -0.00, 0.00, 0.00, -39.44, 0.02, -0.03, 0.01, 0.00, -0.00, -88.62, 0.03, -0.03, -0.05, 0.10, 0.01, -113.34, 0.04, -0.02, -0.03, 1.34, -0.00, 0.01, -42.36, 0.02, -0.10, 0.00, 0.00, 0.04, -41.15, -0.02, -0.15, -0.00, 0.00, 0.02, -90.24, 0.03, -0.28, 0.00, -0.00, 0.04, -111.75, 0.01, -0.24, -0.00, 0.00, -0.01, 0.01, -42.09, 0.01, 0.00, 0.01, -0.01, -0.01, -40.52, -0.02, 0.00, -0.01, -0.02, 0.01, -89.24, 0.01, 0.00, 0.00, -0.02, 0.01, -102.23, 0.00, 0.00, 0.00, -0.01, -0.08, -0.00, -36.29, 0.00, 0.00, -0.01, -0.12, -0.02, -28.90, 0.00, -0.00, -0.01, -0.25, 0.01, -71.34, 0.00, 0.01, -0.01, -0.18, -0.01, -90.57, 0.00, 0.00, -3.85, -0.02, -0.00, -0.02, -0.00, 0.00, -3.01, -0.02, 0.00, 0.00, -0.00, -0.01, -7.03, -0.03, -0.01, -0.00, -2.84, 0.00, -8.88, -0.02, -0.01, -0.00, -0.04, -0.01, -0.00, -0.00, -0.01, -0.00, -0.00, -35.89, -0.00, 0.00, -0.00, -0.00, -0.00, -28.90, -0.00, 0.00, -0.01, 0.00, -0.00, -68.56, -0.00, 0.01, -0.01, -0.00, 0.00, -97.47;

    // Inizializza l'errore integrale a zero
    int_error_.setZero();
    
    // IMPOSTA I LIMITI PER L'ANTI-WINDUP (da tarare)
    double pos_int_limit = 10.0;
    double orient_int_limit = 5.0;
    double vel_int_limit = 10.0;
    double ang_vel_int_limit = 5.0;
    integral_limits_ << pos_int_limit, pos_int_limit, pos_int_limit, orient_int_limit, orient_int_limit, orient_int_limit,
                        vel_int_limit, vel_int_limit, vel_int_limit, ang_vel_int_limit, ang_vel_int_limit, ang_vel_int_limit;

    // INIZIALIZZA I LIMITI DI SATURAZIONE DELLO SFORZO
    // Valori presi dal file YAML per `u_sat` in ordine [x, y, z, roll, pitch, yaw]
    wrench_saturation_limits_ <<
      113.137,  // x (Forza su surge)
      113.137,  // y (Forza su sway)
      80.0,     // z (Forza su heave)
      17.042,   // roll (Coppia su roll)
      8.402,    // pitch (Coppia su pitch)
      27.322;   // yaw (Coppia su yaw)

    // Inizializza il tempo per il calcolo del dt
    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "HinfController initializzato con limiti di saturazione.");
  }

  Vector6d computeWrench(const Vector6d &se3_error,
                           const Vector6d &vel,
                           const Vector6d &vel_setpoint) override
  {
    // 1. Calcola il dt dinamico
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    if (dt <= 0 || dt > 0.5) {
      RCLCPP_WARN(this->get_logger(), "Delta time anomalo: %.4f s. Salto il calcolo.", dt);
      return Vector6d::Zero();
    }

    // 2. Definisci il vettore di errore proporzionale
    Vector6d velocity_error = vel_setpoint - vel;
    Eigen::Matrix<double, 12, 1> proportional_error;
    proportional_error << -se3_error, -velocity_error;

    // 3. Aggiorna l'errore integrale
    int_error_ += proportional_error * dt;

    // 4. Applica l'ANTI-WINDUP sull'errore integrale
    int_error_ = int_error_.cwiseMax(-integral_limits_).cwiseMin(integral_limits_);

    // 5. Costruisci il vettore di stato completo
    Eigen::Matrix<double, 24, 1> state_vector;
    state_vector << int_error_, proportional_error;

    // 6. Calcola l'ingresso di controllo
    Vector6d control_wrench = K_gain_matrix_ * state_vector;

    // 7. Applica la saturazione all'uscita (wrench)
    // Limita lo sforzo calcolato per non superare i limiti fisici degli attuatori
    control_wrench = control_wrench.cwiseMax(-wrench_saturation_limits_).cwiseMin(wrench_saturation_limits_);

    // Log per debugging
    RCLCPP_INFO(this->get_logger(), "Saturated Wrench: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
               control_wrench(0), control_wrench(1), control_wrench(2), control_wrench(3), control_wrench(4), control_wrench(5));

    return control_wrench;
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HinfController>());
  rclcpp::shutdown();
  return 0;
}