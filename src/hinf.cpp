#include <auv_control/controller_io.h>
#include <Eigen/Dense> // Necessario per Eigen::MatrixXd e tipi simili

// Spazio dei nomi utilizzato dalla libreria auv_control
using namespace auv_control;

// Puoi rinominare la classe se lo desideri, ad esempio HinfController
class HinfController : public ControllerIO
{
public:
  // Dichiarazione del tuo guadagno K.
  // Assumiamo qui che K sia una matrice 6x6.
  // Se K fosse uno scalare, dovresti dichiararlo come: double k_gain_scalar_;
  Eigen::Matrix<double, 6, 12> K_gain_matrix_;

  // Costruttore
  HinfController() : ControllerIO("hinf_controller") // Nome del nodo ROS, puoi cambiarlo
  {
    // --- INIZIALIZZA QUI LA TUA MATRICE DI GUADAGNO K ---
    // Questo è il passaggio più importante. Devi inserire i valori
    // della matrice K che hai calcolato.

    // Esempio di inizializzazione (SOSTITUISCI CON I TUOI VALORI REALI):
    K_gain_matrix_ <<
    -2776.3,  621.2,   1.6,    29.5,   -0.0,     9.1,   -150.2,   30.2,     2.3,     0.4,     1.7,     0.0,
    667.0, -2123.7,  48.9,  -55.4,  -42.9,     0.1,     23.8, -164.0,     0.8,    -1.6,    -0.5,     0.0,
    -46.5,   38.8, -1577.9,   1.0,    9.4,    23.1,     -2.0,    1.5,  -140.5,     0.0,     0.1,     0.1,
     23.9, -295.9,  84.1, -4013.7, 160.7,    -7.0,      0.6,  -16.1,     6.3,   -31.0,     0.8,    -0.0,
     92.1, -236.9,  21.3,   163.9, -3983.7, -34.0,      4.2,  -14.3,     4.1,     0.7,   -30.6,    -0.2,
    13.9,   -5.4,  48.9,     3.7,  -33.4, -3715.9,     0.5,   -0.1,    -0.6,    -0.0,    -0.2,   -29.3;


    // Se K fosse uno scalare:
    // k_gain_scalar_ = TUO_VALORE_SCALARE_K; // Esempio: 2.5;

    // È buona norma stampare un log per confermare l'inizializzazione
    RCLCPP_INFO(this->get_logger(), "HinfController initializzato con la matrice K.");
    // std::cout << "Matrice K inizializzata:\n" << K_gain_matrix_ << std::endl; // Per debug se RCLCPP non è subito visibile
  }

  /**
   * @brief Calcola lo sforzo (wrench) da applicare all'AUV.
   *
   * @param se3_error Errore di posa (posizione e orientamento) come Vector6d.
   * Generalmente [err_x, err_y, err_z, err_roll, err_pitch, err_yaw].
   * @param vel Velocità attuale dell'AUV come Vector6d.
   * Generalmente [vx, vy, vz, vroll, vpitch, vyaw].
   * @param vel_setpoint Setpoint di velocità dell'AUV come Vector6d.
   * @return Lo sforzo calcolato (forze e coppie) come Vector6d.
   */
  Vector6d computeWrench(const Vector6d &se3_error,
                           const Vector6d &vel,
                           const Vector6d &vel_setpoint) override
    {
      // L'errore di velocità, se necessario per una legge di controllo più complessa
      Vector6d velocity_error = vel_setpoint - vel;

      // --- CALCOLO DELL'INGRESSO DI CONTROLLO ---
      // L'ingresso di controllo è K * errore.
      // Qui assumiamo che "l'errore" a cui si riferisce K sia se3_error.

      Vector6d control_wrench;

      Eigen::Matrix<double, 12, 1> error;

      error<< -se3_error, -velocity_error;

     

      // Se K è una matrice 6x6 e l'errore è se3_error (Vector6d):
      control_wrench = K_gain_matrix_ * error;

      // Se K fosse uno scalare e l'errore fosse se3_error:
      // control_wrench = k_gain_scalar_ * se3_error;

      // Se il tuo "errore" per K fosse l'errore di velocità:
      // Vector6d velocity_error = vel_setpoint - vel;
      // control_wrench = K_gain_matrix_ * velocity_error; // Se K è matrice
      // control_wrench = k_gain_scalar_ * velocity_error; // Se K è scalare

      // Log per debugging (opzionale, ma utile)
      // RCLCPP_INFO(this->get_logger(), "SE3 Error: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      //             se3_error(0), se3_error(1), se3_error(2), se3_error(3), se3_error(4), se3_error(5));
      // RCLCPP_INFO(this->get_logger(), "Computed Wrench: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      //             control_wrench(0), control_wrench(1), control_wrench(2), control_wrench(3), control_wrench(4), control_wrench(5));

      return control_wrench;
    }

private:
  // Nessun membro privato aggiuntivo necessario per questa semplice implementazione,
  // a meno che K non venga caricato come parametro ROS in modo più avanzato.
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Crea e avvia il nodo del controller
  // Assicurati che il nome della classe qui corrisponda a quello che hai definito sopra
  rclcpp::spin(std::make_shared<HinfController>());
  rclcpp::shutdown();
  return 0;
}