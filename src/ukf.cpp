#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Usiamo degli alias per rendere il codice più leggibile
using ImuMsg = sensor_msgs::msg::Imu;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using OdometryMsg = nav_msgs::msg::Odometry;

// Definiamo la classe del nostro nodo
class UkfNode : public rclcpp::Node
{
public:
    // Il costruttore del nodo
    UkfNode() : Node("ukf_node") // Il nome del nodo deve corrispondere a quello nel file di lancio
    {
        // Creiamo il publisher per l'output del filtro.
        // Il nome del topic 'odometry/filtered' è quello di default, che può essere
        // rimappato dal file di lancio se necessario.
        odometry_publisher_ = this->create_publisher<OdometryMsg>("odometry/filtered", 10);

        // Creiamo i subscriber per gli input del filtro.
        // Questi sono i nomi INTERNI che verranno rimappati dal file di lancio.
        imu_subscription_ = this->create_subscription<ImuMsg>(
            "imu/data", 10, std::bind(&UkfNode::imu_callback, this, std::placeholders::_1));
            
        dvl_subscription_ = this->create_subscription<TwistStampedMsg>(
            "dvl/twist", 10, std::bind(&UkfNode::dvl_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nodo Scheletro UKF avviato con successo.");
        RCLCPP_INFO(this->get_logger(), "Iscritto a '%s' e '%s'", imu_subscription_->get_topic_name(), dvl_subscription_->get_topic_name());
        RCLCPP_INFO(this->get_logger(), "Pubblica su '%s'", odometry_publisher_->get_topic_name());
    }

private:
    // Funzioni di callback (intenzionalmente vuote per ora)
    void imu_callback(const ImuMsg::SharedPtr msg)
    {
        // La logica di elaborazione del messaggio IMU andrà qui.
        (void)msg; // Dice al compilatore che sappiamo di non usare il parametro, per evitare warning.
    }

    void dvl_callback(const TwistStampedMsg::SharedPtr msg)
    {
        // La logica di elaborazione del messaggio DVL andrà qui.
        (void)msg;
    }

    // Dichiarazione dei publisher e subscriber
    rclcpp::Publisher<OdometryMsg>::SharedPtr odometry_publisher_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscription_;
    rclcpp::Subscription<TwistStampedMsg>::SharedPtr dvl_subscription_;
};

// Funzione main, il punto di ingresso del programma
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Crea un'istanza del nostro nodo e la mette in esecuzione
  rclcpp::spin(std::make_shared<UkfNode>());
  rclcpp::shutdown();
  return 0;
}