#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem> // Per la gestione di path e cartelle (richiede C++17)
#include <iomanip>    // Per std::setw e std::setfill

// Usiamo un alias per il messaggio Image per scrivere meno codice
using ImageMsg = sensor_msgs::msg::Image;

class ImageSaver : public rclcpp::Node
{
public:
  ImageSaver() : Node("image_saver_node")
  {
    // --- Dichiarazione dei Parametri ---
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("output_path", "~/rov_images_cpp");
    this->declare_parameter<int>("save_every_n_frames", 30);
    this->declare_parameter<std::string>("file_format", "jpg");

    // --- Lettura dei Parametri ---
    std::string image_topic = this->get_parameter("image_topic").as_string();
    std::string output_path_str = this->get_parameter("output_path").as_string();
    save_every_n_ = this->get_parameter("save_every_n_frames").as_int();
    file_format_ = this->get_parameter("file_format").as_string();

    // --- Gestione della Cartella di Output ---
    // Espande il carattere '~' nella home directory
    if (output_path_str.rfind("~", 0) == 0) {
      output_path_str.replace(0, 1, getenv("HOME"));
    }
    output_path_ = output_path_str;
    
    // Crea la cartella di destinazione se non esiste
    if (!std::filesystem::exists(output_path_)) {
      RCLCPP_INFO(this->get_logger(), "La cartella '%s' non esiste. La creo...", output_path_.c_str());
      std::filesystem::create_directories(output_path_);
    }

    // --- Sottoscrizione ---
    // Definiamo un QoS profile adatto ai dati dei sensori
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    subscription_ = this->create_subscription<ImageMsg>(
      image_topic, qos, std::bind(&ImageSaver::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo Image Saver (C++) avviato.");
    RCLCPP_INFO(this->get_logger(), "Sottoscritto a: '%s'", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Salvataggio in: '%s'", output_path_.c_str());
  }

private:
  void topic_callback(const ImageMsg::SharedPtr msg)
  {
    received_frame_count_++;
    if (received_frame_count_ % save_every_n_ != 0) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Ricevuto un frame da salvare...");
    try
    {
      // Converti il messaggio ROS in un'immagine OpenCV
      // Usiamo toCvCopy per avere una nostra copia modificabile dell'immagine
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // Crea un nome file con un contatore formattato con zeri (es. frame_000001.jpg)
      std::stringstream ss;
      ss << "frame_" << std::setw(6) << std::setfill('0') << saved_image_count_ << "." << file_format_;
      std::string file_name = ss.str();
      std::string full_path = output_path_ + "/" + file_name;
      
      // Salva l'immagine
      if (cv::imwrite(full_path, cv_ptr->image)) {
        RCLCPP_INFO(this->get_logger(), "Immagine salvata in: %s", full_path.c_str());
        saved_image_count_++;
      } else {
        RCLCPP_WARN(this->get_logger(), "Salvataggio fallito per: %s", full_path.c_str());
      }
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge: %s", e.what());
    }
  }

  // Dichiarazione delle variabili membro
  rclcpp::Subscription<ImageMsg>::SharedPtr subscription_;
  std::string output_path_;
  std::string file_format_;
  int save_every_n_;
  long received_frame_count_ = 0;
  long saved_image_count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSaver>());
  rclcpp::shutdown();
  return 0;
}