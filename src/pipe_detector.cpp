#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <iomanip> // Per std::setw e std::setfill

// Usiamo un alias per il messaggio Image per scrivere meno codice
using ImageMsg = sensor_msgs::msg::Image;

class CylinderDetector : public rclcpp::Node
{
public:
  CylinderDetector() : Node("cylinder_detector_node")
  {
    // --- Parametri ---
    this->declare_parameter<std::string>("image_topic", "/bluerov2/image");
    
    // Parametri per il rilevamento del colore nero in HSV
    // Il nero è definito da un basso valore di 'Value' (luminosità).
    this->declare_parameter<int>("hsv_v_low", 0);
    this->declare_parameter<int>("hsv_v_high", 50); // Valore da calibrare, 50 è un buon punto di partenza
    this->declare_parameter<int>("min_area", 1000); // Area minima in pixel per considerare un oggetto valido

    // --- Lettura dei Parametri ---
    std::string image_topic = this->get_parameter("image_topic").as_string();
    hsv_v_low_ = this->get_parameter("hsv_v_low").as_int();
    hsv_v_high_ = this->get_parameter("hsv_v_high").as_int();
    min_area_ = this->get_parameter("min_area").as_int();

    // --- Sottoscrizione e Pubblicazione ---
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    subscription_ = this->create_subscription<ImageMsg>(
      image_topic, qos, std::bind(&CylinderDetector::image_callback, this, std::placeholders::_1));

    // Creiamo un publisher per l'immagine con il rilevamento visualizzato
    processed_image_pub_ = this->create_publisher<ImageMsg>("/cylinder_detector/processed_image", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo Cylinder Detector (C++) avviato.");
    RCLCPP_INFO(this->get_logger(), "In ascolto sul topic: '%s'", image_topic.c_str());
  }

private:
  void image_callback(const ImageMsg::SharedPtr msg)
  {
    try
    {
      // Converti il messaggio ROS in un'immagine OpenCV
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat& frame = cv_ptr->image;

      // 1. Converti l'immagine in HSV
      cv::Mat hsv_frame;
      cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

      // 2. Crea una maschera per il colore nero
      // Il nero ha una bassa 'Value'. Hue e Saturation possono variare.
      cv::Scalar lower_black = cv::Scalar(0, 0, hsv_v_low_);
      cv::Scalar upper_black = cv::Scalar(180, 255, hsv_v_high_);
      cv::Mat mask;
      cv::inRange(hsv_frame, lower_black, upper_black, mask);

      // 3. Trova i contorni nella maschera
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // 4. Se vengono trovati contorni, analizza il più grande
      if (!contours.empty()) {
        // Trova il contorno con l'area maggiore
        double max_area = 0;
        int largest_contour_idx = -1;
        for (size_t i = 0; i < contours.size(); i++) {
          double area = cv::contourArea(contours[i]);
          if (area > max_area) {
            max_area = area;
            largest_contour_idx = i;
          }
        }

        // 5. Se l'area del contorno più grande supera la soglia minima...
        if (max_area > min_area_) {
          // Disegna un rettangolo verde attorno all'oggetto
          cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_idx]);
          cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 3);
          
          // Stampa le coordinate del centro del rettangolo
          int center_x = bounding_box.x + bounding_box.width / 2;
          int center_y = bounding_box.y + bounding_box.height / 2;
          RCLCPP_INFO(this->get_logger(), "Cilindro Rilevato! Centro a (x: %d, y: %d)", center_x, center_y);
        }
      }

      // 6. Pubblica l'immagine processata (con il rettangolo)
      processed_image_pub_->publish(*cv_ptr->toImageMsg());
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge: %s", e.what());
    }
  }

  // Variabili membro
  rclcpp::Subscription<ImageMsg>::SharedPtr subscription_;
  rclcpp::Publisher<ImageMsg>::SharedPtr processed_image_pub_;
  int hsv_v_low_, hsv_v_high_, min_area_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderDetector>());
  rclcpp::shutdown();
  return 0;
}