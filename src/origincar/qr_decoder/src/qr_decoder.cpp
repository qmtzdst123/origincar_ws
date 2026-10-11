#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp" 
#include <cv_bridge/cv_bridge.h>               
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "zbar.h"
#include <std_msgs/msg/string.hpp>
#include "origincar_msg/msg/sign.hpp" 

class QrCodeDetection : public rclcpp::Node
{
public:
  QrCodeDetection() : Node("image_subscriber")
  {
    // Subscribe to CompressedImage now
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "image", 10, std::bind(&QrCodeDetection::imageCallback, this, std::placeholders::_1));

    sign_com_pub = this->create_publisher<std_msgs::msg::String>(
      "/sign", 10); // Changed QoS to 10, a common default. Original was 1.
  }

private:
  // Callback now receives CompressedImage
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try {
      cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

      if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image.");
        return;
      }

      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

      zbar::ImageScanner scanner;
      scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
      // Create ZBar image from the grayscale OpenCV Mat
      zbar::Image zbar_image(gray.cols, gray.rows, "Y800", (uchar *)gray.data, gray.cols * gray.rows);
      
      int n = scanner.scan(zbar_image);
      if (n > 0) {
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); 
            symbol != zbar_image.symbol_end(); ++symbol) {
          const char *qrCode_msg = symbol->get_data().c_str();
          RCLCPP_INFO(this->get_logger(), "Scanned QR Code: %s", qrCode_msg);

          auto sign_com_msg = std_msgs::msg::String();
          sign_com_msg.data = qrCode_msg;

          sign_com_pub->publish(std::move(sign_com_msg));
          RCLCPP_INFO(this->get_logger(), "Published QR Data: %s", sign_com_msg.data.c_str());
        }
      } else {
        // Optional: Log if no QR codes are found in a frame
        // RCLCPP_INFO(this->get_logger(), "No QR codes found in this frame.");
      }
      
      zbar_image.set_data(NULL, 0); // Clean up zbar image data buffer
    }
    catch (const cv::Exception &e) { // Catch OpenCV specific errors
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
      return;
    }
    catch (const std::exception &e) { // Catch other standard library errors
      RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_; 
  rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_pub; 
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sign_com_pub;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QrCodeDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}