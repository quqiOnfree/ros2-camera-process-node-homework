#include <cstddef>
#include <cstdio>
#include <format>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

#include "detect/number_detector.hpp"

class DetectNode : public rclcpp::Node {
public:
  DetectNode() : Node("detect_node") {
    using std::placeholders::_1;
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&DetectNode::imageCallback, this, _1));

    digit_class_pub_ =
        this->create_publisher<std_msgs::msg::Int32>("/digit_class", 10);
    digit_score_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("/digit_score", 10);

    RCLCPP_INFO(this->get_logger(),
                "Loading template images for number detection...");
    for (std::size_t i = 0; i < 10; ++i) {
      cv::Mat src;
      std::string path = std::format("detect/templates/{}white.png", i);
      src = cv::imread(path);
      if (src.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to load image: %s",
                    path.c_str());
        continue;
      }
      number_detector_.add(i, src);
    }

    RCLCPP_INFO(this->get_logger(),
                "detect_node started, subscribing to /camera1/image_raw, "
                "publishing to /digit_class and /digit_score");
  }

  virtual ~DetectNode() = default;

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat mat;
    int type = 0;
    if (msg->encoding == "rgb8" || msg->encoding == "bgr8") {
      type = CV_8UC3;
    } else if (msg->encoding == "mono8") {
      type = CV_8UC1;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s",
                  msg->encoding.c_str());
      return;
    }
    mat = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width),
                  type, msg->data.data(), msg->step};
    if (msg->encoding == "rgb8") {
      cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    }
    std::vector<cv::Point> points;
    if (!number_detector_.detect(mat, points)) {
      return;
    }
    std_msgs::msg::Int32 digit_class_msg;
    std_msgs::msg::Float32 digit_score_msg;
    std::uint8_t num;
    double confidence;
    number_detector_.decode(mat, points, num, confidence);
    RCLCPP_INFO(this->get_logger(), "Detected digit: %d with confidence: %f",
                num, confidence);
    digit_class_msg.data = num;
    digit_score_msg.data = static_cast<float>(confidence);
    digit_class_pub_->publish(digit_class_msg);
    digit_score_pub_->publish(digit_score_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr digit_class_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr digit_score_pub_;
  inline static NumberDetector number_detector_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
