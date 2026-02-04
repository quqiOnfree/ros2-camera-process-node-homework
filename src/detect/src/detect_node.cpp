#include <cstdio>
#include <format>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class DetectNode : public rclcpp::Node {
public:
  DetectNode() : Node("detect_node") {
    using std::placeholders::_1;
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&DetectNode::imageCallback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "detect_node started, subscribing to /camera1/image_raw");
  }

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
    RCLCPP_INFO(this->get_logger(), "Received image: %dx%d, channels=%d",
                mat.cols, mat.rows, mat.channels());
    cv::Mat gray;
    cv::cvtColor(mat, gray, cv::COLOR_BGR2GRAY);
    cv::Mat blur;
    cv::GaussianBlur(gray, blur, cv::Size{5, 5}, 0);
    cv::adaptiveThreshold(blur, blur, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, 11, 2);
    cv::Mat edges;
    cv::Canny(blur, edges, 50, 150);
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size{5, 5}));
    cv::morphologyEx(edges, edges, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size{3, 3}));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "Found %zu contours", contours.size());
    cv::drawContours(mat, contours, -1, {255, 0, 255}, 2);
    cv::imwrite(std::format("build/image_{}.png",
                            this->get_clock()->now().nanoseconds()),
                mat);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
