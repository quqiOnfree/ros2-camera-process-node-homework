#include <cstdio>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.h>
#include <usb_cam/formats/utils.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class DetectNode: rclcpp::Node {
public:
  DetectNode(): Node("detect_node") {
    
  }
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  // cv::Mat img;

  printf("hello world detect package\n");
}
