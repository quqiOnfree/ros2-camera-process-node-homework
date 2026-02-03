#include <cstdio>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.h>

class DetectNode: rclcpp::Node {
public:
  DetectNode(): Node("detect_node") {
    
  }
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  printf("hello world detect package\n");
}
