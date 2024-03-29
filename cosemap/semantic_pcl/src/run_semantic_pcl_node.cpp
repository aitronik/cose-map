#include <rclcpp/rclcpp.hpp>
#include "semantic_pcl/semantic_pcl_node.h"
#include <iostream>
#include <unistd.h>

int main (int argc, char **argv) {

    /* Initialization of the Ros Node */
    typedef SemanticPclNode NODE;     

    rclcpp::init(argc, argv); 
    auto myNode = std::make_shared<NODE>();
    myNode->run();
    return 0;
}
