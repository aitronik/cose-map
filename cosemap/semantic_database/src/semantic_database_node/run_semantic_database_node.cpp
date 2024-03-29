#include <rclcpp/rclcpp.hpp>
#include "semantic_database/semantic_database_node.h"
#include <iostream>
#include <unistd.h>

int main (int argc, char **argv) {

    /* Initialization of the Ros Node */
    typedef SemanticDatabaseNode NODE;     

    rclcpp::init(argc, argv); 
    auto myNode = std::make_shared<NODE>();
    myNode->runLoop();
    return 0;
}