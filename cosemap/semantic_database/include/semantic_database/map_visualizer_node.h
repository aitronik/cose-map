#include <rclcpp/rclcpp.hpp>
#include <iostream>


#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "cosemap_tools/base_node/base_node.h"
#include "cosemap_tools/map_loader/map_loader.h"
#include "cosemap_tools/sql_helper/sql_helper.h"

/* Node that manage the database from the Semantic  */
class MapVisualizerNode : public BaseNode{

public:
    /**
     * @brief Construct a new Semantic Database Node object
     * 
     */
    MapVisualizerNode();

    /**
     * @brief Destroy the Ground Truth Node object
     * 
     */
    ~MapVisualizerNode();

    /* Service to to make query to database */    
    /* Filtered map publisher*/
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr m_mapPub;

    /* objects bounfing boxes array publisher */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_boxesPub;
    /* objects centroids array publisher*/
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_centroidsPub;
    /* objects labels array publisher*/
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_labelsPub;

private:

    /**
     * @brief Initialize node 
     */
    void init() override;

    /**
     * @brief Step function
     * 
     */
    void step() override;


    /* filtered map frame id */
    std::string m_mapFrameId;
    
    MapLoader m_mapLoader;
    sqlHelper m_sqlConn;

};